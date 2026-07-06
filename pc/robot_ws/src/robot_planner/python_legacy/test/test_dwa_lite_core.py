from pathlib import Path

from local_planner.dwa_lite_core import DwaLitePlannerCore
from local_planner.dwa_lite_params import DwaLiteParams
from local_planner.planner_utils import PlannerState, Pose2D, Velocity2D


def _fast_params(**overrides):
    values = {
        "use_dynamic_window": False,
        "cmd_filter_alpha": 1.0,
        "acc_lim_x": 10.0,
        "acc_lim_y": 10.0,
        "acc_lim_theta": 10.0,
        "jerk_lim_x": 100.0,
        "jerk_lim_y": 100.0,
        "jerk_lim_theta": 100.0,
        "vx_deadband": 0.0,
        "vy_deadband": 0.0,
        "wz_deadband": 0.0,
        "vx_samples": 4,
        "vy_samples": 5,
        "wz_samples": 3,
        "sim_time": 0.6,
        "dt": 0.2,
    }
    values.update(overrides)
    return DwaLiteParams(**values)


def _runtime_params_from_yaml(**overrides):
    config_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "robot1_dwa_lite.yaml"
    )
    values = {}
    valid_fields = set(DwaLiteParams.__dataclass_fields__)
    for line in config_path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        key, raw_value = stripped.split(":", 1)
        key = key.strip()
        raw_value = raw_value.strip()
        if key == "control_frequency":
            values["control_period"] = 1.0 / max(1.0, float(raw_value))
            continue
        if key not in valid_fields:
            continue
        if raw_value.lower() in ("true", "false"):
            values[key] = raw_value.lower() == "true"
        else:
            try:
                values[key] = int(raw_value)
            except ValueError:
                values[key] = float(raw_value)
    values.update(overrides)
    return DwaLiteParams(**values)


def test_core_reports_empty_and_stale_inputs_without_moving():
    planner = DwaLitePlannerCore(_fast_params())
    pose = Pose2D(0.0, 0.0, 0.0)
    velocity = Velocity2D(0.0, 0.0, 0.0)

    assert planner.plan(pose, velocity, [], []).planner_state == PlannerState.EMPTY_PATH
    assert planner.plan(None, None, [(1.0, 0.0)], []).planner_state == PlannerState.STALE_ODOM
    assert planner.plan(pose, velocity, [(1.0, 0.0)], [], scan_fresh=False).planner_state == PlannerState.STALE_SCAN


def test_core_tracks_open_path_with_forward_velocity():
    planner = DwaLitePlannerCore(_fast_params())

    result = planner.plan(
        Pose2D(0.0, 0.0, 0.0),
        Velocity2D(0.0, 0.0, 0.0),
        [(0.0, 0.0), (1.0, 0.0)],
        [],
    )

    assert result.valid
    assert result.planner_state == PlannerState.TRACK_PATH
    assert result.best_cmd.vx > 0.0
    assert result.best_trajectory[-1].x > 0.0


def test_core_rotates_toward_sideways_path_without_lateral_drift():
    planner = DwaLitePlannerCore(
        _fast_params(
            heading_weight=0.8,
            lateral_weight=0.65,
            velocity_weight=0.8,
            max_wz=0.42,
        )
    )

    result = planner.plan(
        Pose2D(0.0, 0.0, 0.0),
        Velocity2D(0.0, 0.0, 0.0),
        [(0.0, 0.0), (0.0, 1.0)],
        [],
    )

    assert result.valid
    assert result.planner_state == PlannerState.TRACK_PATH
    assert abs(result.best_cmd.vy) <= 0.05
    assert result.best_cmd.wz > 0.10


def test_runtime_params_ramp_heading_before_using_large_lateral_speed():
    planner = DwaLitePlannerCore(_runtime_params_from_yaml())
    pose = Pose2D(0.0, 0.0, 0.0)
    velocity = Velocity2D(0.0, 0.0, 0.0)
    result = None

    for _ in range(15):
        result = planner.plan(
            pose,
            velocity,
            [(0.0, 0.0), (0.0, 1.3)],
            [],
        )
        velocity = result.best_cmd
        pose = result.best_trajectory[1]

    assert result is not None
    assert result.valid
    assert abs(result.best_cmd.vy) <= 0.08
    assert result.best_cmd.wz > 0.10


def test_core_keeps_moving_when_obstacles_are_outside_min_clearance():
    planner = DwaLitePlannerCore(
        _fast_params(
            obstacle_min_dist=0.24,
            obstacle_weight=4.0,
        )
    )

    result = planner.plan(
        Pose2D(0.0, 0.0, 0.0),
        Velocity2D(0.0, 0.0, 0.0),
        [(0.0, 0.0), (1.0, 0.0)],
        [(0.0, 0.55), (0.0, -0.55)],
    )

    assert result.valid
    assert result.planner_state == PlannerState.TRACK_PATH
    assert result.best_cmd.vx > 0.0


def test_core_prefers_sidestep_left_when_front_blocked_and_left_side_clear():
    planner = DwaLitePlannerCore(
        _fast_params(
            path_weight=0.2,
            goal_weight=0.2,
            lateral_weight=4.0,
            front_check_distance=0.75,
            hard_stop_distance=0.1,
        )
    )

    result = planner.plan(
        Pose2D(0.0, 0.0, 0.0),
        Velocity2D(0.0, 0.0, 0.0),
        [(0.0, 0.0), (1.0, 0.0)],
        [(0.4, 0.0), (0.0, -0.35)],
    )

    assert result.valid
    assert result.planner_state == PlannerState.SIDESTEP_LEFT
    assert result.best_cmd.vy > 0.0


def test_core_hard_stops_and_reports_blocked_stop():
    planner = DwaLitePlannerCore(_fast_params(hard_stop_distance=0.1))

    result = planner.plan(
        Pose2D(0.0, 0.0, 0.0),
        Velocity2D(0.0, 0.0, 0.0),
        [(0.0, 0.0), (1.0, 0.0)],
        [(0.05, 0.0)],
    )

    assert not result.valid
    assert result.planner_state == PlannerState.BLOCKED_STOP
    assert result.best_cmd == Velocity2D(0.0, 0.0, 0.0)


def test_core_returns_goal_reached_near_final_path_point():
    planner = DwaLitePlannerCore(_fast_params(goal_tolerance=0.2))

    result = planner.plan(
        Pose2D(0.95, 0.0, 0.0),
        Velocity2D(0.1, 0.0, 0.0),
        [(0.0, 0.0), (1.0, 0.0)],
        [],
    )

    assert not result.valid
    assert result.planner_state == PlannerState.GOAL_REACHED
    assert result.best_cmd == Velocity2D(0.0, 0.0, 0.0)
