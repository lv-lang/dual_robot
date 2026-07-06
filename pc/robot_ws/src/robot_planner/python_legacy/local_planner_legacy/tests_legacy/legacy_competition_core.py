import math

from local_planner.competition_core import (
    BypassSide,
    MecanumCompetitionPlannerConfig,
    MecanumCompetitionPlannerCore,
    PlannerState,
)
from local_planner.trajectory_sampler import Pose2D, Velocity2D


def _pose() -> Pose2D:
    return Pose2D(0.0, 0.0, 0.0)


def _velocity() -> Velocity2D:
    return Velocity2D(0.0, 0.0, 0.0)


def _path():
    return [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]


def test_waiting_without_path_returns_zero_command():
    planner = MecanumCompetitionPlannerCore()

    result = planner.plan(_pose(), _velocity(), [])

    assert result.valid is False
    assert result.best_cmd == Velocity2D(0.0, 0.0, 0.0)
    assert result.planner_state.state == PlannerState.WAITING
    assert result.local_trajectory == [_pose()]


def test_open_path_tracks_forward_without_ros_dependencies():
    planner = MecanumCompetitionPlannerCore()

    result = planner.plan(_pose(), _velocity(), _path())

    assert result.valid
    assert result.planner_state.state == PlannerState.TRACK_PATH
    assert result.best_cmd.vx > 0.0
    assert abs(result.best_cmd.vy) < 1e-6
    assert result.local_trajectory[-1].x > 0.0


def test_obstacle_confirmation_enters_preferred_left_sidestep():
    planner = MecanumCompetitionPlannerCore(
        MecanumCompetitionPlannerConfig(
            obstacle_confirm_frames=2,
            min_state_duration=0.0,
            preferred_bypass_side=BypassSide.LEFT,
        )
    )
    obstacle = [(0.30, 0.0)]

    first = planner.plan(_pose(), _velocity(), _path(), obstacle)
    second = planner.plan(_pose(), _velocity(), _path(), obstacle)

    assert first.planner_state.state == PlannerState.TRACK_PATH
    assert second.planner_state.state == PlannerState.SIDESTEP_LEFT
    assert second.planner_state.obstacle_confirm_count == 2
    assert second.planner_state.active_side == BypassSide.LEFT
    assert second.best_cmd.vy > 0.0


def test_blocked_stop_when_front_and_both_sides_are_blocked():
    planner = MecanumCompetitionPlannerCore(
        MecanumCompetitionPlannerConfig(
            obstacle_confirm_frames=1,
            min_state_duration=0.0,
            side_min_clearance=0.28,
        )
    )
    obstacles = [(0.20, 0.0), (0.15, 0.20), (0.15, -0.20)]

    result = planner.plan(_pose(), _velocity(), _path(), obstacles)

    assert result.planner_state.state == PlannerState.BLOCKED_STOP
    assert result.best_cmd == Velocity2D(0.0, 0.0, 0.0)


def test_clear_frames_move_from_sidestep_to_bypass_and_rejoin():
    planner = MecanumCompetitionPlannerCore(
        MecanumCompetitionPlannerConfig(
            obstacle_confirm_frames=1,
            obstacle_clear_frames=2,
            min_state_duration=0.0,
        )
    )

    sidestep = planner.plan(_pose(), _velocity(), _path(), [(0.30, 0.0)])
    first_clear = planner.plan(_pose(), _velocity(), _path(), [])
    bypass = planner.plan(_pose(), _velocity(), _path(), [])
    rejoin = planner.plan(Pose2D(0.0, 0.2, 0.0), _velocity(), _path(), [])

    assert sidestep.planner_state.state == PlannerState.SIDESTEP_LEFT
    assert first_clear.planner_state.state == PlannerState.SIDESTEP_LEFT
    assert bypass.planner_state.state == PlannerState.BYPASS_FORWARD
    assert rejoin.planner_state.state == PlannerState.REJOIN_PATH
    assert rejoin.best_cmd.vy < 0.0


def test_side_switch_cooldown_prevents_immediate_left_right_flip():
    planner = MecanumCompetitionPlannerCore(
        MecanumCompetitionPlannerConfig(
            obstacle_confirm_frames=1,
            min_state_duration=0.0,
            side_switch_cooldown=1.0,
        )
    )

    left = planner.plan(_pose(), _velocity(), _path(), [(0.30, 0.0)], dt=0.1)
    blocked_before_cooldown = planner.plan(
        _pose(),
        _velocity(),
        _path(),
        [(0.30, 0.0), (0.20, 0.20)],
        dt=0.1,
    )
    right_after_cooldown = planner.plan(
        _pose(),
        _velocity(),
        _path(),
        [(0.30, 0.0), (0.20, 0.20)],
        dt=1.0,
    )

    assert left.planner_state.state == PlannerState.SIDESTEP_LEFT
    assert blocked_before_cooldown.planner_state.state != PlannerState.SIDESTEP_RIGHT
    assert right_after_cooldown.planner_state.state == PlannerState.SIDESTEP_RIGHT
    assert right_after_cooldown.best_cmd.vy < 0.0


def test_goal_approach_limits_velocity_and_goal_reached_stops():
    planner = MecanumCompetitionPlannerCore(
        MecanumCompetitionPlannerConfig(min_state_duration=0.0, goal_approach_distance=0.5)
    )

    approach = planner.plan(Pose2D(0.7, 0.0, 0.0), _velocity(), [(0.7, 0.0), (1.0, 0.0)])
    reached = planner.plan(Pose2D(0.95, 0.0, 0.0), _velocity(), [(0.7, 0.0), (1.0, 0.0)])

    assert approach.planner_state.state == PlannerState.GOAL_APPROACH
    assert approach.best_cmd.vx <= planner.config.goal_max_vx
    assert reached.planner_state.reason == "goal_reached"
    assert reached.best_cmd == Velocity2D(0.0, 0.0, 0.0)


def test_path_accepts_pose2d_points_and_filters_invalid_obstacles():
    planner = MecanumCompetitionPlannerCore()

    result = planner.plan(
        _pose(),
        _velocity(),
        [Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0)],
        [(float("nan"), 0.0), (float("inf"), 0.0)],
    )

    assert result.valid
    assert math.isfinite(result.best_cmd.vx)
