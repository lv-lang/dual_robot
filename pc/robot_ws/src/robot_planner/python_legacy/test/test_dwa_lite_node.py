import ast
from pathlib import Path

from local_planner.dwa_lite_core import DwaLitePlannerCore
from local_planner.dwa_lite_params import DwaLiteParams
from local_planner.planner_utils import PlannerState, Pose2D, Velocity2D


ROBOT_PLANNER_DIR = Path(__file__).resolve().parents[1]
LOCAL_PLANNER_DIR = ROBOT_PLANNER_DIR / "local_planner"
NODE_FILE = LOCAL_PLANNER_DIR / "dwa_lite_node.py"
CORE_FILES = [
    LOCAL_PLANNER_DIR / "dwa_lite_core.py",
    LOCAL_PLANNER_DIR / "dwa_lite_params.py",
    LOCAL_PLANNER_DIR / "scan_obstacle_model.py",
    LOCAL_PLANNER_DIR / "footprint_checker.py",
    LOCAL_PLANNER_DIR / "trajectory_rollout.py",
    LOCAL_PLANNER_DIR / "trajectory_sampler.py",
    LOCAL_PLANNER_DIR / "trajectory_scorer.py",
    LOCAL_PLANNER_DIR / "velocity_smoother.py",
    LOCAL_PLANNER_DIR / "planner_utils.py",
]


def _tree(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def _string_literals(tree: ast.AST) -> set[str]:
    values = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            values.add(node.value)
    return values


def _attr_call_names(tree: ast.AST) -> set[str]:
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            names.add(node.func.attr)
    return names


def _self_assignments_to_create_publisher(tree: ast.AST) -> set[str]:
    publishers = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        value = node.value
        if not (
            isinstance(value, ast.Call)
            and isinstance(value.func, ast.Attribute)
            and value.func.attr == "create_publisher"
        ):
            continue
        for target in node.targets:
            if (
                isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name)
                and target.value.id == "self"
            ):
                publishers.add(target.attr)
    return publishers


def _params(**overrides) -> DwaLiteParams:
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


def _is_zero(cmd: Velocity2D) -> bool:
    return abs(cmd.vx) < 1e-9 and abs(cmd.vy) < 1e-9 and abs(cmd.wz) < 1e-9


def test_dwa_lite_core_files_are_ros_free():
    for path in CORE_FILES:
        tree = _tree(path)
        imported_modules = {
            alias.name
            for node in ast.walk(tree)
            if isinstance(node, ast.Import)
            for alias in node.names
        }
        imported_modules.update(
            node.module
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module
        )

        assert "rclpy" not in imported_modules
        assert not any(module.startswith("rclpy.") for module in imported_modules)
        assert "create_publisher" not in _attr_call_names(tree)
        assert "create_subscription" not in _attr_call_names(tree)


def test_dwa_lite_node_direct_output_topics_are_robot1_only():
    tree = _tree(NODE_FILE)
    literals = _string_literals(tree)

    assert "/robot1/cmd_vel" in literals
    assert "/robot1/local_path" in literals
    assert "/robot1/planner_state" in literals
    assert "/robot1/cmd_vel_raw" not in literals
    assert "/cmd_vel" not in literals
    assert "/scan" not in literals
    assert "/odom" not in literals


def test_dwa_lite_node_has_only_expected_publishers():
    tree = _tree(NODE_FILE)
    publishers = _self_assignments_to_create_publisher(tree)

    assert {"cmd_pub", "local_path_pub", "state_pub"}.issubset(publishers)
    assert "cmd_vel_raw_pub" not in publishers
    assert "raw_cmd_pub" not in publishers


def test_dwa_lite_node_keeps_subscriptions_in_robot1_namespace():
    tree = _tree(NODE_FILE)
    literals = _string_literals(tree)

    assert "/robot1/global_path" in literals
    assert "/robot1/odom" in literals
    assert "/robot1/scan" in literals
    assert "/global_path" not in literals


def test_dwa_lite_core_empty_path_returns_zero_command():
    result = DwaLitePlannerCore(_params()).plan(
        Pose2D(0.0, 0.0, 0.0),
        Velocity2D(0.0, 0.0, 0.0),
        [],
        [],
    )

    assert result.planner_state == PlannerState.EMPTY_PATH
    assert _is_zero(result.best_cmd)


def test_dwa_lite_core_straight_path_outputs_forward_velocity():
    result = DwaLitePlannerCore(_params()).plan(
        Pose2D(0.0, 0.0, 0.0),
        Velocity2D(0.0, 0.0, 0.0),
        [(0.0, 0.0), (1.0, 0.0)],
        [],
    )

    assert result.valid
    assert result.planner_state == PlannerState.TRACK_PATH
    assert result.best_cmd.vx > 0.0


def test_dwa_lite_core_front_obstacle_outputs_lateral_velocity():
    result = DwaLitePlannerCore(
        _params(
            path_weight=0.2,
            goal_weight=0.2,
            lateral_weight=4.0,
            front_check_distance=0.75,
            hard_stop_distance=0.1,
        )
    ).plan(
        Pose2D(0.0, 0.0, 0.0),
        Velocity2D(0.0, 0.0, 0.0),
        [(0.0, 0.0), (1.0, 0.0)],
        [(0.4, 0.0), (0.0, -0.35)],
    )

    assert result.valid
    assert result.planner_state == PlannerState.SIDESTEP_LEFT
    assert result.best_cmd.vy > 0.0


def test_dwa_lite_core_no_feasible_trajectory_returns_zero_command():
    result = DwaLitePlannerCore(_params(hard_stop_distance=-1.0)).plan(
        Pose2D(0.0, 0.0, 0.0),
        Velocity2D(0.0, 0.0, 0.0),
        [(0.0, 0.0), (1.0, 0.0)],
        [(-0.05, 0.0)],
    )

    assert not result.valid
    assert result.planner_state == PlannerState.BLOCKED_STOP
    assert result.reason == "no_feasible_trajectory"
    assert _is_zero(result.best_cmd)
