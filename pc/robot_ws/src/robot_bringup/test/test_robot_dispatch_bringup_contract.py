from pathlib import Path


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / 'launch'
    / 'robot_dispatch_gazebo.launch.py'
)


def _source() -> str:
    return LAUNCH_PATH.read_text(encoding='utf-8')


def test_task_layer_launch_includes_verified_dual_robot_stack():
    source = _source()

    assert 'dual_robot_gazebo_nav2.launch.py' in source
    assert "'rviz_config': LaunchConfiguration('rviz_config')" in source
    assert "'enable_robot1': LaunchConfiguration('enable_robot1')" in source
    assert "'enable_robot2': LaunchConfiguration('enable_robot2')" in source


def test_task_layer_launch_starts_dispatch_and_two_executors():
    source = _source()

    assert "package='robot_dispatch'" in source
    assert "executable='robot_dispatch_node'" in source
    assert "package='robot_mission'" in source
    assert "executable='mission_executor_node'" in source
    assert "'robot1_execute_mission_action': '/robot1/execute_mission'" in source
    assert "'robot2_execute_mission_action': '/robot2/execute_mission'" in source


def test_terminal_console_is_documented_but_not_auto_launched():
    source = _source()

    assert 'ros2 run robot_dispatch terminal_dispatch_console' in source
    assert "executable='terminal_dispatch_console'" not in source


def test_dispatch_marker_topic_and_task_point_config_are_wired():
    source = _source()

    assert "task_points_file" in source
    assert "task_points.yaml" in source
    assert "'marker_topic': '/robot_dispatch/markers'" in source

