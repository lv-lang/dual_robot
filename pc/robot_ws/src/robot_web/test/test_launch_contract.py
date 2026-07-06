from pathlib import Path


LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "robot_web.launch.py"


def test_robot_web_launch_starts_gateway_as_process_without_ros_args():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert "ExecuteProcess" in source
    assert "OpaqueFunction" in source
    assert "launch_ros.actions import Node" not in source
    assert "--ros-args" not in source


def test_robot_web_launch_passes_current_map_to_gateway():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert "map_yaml" in source
    assert "--map-yaml" in source
    assert "real_competition_map.yaml" in source


def test_robot_web_launch_passes_camera_config_to_gateway():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert "cameras_file" in source
    assert "--cameras-file" in source
    assert "cameras.yaml" in source


def test_robot_web_launch_does_not_expose_demo_system_control():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert "demo_system_control" not in source
    assert "--demo-system-control" not in source
