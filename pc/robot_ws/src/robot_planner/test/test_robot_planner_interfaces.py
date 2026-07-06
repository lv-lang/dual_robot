from pathlib import Path


PKG = Path(__file__).resolve().parents[1]


def test_robot_planner_is_ament_cmake():
    package_xml = (PKG / "package.xml").read_text(encoding="utf-8")
    assert "<build_type>ament_cmake</build_type>" in package_xml
    assert "ament_python" not in package_xml


def test_mpc_dwb_config_uses_robot1_topics_only():
    config = (PKG / "config" / "robot1_mpc_dwb.yaml").read_text(encoding="utf-8")
    assert "/robot1/global_path" in config
    assert "/robot1/odom" in config
    assert "/robot1/scan" in config
    assert "/robot1/cmd_vel" in config
    assert "\n    cmd_vel_topic: /cmd_vel" not in config
    assert "\n    odom_topic: /odom" not in config
    assert "\n    scan_topic: /scan" not in config


def test_cmake_installs_new_executables():
    cmake = (PKG / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "add_executable(astar_global_planner" in cmake
    assert "add_executable(mpc_dwb_planner" in cmake
    assert "dwa_lite_planner" not in cmake
