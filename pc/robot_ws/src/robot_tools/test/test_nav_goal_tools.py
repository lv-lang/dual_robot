from pathlib import Path

import yaml

from robot_tools.nav2_goal_utils import (
    forbidden_topics,
    normalize_frame_id,
    required_robot_topics,
    status_to_label,
)
from robot_tools.seed_real_initial_poses import (
    _default_config_path as _default_initial_pose_config_path,
    load_initial_poses,
)
from robot_tools.send_g1_nav_goals import _default_config_path


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def test_nav_goal_utils_preserve_dual_robot_topic_contract():
    assert normalize_frame_id('/map') == 'map'
    assert normalize_frame_id('') == 'map'
    assert required_robot_topics(['mecanum']) == {
        '/mecanum/cmd_vel',
        '/mecanum/odom',
        '/mecanum/scan',
    }
    assert {
        '/cmd_vel',
        '/odom',
        '/scan',
        '/mecanum/map',
        '/ackermann/map',
    }.issubset(forbidden_topics(['mecanum', 'ackermann']))


def test_status_labels_are_user_visible():
    assert status_to_label(4) == 'SUCCEEDED'
    assert status_to_label(5) == 'CANCELED'
    assert status_to_label(6) == 'FAILED'
    assert status_to_label(12345) == 'UNKNOWN_STATUS_12345'


def test_g1_source_config_path_exists_without_install_space(monkeypatch):
    from ament_index_python.packages import PackageNotFoundError

    monkeypatch.setattr(
        'robot_tools.send_g1_nav_goals.get_package_share_directory',
        lambda package_name: (_ for _ in ()).throw(PackageNotFoundError(package_name)),
    )

    assert _default_config_path() == (
        PACKAGE_DIR.parents[0] / 'robot_bringup' / 'config' / 'g1_points.yaml'
    )
    assert _default_config_path().exists()


def test_real_initial_pose_source_config_path_exists_without_install_space(monkeypatch):
    from ament_index_python.packages import PackageNotFoundError

    monkeypatch.setattr(
        'robot_tools.seed_real_initial_poses.get_package_share_directory',
        lambda package_name: (_ for _ in ()).throw(PackageNotFoundError(package_name)),
    )

    assert _default_initial_pose_config_path() == (
        PACKAGE_DIR.parents[0] / 'robot_bringup' / 'config' / 'real_task_points.yaml'
    )
    assert _default_initial_pose_config_path().exists()


def test_real_initial_poses_seed_from_waiting_areas():
    config_path = (
        PACKAGE_DIR.parents[0] / 'robot_bringup' / 'config' / 'real_task_points.yaml')
    poses = load_initial_poses(config_path)
    points = yaml.safe_load(config_path.read_text(encoding='utf-8'))['points']

    assert poses['mecanum'].point_name == 'W1'
    assert poses['mecanum'].frame_id == 'map'
    assert poses['mecanum'].x == points['W1']['x']
    assert poses['mecanum'].y == points['W1']['y']
    assert poses['ackermann'].point_name == 'W2'
    assert poses['ackermann'].frame_id == 'map'
    assert poses['ackermann'].x == points['W2']['x']
    assert poses['ackermann'].y == points['W2']['y']


def test_setup_exposes_real_initial_pose_seeder():
    setup_py = (PACKAGE_DIR / 'setup.py').read_text(encoding='utf-8')

    assert (
        'seed_real_initial_poses = '
        'robot_tools.seed_real_initial_poses:main'
    ) in setup_py


def test_real_robot_rviz_has_independent_initial_pose_tools():
    rviz = (PACKAGE_DIR / 'rviz' / 'real_robot_low_bandwidth.rviz').read_text(
        encoding='utf-8')

    assert 'Value: /mecanum/initialpose' in rviz
    assert 'Value: /ackermann/initialpose' in rviz


def test_real_robot_low_bandwidth_rviz_avoids_high_frequency_topics():
    rviz = (PACKAGE_DIR / 'rviz' / 'real_robot_low_bandwidth.rviz').read_text(
        encoding='utf-8')

    forbidden_fragments = [
        'rviz_default_plugins/LaserScan',
        'rviz_default_plugins/RobotModel',
        'rviz_default_plugins/TF\n      Enabled: true',
        '/mecanum/scan',
        '/ackermann/scan',
        '/mecanum/odom',
        '/ackermann/odom',
        'local_costmap',
        'global_costmap',
        'PointCloud2',
        'Image',
    ]
    for fragment in forbidden_fragments:
        assert fragment not in rviz

    assert 'Value: /mecanum/amcl_pose' in rviz
    assert 'Value: /ackermann/amcl_pose' in rviz


def test_real_robot_low_bandwidth_rviz_goal_tools_target_low_bandwidth_topics():
    rviz = (PACKAGE_DIR / 'rviz' / 'real_robot_low_bandwidth.rviz').read_text(
        encoding='utf-8')

    assert 'Value: /mecanum/goal_pose' in rviz
    assert 'Value: /ackermann/goal_pose' in rviz


def test_setup_exposes_rviz_goal_relay():
    setup_py = (PACKAGE_DIR / 'setup.py').read_text(encoding='utf-8')

    assert (
        'rviz_goal_to_nav2_action = '
        'robot_tools.rviz_goal_to_nav2_action:main'
    ) in setup_py


def test_task_monitor_rviz_enables_dispatch_point_markers():
    rviz = (PACKAGE_DIR / 'rviz' / 'dual_robot_task_monitor.rviz').read_text(
        encoding='utf-8')

    assert 'Value: /robot_dispatch/markers' in rviz
    for namespace in [
        'waiting_area_points',
        'pickup_points',
        'delivery_points',
        'inspection_points',
        'active_goal',
        'resource_locks',
        'abnormal',
        'recheck',
        'task_state',
    ]:
        assert f'{namespace}: true' in rviz
