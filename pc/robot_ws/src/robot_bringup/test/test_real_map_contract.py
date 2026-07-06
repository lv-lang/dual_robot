from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def _load_yaml(relative_path):
    with (ROOT / relative_path).open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_real_map_and_task_points_share_version():
    map_config = _load_yaml('maps/real_competition_map.yaml')
    task_points = _load_yaml('config/real_task_points.yaml')
    launch_source = (ROOT / 'launch' / 'real_robot_control_plane.launch.py').read_text(
        encoding='utf-8')

    assert map_config['image'] == 'real_competition_map.pgm'
    assert map_config['map_version']
    assert task_points['map_version'] == map_config['map_version']
    assert (
        f"DeclareLaunchArgument('map_version', "
        f"default_value='{map_config['map_version']}')"
    ) in launch_source
    assert "mecanum_ackermann_nav.rviz" in launch_source
    assert "DeclareLaunchArgument('start_rviz_goal_relay', default_value='false')" in launch_source
    assert "executable='rviz_goal_to_nav2_action'" in launch_source
    assert "DeclareLaunchArgument('seed_initial_poses', default_value='false')" in launch_source
    assert "executable='seed_real_initial_poses'" in launch_source


def test_real_map_image_is_not_placeholder():
    map_image = ROOT / 'maps' / 'real_competition_map.pgm'

    assert map_image.exists()
    assert map_image.stat().st_size > 1024
