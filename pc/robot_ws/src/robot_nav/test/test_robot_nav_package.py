import re
from pathlib import Path

import pytest
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def _read(relative_path):
    return (PACKAGE_DIR / relative_path).read_text(encoding='utf-8')


def _robot_gazebo_file(relative_path):
    path = PACKAGE_DIR.parents[0] / 'robot_gazebo' / relative_path
    if not path.exists():
        pytest.skip('robot_gazebo is not present in this deployment workspace')
    return path.read_text(encoding='utf-8')


def test_setup_installs_all_launch_python_files():
    setup_py = _read('setup.py')
    assert "glob(os.path.join('launch', '*.py'))" in setup_py
    assert "_recursive_data_files('rviz_rendering_overlay')" in setup_py


def test_expected_nav2_entrypoints_exist():
    assert (PACKAGE_DIR / 'launch' / 'robot1_nav2_sim.launch.py').exists()
    assert (PACKAGE_DIR / 'launch' / 'robot2_nav2_sim.launch.py').exists()
    assert (PACKAGE_DIR / 'launch' / 'robot1_nav2_real.launch.py').exists()
    assert (PACKAGE_DIR / 'launch' / 'robot1_nav2_rdk_legacy.launch.py').exists()
    assert (PACKAGE_DIR / 'launch' / 'laser_bringup_launch.py').exists()


def test_mecanum_real_lifecycle_allows_slow_real_robot_startup():
    real_launch = _read('launch/mecanum_nav2_real.launch.py')

    assert "'bond_timeout': 20.0" in real_launch


def test_real_launch_does_not_include_hardware_bringup():
    real_launch = _read('launch/robot1_nav2_real.launch.py')
    legacy_launch = _read('launch/robot1_nav2_rdk_legacy.launch.py')
    assert 'laser_bringup_launch.py' not in real_launch
    assert 'yahboomcar_bringup' not in real_launch
    assert 'rplidar_ros' not in real_launch
    assert 'sllidar_ros2' not in real_launch
    assert 'laser_bringup_launch.py' not in legacy_launch
    assert 'yahboomcar_bringup' not in legacy_launch
    assert 'rplidar_ros' not in legacy_launch
    assert 'sllidar_ros2' not in legacy_launch


def test_sim_launch_disables_gazebo_map_to_odom_tf_for_amcl():
    sim_launch = _read('launch/robot1_nav2_sim.launch.py')
    real_launch = _read('launch/robot1_nav2_real.launch.py')
    assert "'params_file': LaunchConfiguration('nav2_params_file')" in sim_launch
    assert "'params_file': LaunchConfiguration('nav2_params_file')" in real_launch
    assert "'params_file'," not in sim_launch
    assert 'DIRECT_NAV2_REQUIRED_PACKAGES' not in sim_launch
    assert 'starting direct ' not in sim_launch
    assert "package='nav2_controller'" not in sim_launch
    assert "package='nav2_lifecycle_manager'" not in sim_launch
    assert '_optional_nav2_bringup(nav2_launch_file)' in sim_launch
    assert "'publish_map': publish_gazebo_map" in sim_launch
    assert "'publish_map_to_odom_tf': publish_gazebo_map_to_odom_tf" in sim_launch
    assert "'map_topic': '/map'" in sim_launch
    assert "gazebo_map_fallback_default = 'false' if nav2_available else 'true'" in sim_launch
    assert 'GroupAction(actions=[gazebo], scoped=True)' in sim_launch
    assert "('/tf', 'tf')" in sim_launch
    assert "('/tf_static', 'tf_static')" in sim_launch
    assert "('/tf', 'tf')" in real_launch
    assert "('/tf_static', 'tf_static')" in real_launch
    assert "'QT_QPA_PLATFORM': 'xcb'" in sim_launch
    assert "'QT_QPA_PLATFORM': 'xcb'" in real_launch
    assert "'LIBGL_ALWAYS_SOFTWARE': '1'" in sim_launch
    assert "'LIBGL_ALWAYS_SOFTWARE': '1'" in real_launch
    assert "'AMENT_PREFIX_PATH'" in sim_launch
    assert "'AMENT_PREFIX_PATH'" in real_launch
    assert "'rviz_rendering_overlay'" in sim_launch
    assert "'rviz_rendering_overlay'" in real_launch
    assert 'TimerAction(period=5.0, actions=[rviz])' in sim_launch
    assert 'TimerAction(period=5.0, actions=[rviz])' in real_launch


def test_sim_launch_defaults_to_stable_empty_world():
    sim_launch = _read('launch/robot1_nav2_sim.launch.py')
    empty_index = sim_launch.index("'worlds', 'robot1_mecanum_empty.world'")
    pioneer_index = sim_launch.index("'worlds', 'pioneer_test_20x10.world'")
    assert empty_index < pioneer_index


def test_rviz_map_displays_use_transient_local_full_maps():
    rviz = _read('rviz/robot1_nav2.rviz')
    for topic in [
        '/map',
        '/robot1/global_costmap/costmap',
        '/robot1/local_costmap/costmap',
    ]:
        topic_index = rviz.index(f'Value: {topic}')
        topic_block = rviz[rviz.rfind('Topic:', 0, topic_index):topic_index]
        assert 'Durability Policy: Transient Local' in topic_block


def test_rviz_static_map_does_not_subscribe_to_missing_map_updates():
    rviz = _read('rviz/robot1_nav2.rviz')
    name_index = rviz.index('Name: Map')
    display_start = rviz.rfind('- Alpha:', 0, name_index)
    display_end = rviz.find('\n    - Alpha:', name_index)
    display_block = rviz[display_start:display_end]
    assert 'Value: /map' in display_block
    assert 'Draw Behind: true' in display_block
    assert 'Value: /map_updates' not in display_block


def test_rviz_rendering_overlay_avoids_mixed_sampler_shader_error():
    shader = _read(
        'rviz_rendering_overlay/share/rviz_rendering/ogre_media/materials/glsl120/'
        'indexed_8bit_image.frag'
    )
    vertex_shader = _read(
        'rviz_rendering_overlay/share/rviz_rendering/ogre_media/materials/glsl120/'
        'indexed_8bit_image.vert'
    )
    material = _read(
        'rviz_rendering_overlay/share/rviz_rendering/ogre_media/materials/scripts/'
        'indexed_8bit_image.material'
    )
    assert '#version 420 compatibility' in shader
    assert '#version 420 compatibility' in vertex_shader
    assert 'uniform sampler2D eight_bit_image;' in shader
    assert 'uniform sampler1D palette;' in shader
    assert 'layout(binding = 0)' in shader
    assert 'layout(binding = 1)' in shader
    assert 'texture( palette,' in shader
    assert 'texture test_20x20.png 1d' in material


def test_rviz_keeps_costmap_overlays_disabled_by_default():
    rviz = _read('rviz/robot1_nav2.rviz')
    for display_name in ['global_costmap', 'local_costmap']:
        name_index = rviz.index(f'Name: {display_name}')
        display_start = rviz.rfind('- Alpha:', 0, name_index)
        display_end = rviz.find('\n    - Alpha:', name_index)
        display_block = rviz[display_start:display_end]
        assert 'Enabled: false' in display_block
        assert 'Value: false' in display_block


def test_dual_robot_rviz_uses_shared_map_and_namespaced_topics():
    rviz = _read('rviz/dual_robot_nav2.rviz')
    loaded = yaml.safe_load(rviz)

    assert loaded['Visualization Manager']['Global Options']['Fixed Frame'] == 'map'
    assert 'Value: /map' in rviz
    assert 'Value: /robot1/robot_description' in rviz
    assert 'Value: /robot2/robot_description' in rviz

    for robot in ['robot1', 'robot2']:
        for suffix in [
            'scan',
            'odom',
            'plan',
            'local_plan',
            'goal_pose',
            'global_costmap/costmap',
            'local_costmap/costmap',
        ]:
            assert f'Value: /{robot}/{suffix}' in rviz
        assert f'Value: /{robot}/map' not in rviz

    assert 'Value: /cmd_vel' not in rviz
    assert 'Value: /odom' not in rviz
    assert 'Value: /scan' not in rviz

    tools = loaded['Visualization Manager']['Tools']
    goal_topics = [
        tool['Topic']['Value']
        for tool in tools
        if tool.get('Class') == 'rviz_default_plugins/SetGoal'
    ]
    assert goal_topics == ['/robot1/goal_pose', '/robot2/goal_pose']


def test_robot_gazebo_has_optional_map_to_odom_tf_switch():
    gazebo_launch = _robot_gazebo_file('launch/robot1_mecanum_world.launch.py')
    assert (
        "publish_map_to_odom_tf = LaunchConfiguration('publish_map_to_odom_tf')"
        in gazebo_launch
    )
    assert "DeclareLaunchArgument(\n            'publish_map_to_odom_tf'" in gazebo_launch
    assert 'condition=IfCondition(publish_map_to_odom_tf)' in gazebo_launch
    assert "map_topic = LaunchConfiguration('map_topic')" in gazebo_launch
    assert "'topic': map_topic" in gazebo_launch


def test_robot_gazebo_publishes_tf_on_robot1_namespace_for_nav2():
    gazebo_launch = _robot_gazebo_file('launch/robot1_mecanum_world.launch.py')
    gazebo_xacro = _robot_gazebo_file('urdf/robot1_mecanum_gazebo.urdf.xacro')

    assert "('/tf', 'tf')" in gazebo_launch
    assert "('/tf_static', 'tf_static')" in gazebo_launch
    assert '<remapping>/tf:=tf</remapping>' in gazebo_xacro
    assert '<remapping>/tf_static:=tf_static</remapping>' in gazebo_xacro


def test_robot_nav_maps_keep_only_real_common_map():
    map_files = sorted(path.name for path in (PACKAGE_DIR / 'maps').iterdir())
    assert map_files == ['lv_home.pgm', 'lv_home.yaml']


def test_nav2_params_use_robot1_topics_and_global_map_only():
    forbidden_global_topic = re.compile(r'(?<!/robot1)/(scan|odom|cmd_vel)\\b')
    for params_path in [
        PACKAGE_DIR / 'params' / 'robot1_nav2_sim.yaml',
        PACKAGE_DIR / 'params' / 'robot1_nav2_real.yaml',
    ]:
        params_text = params_path.read_text(encoding='utf-8')
        assert forbidden_global_topic.search(params_text) is None
        assert '/robot1/scan' in params_text
        assert '/robot1/odom' in params_text
        assert '/map' in params_text


def test_nav2_params_are_loadable_yaml():
    for params_path in [
        PACKAGE_DIR / 'params' / 'robot1_nav2_sim.yaml',
        PACKAGE_DIR / 'params' / 'robot1_nav2_real.yaml',
    ]:
        with params_path.open(encoding='utf-8') as stream:
            loaded = yaml.safe_load(stream)
        assert loaded['amcl']['ros__parameters']['scan_topic'] == '/robot1/scan'
        assert loaded['amcl']['ros__parameters']['odom_frame_id'] == 'robot1/odom'
        assert loaded['controller_server']['ros__parameters']['FollowPath']['plugin'] == (
            'dwb_core::DWBLocalPlanner'
        )


def test_robot2_nav2_sim_params_use_rpp_and_shared_map():
    params_path = PACKAGE_DIR / 'params' / 'robot2_nav2_sim.yaml'
    params_text = params_path.read_text(encoding='utf-8')
    with params_path.open(encoding='utf-8') as stream:
        loaded = yaml.safe_load(stream)

    controller = loaded['controller_server']['ros__parameters']['FollowPath']
    assert controller['plugin'] == (
        'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    )
    assert loaded['bt_navigator']['ros__parameters']['global_frame'] == 'map'
    assert loaded['bt_navigator']['ros__parameters']['robot_base_frame'] == (
        'robot2/base_footprint'
    )
    assert loaded['bt_navigator']['ros__parameters']['odom_topic'] == '/robot2/odom'
    assert loaded['local_costmap']['local_costmap']['ros__parameters']['global_frame'] == (
        'robot2/odom'
    )
    assert loaded['global_costmap']['global_costmap']['ros__parameters']['global_frame'] == (
        'map'
    )
    assert loaded['global_costmap']['global_costmap']['ros__parameters']['static_layer'][
        'map_topic'
    ] == '/map'
    assert '/robot2/scan' in params_text
    assert '/robot2/cmd_vel' not in params_text
    assert '/robot2/map' not in params_text
    assert 'amcl:' not in params_text
    assert 'min_y_velocity_threshold: 0.5' in params_text
    assert 'max_velocity: [0.35, 0.0, 0.80]' in params_text


def test_sim_amcl_has_initial_pose_matching_default_spawn():
    sim_params_path = PACKAGE_DIR / 'params' / 'robot1_nav2_sim.yaml'
    with sim_params_path.open(encoding='utf-8') as stream:
        loaded = yaml.safe_load(stream)
    amcl_params = loaded['amcl']['ros__parameters']
    assert amcl_params['set_initial_pose'] is True
    assert amcl_params['initial_pose']['x'] == -8.5
    assert amcl_params['initial_pose']['y'] == -3.8
    assert amcl_params['initial_pose']['yaw'] == 0.0


def test_rdk_legacy_launch_keeps_lv_ws_hardware_interface():
    legacy_launch = _read('launch/robot1_nav2_rdk_legacy.launch.py')
    legacy_params_text = _read('params/robot1_nav2_rdk_legacy.yaml')
    legacy_rviz = _read('rviz/robot1_nav2_rdk_legacy.rviz')

    assert "default_value=''" in legacy_launch
    assert "arguments[name] = 'False'" in legacy_launch
    assert '/scan' in legacy_params_text
    assert '/odom' in legacy_params_text
    assert 'base_frame_id: base_footprint' in legacy_params_text
    assert 'odom_frame_id: odom' in legacy_params_text
    assert '/robot1/scan' not in legacy_params_text
    assert '/robot1/odom' not in legacy_params_text
    assert 'robot1/base_footprint' not in legacy_params_text
    assert 'Value: /scan' in legacy_rviz
    assert 'Value: /odom' in legacy_rviz
    assert 'Value: /robot1/scan' not in legacy_rviz
    assert 'Value: /robot1/odom' not in legacy_rviz


def test_rdk_legacy_nav2_params_are_loadable_yaml():
    params_path = PACKAGE_DIR / 'params' / 'robot1_nav2_rdk_legacy.yaml'
    with params_path.open(encoding='utf-8') as stream:
        loaded = yaml.safe_load(stream)

    assert loaded['amcl']['ros__parameters']['scan_topic'] == '/scan'
    assert loaded['amcl']['ros__parameters']['odom_frame_id'] == 'odom'
    assert loaded['bt_navigator']['ros__parameters']['odom_topic'] == '/odom'
    assert loaded['local_costmap']['local_costmap']['ros__parameters']['global_frame'] == (
        'odom'
    )
    assert loaded['controller_server']['ros__parameters']['FollowPath']['plugin'] == (
        'dwb_core::DWBLocalPlanner'
    )


def test_rdk_legacy_dwb_tuning_uses_mecanum_lateral_motion():
    params_path = PACKAGE_DIR / 'params' / 'robot1_nav2_rdk_legacy.yaml'
    with params_path.open(encoding='utf-8') as stream:
        loaded = yaml.safe_load(stream)

    controller = loaded['controller_server']['ros__parameters']
    follow_path = controller['FollowPath']
    goal_checker = controller['general_goal_checker']
    local_costmap = loaded['local_costmap']['local_costmap']['ros__parameters']
    smoother = loaded['velocity_smoother']['ros__parameters']

    assert controller['controller_frequency'] >= 12.0
    assert goal_checker['xy_goal_tolerance'] <= 0.20
    assert goal_checker['yaw_goal_tolerance'] <= 0.20
    assert follow_path['min_vel_x'] == 0.0
    assert follow_path['max_vel_y'] >= 0.12
    assert follow_path['acc_lim_y'] >= 0.70
    assert follow_path['vy_samples'] >= 11
    assert follow_path['max_vel_theta'] <= 0.50
    assert follow_path['BaseObstacle.scale'] >= 2.5
    assert follow_path['PathDist.scale'] >= 18.0
    assert follow_path['PathAlign.scale'] <= 24.0
    assert follow_path['RotateToGoal.scale'] >= 4.0
    assert follow_path['RotateToGoal.slowing_factor'] >= 3.0
    assert local_costmap['resolution'] <= 0.03
    assert 'footprint' in local_costmap
    assert 'robot_radius' not in local_costmap
    assert local_costmap['footprint_padding'] >= 0.02
    assert local_costmap['obstacle_layer']['scan']['obstacle_min_range'] <= 0.05
    assert local_costmap['inflation_layer']['inflation_radius'] <= 0.24
    assert local_costmap['inflation_layer']['inflation_radius'] >= 0.20
    assert smoother['max_velocity'][1] == follow_path['max_vel_y']
    assert smoother['max_accel'][1] == follow_path['acc_lim_y']
    assert smoother['max_velocity'][2] == follow_path['max_vel_theta']
