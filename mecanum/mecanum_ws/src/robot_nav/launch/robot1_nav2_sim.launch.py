"""Start robot1 Gazebo and Nav2 for PC-side parameter tuning."""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ROBOT_NAMESPACE = 'robot1'


def _share_path(package_name, *relative_parts):
    try:
        return os.path.join(get_package_share_directory(package_name), *relative_parts)
    except PackageNotFoundError:
        return None


def _first_existing_path(*paths):
    for path in paths:
        if path and os.path.exists(path):
            return path
    return ''


def _optional_include(package_name, launch_file, launch_arguments, condition=None):
    launch_path = _share_path(package_name, 'launch', launch_file)
    if launch_path is not None and os.path.exists(launch_path):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments=launch_arguments.items(),
            condition=condition,
        )

    missing_target = (
        f'{package_name}/launch/{launch_file}'
        if launch_path is None
        else launch_path
    )
    return LogInfo(
        msg=f'robot_nav: optional launch not available: {missing_target}',
        condition=condition,
    )


def _rviz_environment():
    env = {
        'QT_QPA_PLATFORM': 'xcb',
        'LIBGL_ALWAYS_SOFTWARE': '1',
    }
    overlay_prefix = _share_path('robot_nav', 'rviz_rendering_overlay')
    if overlay_prefix and os.path.exists(overlay_prefix):
        current_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
        env['AMENT_PREFIX_PATH'] = (
            f'{overlay_prefix}:{current_prefix}' if current_prefix else overlay_prefix
        )
    return env


def _nav2_arguments(nav2_launch_file):
    arguments = {
        'namespace': LaunchConfiguration('namespace'),
        'map': LaunchConfiguration('map'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'params_file': LaunchConfiguration('nav2_params_file'),
        'autostart': LaunchConfiguration('autostart'),
        'use_composition': LaunchConfiguration('use_composition'),
        'use_respawn': LaunchConfiguration('use_respawn'),
        'log_level': LaunchConfiguration('log_level'),
        'slam': 'False',
    }

    try:
        launch_text = open(nav2_launch_file, encoding='utf-8').read()
    except OSError:
        return arguments

    optional_false_args = [
        'use_namespace',
        'use_intra_process_comms',
        'use_keepout_zones',
        'use_speed_zones',
    ]
    for name in optional_false_args:
        if name in launch_text:
            arguments[name] = 'True' if name == 'use_namespace' else 'False'

    if 'use_localization' in launch_text:
        arguments['use_localization'] = 'True'

    return arguments


def _optional_nav2_bringup(nav2_launch_file):
    if nav2_launch_file is not None and os.path.exists(nav2_launch_file):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments=_nav2_arguments(nav2_launch_file).items(),
        )

    return LogInfo(
        msg=(
            'robot_nav: nav2_bringup is not available in the current ROS '
            'environment. Source the workspace or install ros-humble-nav2-bringup.'
        )
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_gazebo = LaunchConfiguration('launch_gazebo')
    launch_rviz = LaunchConfiguration('launch_rviz')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    map_yaml = LaunchConfiguration('map')
    publish_gazebo_map = LaunchConfiguration('publish_gazebo_map')
    publish_gazebo_map_to_odom_tf = LaunchConfiguration('publish_gazebo_map_to_odom_tf')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = LaunchConfiguration('z')
    spawn_yaw = LaunchConfiguration('yaw')
    rviz_config = LaunchConfiguration('rviz_config')

    default_params = _share_path('robot_nav', 'params', 'robot1_nav2_sim.yaml') or ''
    default_rviz = _share_path('robot_nav', 'rviz', 'robot1_nav2.rviz') or ''
    default_map = _first_existing_path(
        _share_path('robot_gazebo', 'maps', 'gazebo_odom_map.yaml'),
    )
    default_world = _first_existing_path(
        # _share_path('robot_gazebo', 'worlds', 'robot1_mecanum_empty.world'),
        _share_path('robot_gazebo', 'worlds', 'pioneer_test_20x10.world'),
    )
    nav2_launch_file = _share_path('nav2_bringup', 'launch', 'bringup_launch.py')
    nav2_available = nav2_launch_file is not None and os.path.exists(nav2_launch_file)
    gazebo_map_fallback_default = 'false' if nav2_available else 'true'

    gazebo = _optional_include(
        'robot_gazebo',
        'robot1_mecanum_world.launch.py',
        {
            'use_sim_time': use_sim_time,
            'world': world,
            'gui': gui,
            'verbose': verbose,
            'publish_map': publish_gazebo_map,
            'publish_map_to_odom_tf': publish_gazebo_map_to_odom_tf,
            'map_topic': '/map',
            'map_yaml': map_yaml,
            'x': spawn_x,
            'y': spawn_y,
            'z': spawn_z,
            'yaw': spawn_yaw,
        },
        IfCondition(launch_gazebo),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=ROBOT_NAMESPACE,
        output='screen',
        arguments=['-d', rviz_config],
        additional_env=_rviz_environment(),
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=ROBOT_NAMESPACE,
            description='Nav2 namespace. Robot1 Nav2 runs under /robot1.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time.',
        ),
        DeclareLaunchArgument(
            'launch_gazebo',
            default_value='true',
            description='Start robot_gazebo with robot1 hardware simulation.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Start RViz with the robot1 Nav2 view.',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Map YAML used by Nav2 map_server. Sim defaults to the Gazebo odom map.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_params,
            description='Nav2 parameter file for Gazebo tuning.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Gazebo world path.',
        ),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument(
            'publish_gazebo_map',
            default_value=gazebo_map_fallback_default,
            description=(
                'Let robot_gazebo publish /map. Defaults to true only when '
                'nav2_bringup is missing.'
            ),
        ),
        DeclareLaunchArgument(
            'publish_gazebo_map_to_odom_tf',
            default_value=gazebo_map_fallback_default,
            description=(
                'Let robot_gazebo publish static map -> robot1/odom. Defaults '
                'to true only when nav2_bringup is missing.'
            ),
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('x', default_value='-8.5'),
        DeclareLaunchArgument('y', default_value='-3.8'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='RViz config path.',
        ),
        LogInfo(
            msg=(
                'robot_nav sim: Gazebo publishes /robot1/scan and /robot1/odom; '
                'Nav2 owns /map and map -> robot1/odom when nav2_bringup is available.'
            )
        ),
        GroupAction(actions=[gazebo], scoped=True),
        _optional_nav2_bringup(nav2_launch_file),
        TimerAction(period=5.0, actions=[rviz]),
    ])
