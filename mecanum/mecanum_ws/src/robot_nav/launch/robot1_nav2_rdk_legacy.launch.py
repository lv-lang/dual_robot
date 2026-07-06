"""Start Nav2 for the RDK X5 real robot without changing lv_ws hardware code.

This launch intentionally matches the original Yahboom hardware interface:
/scan, /odom, /cmd_vel, odom -> base_footprint.
"""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _share_path(package_name, *relative_parts):
    try:
        return os.path.join(get_package_share_directory(package_name), *relative_parts)
    except PackageNotFoundError:
        return None


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
            arguments[name] = 'False'

    if 'use_localization' in launch_text:
        arguments['use_localization'] = 'True'

    return arguments


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


def _optional_nav2_bringup():
    nav2_launch_file = _share_path('nav2_bringup', 'launch', 'bringup_launch.py')
    if nav2_launch_file is not None and os.path.exists(nav2_launch_file):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments=_nav2_arguments(nav2_launch_file).items(),
        )

    return LogInfo(
        msg=(
            'robot_nav: nav2_bringup is not installed. Install/build Nav2 before '
            'running robot1_nav2_rdk_legacy.launch.py.'
        )
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    default_params = _share_path('robot_nav', 'params', 'robot1_nav2_rdk_legacy.yaml') or ''
    default_map = _share_path('robot_nav', 'maps', 'lv_home.yaml') or ''
    default_rviz = _share_path('robot_nav', 'rviz', 'robot1_nav2_rdk_legacy.rviz') or ''

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        additional_env=_rviz_environment(),
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Legacy real robot mode uses no ROS namespace.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Real robot uses wall time.',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Real robot map YAML. Defaults to lv_home.yaml.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_params,
            description='Nav2 parameters for the RDK lv_ws legacy hardware interface.',
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Start RViz. Usually false on the RDK X5.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='RViz config path.',
        ),
        LogInfo(
            msg=(
                'robot_nav RDK legacy: start ~/lv_ws hardware separately. Nav2 consumes '
                '/scan and /odom and publishes /cmd_vel for the existing chassis driver.'
            )
        ),
        _optional_nav2_bringup(),
        TimerAction(period=5.0, actions=[rviz]),
    ])
