"""Start robot2 Nav2 navigation for PC-side Gazebo validation."""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


ROBOT_NAMESPACE = 'robot2'


def _share_path(package_name, *relative_parts):
    try:
        return os.path.join(get_package_share_directory(package_name), *relative_parts)
    except PackageNotFoundError:
        return None


def _optional_nav2_navigation(nav2_launch_file):
    if nav2_launch_file is not None and os.path.exists(nav2_launch_file):
        return GroupAction(
            actions=[
                PushRosNamespace(LaunchConfiguration('namespace')),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch_file),
                    launch_arguments={
                        'namespace': LaunchConfiguration('namespace'),
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'params_file': LaunchConfiguration('nav2_params_file'),
                        'autostart': LaunchConfiguration('autostart'),
                        'use_composition': LaunchConfiguration('use_composition'),
                        'use_respawn': LaunchConfiguration('use_respawn'),
                        'log_level': LaunchConfiguration('log_level'),
                    }.items(),
                ),
            ],
        )

    return LogInfo(
        msg=(
            'robot_nav: nav2_bringup navigation_launch.py is not available. '
            'Source the workspace or install ros-humble-nav2-bringup.'
        )
    )


def generate_launch_description():
    default_params = _share_path('robot_nav', 'params', 'robot2_nav2_sim.yaml') or ''
    nav2_launch_file = _share_path('nav2_bringup', 'launch', 'navigation_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=ROBOT_NAMESPACE,
            description='Nav2 namespace. Robot2 Nav2 runs under /robot2.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_params,
            description='Robot2 Nav2 RPP parameter file for Gazebo validation.',
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
        LogInfo(
            msg=(
                'robot2 Nav2 sim: expects /map, /robot2/scan, /robot2/odom, '
                'and publishes /robot2/cmd_vel through velocity_smoother.'
            )
        ),
        _optional_nav2_navigation(nav2_launch_file),
    ])
