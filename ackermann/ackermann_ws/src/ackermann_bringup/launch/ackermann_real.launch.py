"""Single-entry real-robot bringup for the ackermann RDK X5."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _share_path(package_name, *relative_parts):
    return os.path.join(get_package_share_directory(package_name), *relative_parts)


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    amcl_delay = LaunchConfiguration('amcl_delay')

    hardware_launch = _share_path('ackermann_multi', 'launch', 'ackermann_a1_bringup.launch.xml')
    amcl_launch = _share_path('ackermann_multi', 'launch', 'ackermann_amcl.launch.py')

    hardware = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(hardware_launch),
        launch_arguments={
            'robot_name': namespace,
        }.items(),
    )

    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(amcl_launch),
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='ackermann'),
        DeclareLaunchArgument('amcl_delay', default_value='3.0'),
        LogInfo(msg='Starting ackermann real bringup: base/lidar first, delayed AMCL only. Start Nav2 manually after initial pose.'),
        hardware,
        TimerAction(period=amcl_delay, actions=[amcl]),
    ])
