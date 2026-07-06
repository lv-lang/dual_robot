import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("robot_vision_bpu")
    default_params_file = os.path.join(
        package_share,
        "config",
        "robot1_bpu_detector.yaml",
    )

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Path to the robot1 BPU detector parameter file.",
            ),
            Node(
                package="robot_vision_bpu",
                executable="bpu_detector",
                name="bpu_vision_detector",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
