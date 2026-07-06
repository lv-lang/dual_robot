import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('robot_nav')
    default_map = os.path.join(package_path, 'maps', 'real_competition_map.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Map YAML to publish on /map for the real dual-robot demo',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'yaml_filename': LaunchConfiguration('map')},
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server']},
        ],
    )

    return LaunchDescription([
        map_arg,
        map_server,
        lifecycle_manager,
    ])
