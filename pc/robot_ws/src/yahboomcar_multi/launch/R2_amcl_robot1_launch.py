import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    RPLIDAR_TYPE = os.getenv('RPLIDAR_TYPE')
    param_file = os.path.join(get_package_share_directory('yahboomcar_multi'),'param','R2_amcl_robot1.yaml')

    amcl_node = Node(
        name="robot1_amcl",
        package='nav2_amcl',
        executable='amcl',
        parameters=[param_file],
        remappings=[
            ('/initialpose', '/robot1/initialpose'),
        ],
        output = "screen"
    )
    
    life_node = Node(
        name="robot1_amcl_lifecycle_manager",
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': False},{'autostart': True},{'node_names': ['robot1_amcl']}]
        )
    if RPLIDAR_TYPE == '4ROS':
        tf_base_link_to_laser = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0435', '5.258E-05', '0.11', '0', '0', '0', 'robot1/base_link', 'robot1/laser']
        )
    else:
        tf_base_link_to_laser = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0435', '5.258E-05', '0.11', '3.14', '0', '0', 'robot1/base_link', 'robot1/laser']
        )
    return LaunchDescription([
        amcl_node,
        life_node,
        tf_base_link_to_laser
    ])
