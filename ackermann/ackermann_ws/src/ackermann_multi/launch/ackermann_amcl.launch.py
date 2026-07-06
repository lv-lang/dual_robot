import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    RPLIDAR_TYPE = os.getenv('RPLIDAR_TYPE')
    param_file = os.path.join(get_package_share_directory('ackermann_multi'),'param','ackermann_amcl.yaml')

    amcl_node = Node(
        name="ackermann_amcl",
        package='nav2_amcl',
        executable='amcl',
        parameters=[param_file],
        remappings=[
            ('/initialpose', '/ackermann/initialpose'),
            ('/amcl_pose', '/ackermann/amcl_pose'),
        ],
        output = "screen"
    )
    
    life_node = Node(
        name="ackermann_amcl_lifecycle_manager",
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': False},{'autostart': True},{'node_names': ['ackermann_amcl']}]
        )
    if RPLIDAR_TYPE == '4ROS':
        tf_base_link_to_laser = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0435', '5.258E-05', '0.11', '0', '0', '0', 'ackermann/base_link', 'ackermann/laser']
        )
    else:
        tf_base_link_to_laser = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0435', '5.258E-05', '0.11', '3.14', '0', '0', 'ackermann/base_link', 'ackermann/laser']
        )
    return LaunchDescription([
        amcl_node,
        life_node,
        tf_base_link_to_laser
    ])
