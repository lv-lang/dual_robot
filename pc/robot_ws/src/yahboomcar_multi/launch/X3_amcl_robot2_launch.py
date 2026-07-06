import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	param_file = os.path.join(get_package_share_directory('yahboomcar_multi'),'param','X3_amcl_robot2.yaml')

	amcl_node = Node(
    	name="robot2_amcl",
        package='nav2_amcl',
        executable='amcl',
        parameters=[param_file],
        remappings=[
            ('/initialpose', '/robot2/initialpose'),
        ],
        output = "screen"
    )
    
	life_node = Node(
    	name="robot2_amcl_lifecycle_manager",
    	package='nav2_lifecycle_manager',
    	executable='lifecycle_manager',
    	output='screen',
    	parameters=[{'use_sim_time': False},{'autostart': True},{'node_names': ['robot2_amcl']}]
    	)
	tf_base_link_to_laser = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace="robot2" ,
            arguments = ['0.0435', '5.258E-05', '0.11', '3.14', '0', '0', 'robot2/base_link', 'robot2/laser']
    )
	return LaunchDescription([
    	amcl_node,
    	life_node,
        tf_base_link_to_laser
    ])
    
    
