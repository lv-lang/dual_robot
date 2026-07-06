import os

from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_path = get_package_share_path('ackermann_description')
    default_model_path = description_path / 'urdf/ackermann_R2.urdf.xacro'
    gmapping_params = os.path.join(
        get_package_share_directory('slam_gmapping'),
        'params',
        'slam_gmapping.yaml',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    lidar_frame = LaunchConfiguration('lidar_frame')
    scan_topic = LaunchConfiguration('scan_topic')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame = LaunchConfiguration('map_frame')
    pub_odom_tf = LaunchConfiguration('pub_odom_tf')

    robot_description = ParameterValue(
        Command(['xacro "', LaunchConfiguration('model'), '"']),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=str(default_model_path)),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('lidar_frame', default_value='laser_link'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('pub_odom_tf', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            output='screen',
        ),

        Node(
            package='ackermann_bringup',
            executable='Ackman_driver_R2',
            name='driver_node',
            output='screen',
        ),

        Node(
            package='ackermann_base_node',
            executable='base_node_R2',
            name='base_node',
            parameters=[{
                'pub_odom_tf': ParameterValue(pub_odom_tf, value_type=bool),
                'odom_frame': odom_frame,
                'base_footprint_frame': base_frame,
                'linear_scale_x': 1.0,
                'linear_scale_y': 1.0,
            }],
            output='screen',
        ),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': lidar_frame,
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen',
        ),

        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            output='screen',
            parameters=[
                gmapping_params,
                {
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'base_frame': base_frame,
                    'odom_frame': odom_frame,
                    'map_frame': map_frame,
                    'particles': 80,
                    'linearUpdate': 0.18,
                    'angularUpdate': 0.15,
                    'temporalUpdate': 0.8,
                    'resampleThreshold': 0.5,
                    'maxUrange': 4.5,
                    'maxRange': 5.0,
                    'minimum_score': 140.0,
                    'sigma': 0.05,
                    'kernelSize': 1,
                    'lstep': 0.05,
                    'astep': 0.05,
                    'iterations': 5,
                    'lsigma': 0.075,
                    'ogain': 3.0,
                    'lskip': 0,
                    'delta': 0.05,
                    'xmin': -15.0,
                    'ymin': -15.0,
                    'xmax': 15.0,
                    'ymax': 15.0,
                    'llsamplerange': 0.01,
                    'llsamplestep': 0.01,
                    'lasamplerange': 0.005,
                    'lasamplestep': 0.005,
                },
            ],
            remappings=[('/scan', scan_topic)],
        ),
    ])
