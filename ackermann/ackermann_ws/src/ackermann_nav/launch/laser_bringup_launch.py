from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    ROBOT_TYPE = os.getenv('ROBOT_TYPE')
    rplidar_type = RPLIDAR_TYPE = os.getenv('RPLIDAR_TYPE')

    if not os.environ.get("LASER_BRINGUP_PRINTED"):
        os.environ["LASER_BRINGUP_PRINTED"] = "1"
        print("\n-------- robot_type = {}, rplidar_type = {} --------\n".format(ROBOT_TYPE, RPLIDAR_TYPE))

    robot_type_arg = DeclareLaunchArgument(
        name='robot_type', 
        default_value=os.getenv('ROBOT_TYPE', 'x3'), 
        choices=['x3', 'r2'],
        description='The type of robot'
    )
    rplidar_type_arg = DeclareLaunchArgument(
        name='rplidar_type', 
        default_value=os.getenv('RPLIDAR_TYPE', 'a1'), 
        choices=['a1', 's2', '4ROS'],
        description='The type of RPLIDAR'
    )
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='ackermann/odom')
    base_footprint_frame_arg = DeclareLaunchArgument(
        'base_footprint_frame', default_value='ackermann/base_footprint')
    base_frame_arg = DeclareLaunchArgument('base_frame', default_value='ackermann/base_link')
    lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value='ackermann/laser')
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/rplidar')
    serial_baudrate_arg = DeclareLaunchArgument('serial_baudrate', default_value='115200')
    frame_prefix_arg = DeclareLaunchArgument('frame_prefix', default_value='ackermann/')
    pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='true')
    use_ekf_arg = DeclareLaunchArgument('use_ekf', default_value='false')


    bringup_r2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ackermann_bringup'), 'launch'),
            '/ackermann_bringup_R2_launch.py']),
        condition=LaunchConfigurationEquals('robot_type', 'r2'),
        launch_arguments={
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_footprint_frame': LaunchConfiguration('base_footprint_frame'),
            'frame_prefix': LaunchConfiguration('frame_prefix'),
            'pub_odom_tf': LaunchConfiguration('pub_odom_tf'),
            'use_ekf': LaunchConfiguration('use_ekf'),
        }.items(),
    )

    lidar_a1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('sllidar_ros2'), 'launch'),
            '/sllidar_launch.py']),
        condition=LaunchConfigurationEquals('rplidar_type', 'a1'),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': LaunchConfiguration('lidar_frame'),
        }.items(),
    )


    tf_base_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.0435', '5.258E-05', '0.11', '3.14', '0', '0',
            LaunchConfiguration('base_frame'),
            LaunchConfiguration('lidar_frame'),
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    return LaunchDescription([
        robot_type_arg,
        rplidar_type_arg,
        odom_frame_arg,
        base_footprint_frame_arg,
        base_frame_arg,
        lidar_frame_arg,
        serial_port_arg,
        serial_baudrate_arg,
        frame_prefix_arg,
        pub_odom_tf_arg,
        use_ekf_arg,
        bringup_r2_launch,
        lidar_a1_launch,
        tf_base_link_to_laser

    ])
