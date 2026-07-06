from launch import LaunchDescription
import os

from launch.actions import IncludeLaunchDescription, LogInfo
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _optional_lidar_launch(package_name, launch_file, condition, missing_msg):
    try:
        package_share = get_package_share_directory(package_name)
    except PackageNotFoundError:
        return LogInfo(msg=missing_msg, condition=condition)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(package_share, 'launch'), launch_file]
        ),
        condition=condition,
    )


def generate_launch_description():

    ROBOT_TYPE = os.getenv('ROBOT_TYPE')
    rplidar_type = RPLIDAR_TYPE = os.getenv('RPLIDAR_TYPE')

    if not os.environ.get("LASER_BRINGUP_PRINTED"):
        os.environ["LASER_BRINGUP_PRINTED"] = "1"
        print(
            "\n-------- robot_type = {}, rplidar_type = {} --------\n".format(
                ROBOT_TYPE,
                RPLIDAR_TYPE,
            )
        )

    robot_type_arg = DeclareLaunchArgument(
        name='robot_type', 
        default_value=os.getenv('ROBOT_TYPE', 'x3'), 
        choices=['x3', 'r2'],
        description='The type of robot'
    )
    rplidar_type_arg = DeclareLaunchArgument(
        name='rplidar_type', 
        default_value=os.getenv('RPLIDAR_TYPE', 'c1'), 
        choices=['a1', 'c1'],
        description='The type of RPLIDAR'
    )
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='mecanum/odom')
    base_footprint_frame_arg = DeclareLaunchArgument(
        'base_footprint_frame', default_value='mecanum/base_footprint')
    base_frame_arg = DeclareLaunchArgument('base_frame', default_value='mecanum/base_link')
    lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value='mecanum/laser')
    frame_prefix_arg = DeclareLaunchArgument('frame_prefix', default_value='mecanum/')
    pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='true')
    use_ekf_arg = DeclareLaunchArgument('use_ekf', default_value='false')

    bringup_x3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('mecanum_bringup'), 'launch'),
            '/mecanum_bringup_X3_launch.py']),
        condition=LaunchConfigurationEquals('robot_type', 'x3'),
        launch_arguments={
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_footprint_frame': LaunchConfiguration('base_footprint_frame'),
            'frame_prefix': LaunchConfiguration('frame_prefix'),
            'pub_odom_tf': LaunchConfiguration('pub_odom_tf'),
            'use_ekf': LaunchConfiguration('use_ekf'),
        }.items(),
    )
    bringup_r2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('mecanum_bringup'), 'launch'),
            '/mecanum_bringup_R2_launch.py']),
        condition=LaunchConfigurationEquals('robot_type', 'r2')
    )

    lidar_a1_launch = _optional_lidar_launch(
        'sllidar_ros2',
        '/sllidar_launch.py',
        LaunchConfigurationEquals('rplidar_type', 'a1'),
        'robot_nav: sllidar_ros2 is not installed; rplidar_type:=a1 is unavailable.',
    )
    try:
        rplidar_share = get_package_share_directory('rplidar_ros')
        lidar_c1_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(rplidar_share, 'launch'), '/rplidar_c1_launch.py']),
            condition=LaunchConfigurationEquals('rplidar_type', 'c1'),
            launch_arguments={'frame_id': LaunchConfiguration('lidar_frame')}.items(),
        )
    except PackageNotFoundError:
        lidar_c1_launch = LogInfo(
            msg='robot_nav: rplidar_ros is not installed; rplidar_type:=c1 is unavailable.',
            condition=LaunchConfigurationEquals('rplidar_type', 'c1'),
        )

    try:
        sllidar_share = get_package_share_directory('sllidar_ros2')
        lidar_a1_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(sllidar_share, 'launch'), '/sllidar_launch.py']),
            condition=LaunchConfigurationEquals('rplidar_type', 'a1'),
            launch_arguments={'frame_id': LaunchConfiguration('lidar_frame')}.items(),
        )
    except PackageNotFoundError:
        lidar_a1_launch = LogInfo(
            msg='robot_nav: sllidar_ros2 is not installed; rplidar_type:=a1 is unavailable.',
            condition=LaunchConfigurationEquals('rplidar_type', 'a1'),
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
        frame_prefix_arg,
        pub_odom_tf_arg,
        use_ekf_arg,
        bringup_x3_launch,
        bringup_r2_launch,
        lidar_a1_launch,
        lidar_c1_launch,
        tf_base_link_to_laser

    ])
