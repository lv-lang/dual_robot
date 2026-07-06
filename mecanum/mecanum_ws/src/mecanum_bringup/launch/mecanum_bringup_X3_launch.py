from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    if not os.environ.get("PRINTED"):
        os.environ["PRINTED"] = "1"
        print("---------------------robot_type = x3---------------------")
    urdf_tutorial_path = get_package_share_path('mecanum_description')
    default_model_path = urdf_tutorial_path / 'urdf/mecanum_X3.urdf'
    default_rviz_config_path = urdf_tutorial_path / 'rviz/mecanum.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='false',
                                            description='Whether to publish the tf from the original odom to the base_footprint')
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom')
    base_footprint_frame_arg = DeclareLaunchArgument('base_footprint_frame', default_value='base_footprint')
    frame_prefix_arg = DeclareLaunchArgument('frame_prefix', default_value='')
    use_ekf_arg = DeclareLaunchArgument('use_ekf', default_value='true')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': LaunchConfiguration('frame_prefix'),
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', LaunchConfiguration('rvizconfig')],
    # )

    driver_node = Node(
        package='mecanum_bringup',
        executable='Mcnamu_driver_X3',
    )

    base_node = Node(
        package='mecanum_base_node',
        executable='base_node_X3',
        # 当使用ekf融合时，该tf有ekf发布
        parameters=[{
            'pub_odom_tf': LaunchConfiguration('pub_odom_tf'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_footprint_frame': LaunchConfiguration('base_footprint_frame'),
            'linear_scale_x': 1.0,
            'linear_scale_y': 1.0,
            'angular_scale': 1.0,
        }],
        remappings=[
            ('odom_raw', 'odom'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    imu_filter_config = os.path.join(              
        get_package_share_directory('mecanum_bringup'),
        'param',
        'imu_filter_param.yaml'
    ) 

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[
            imu_filter_config,
            {
                'publish_tf': False,
                'use_mag': False,
                'fixed_frame': 'mecanum/base_link',
                'world_frame': 'enu',
                'orientation_stddev': 0.1,
            },
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_localization'), 'launch'),
            '/ekf_x1_x3_launch.py']),
        condition=IfCondition(LaunchConfiguration('use_ekf')),
    )

    yahboom_joy_node = Node(
        package='mecanum_ctrl',
        executable='yahboom_joy_X3',
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        pub_odom_tf_arg,
        odom_frame_arg,
        base_footprint_frame_arg,
        frame_prefix_arg,
        use_ekf_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        # rviz_node,
        driver_node,
        base_node,
        imu_filter_node,
        ekf_node,
        # yahboom_joy_node,
        # joy_node
    ])
