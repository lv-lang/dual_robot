import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    description_share = get_package_share_directory('yahboomcar_description')
    gazebo_model_path = os.pathsep.join(
        path for path in [
            os.path.dirname(description_share),
            os.environ.get('GAZEBO_MODEL_PATH', ''),
        ] if path
    )

    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_map = LaunchConfiguration('publish_map')
    publish_map_to_odom_tf = LaunchConfiguration('publish_map_to_odom_tf')
    map_topic = LaunchConfiguration('map_topic')
    map_yaml = LaunchConfiguration('map_yaml')

    default_model = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'urdf',
        'robot1_mecanum_gazebo.urdf.xacro',
    ])
    default_world = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'worlds',
        'pioneer_test_20x10.world',
    ])
    default_map_yaml = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'maps',
        'gazebo_odom_map.yaml',
    ])

    robot_description = ParameterValue(
        Command(['xacro ', model]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [gazebo_ros_share, '/launch/gazebo.launch.py']
        ),
        launch_arguments={
            'world': world,
            'gui': gui,
            'verbose': LaunchConfiguration('verbose'),
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        output='screen',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='robot1',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'rate': 10,
        }],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', '/robot1/robot_description',
            '-entity', 'robot1',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    unpause_physics = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'service',
                    'call',
                    '/unpause_physics',
                    'std_srvs/srv/Empty',
                    '{}',
                ],
                output='screen',
            ),
        ],
    )

    static_map_publisher = Node(
        package='robot_gazebo',
        executable='static_map_publisher',
        namespace='robot1',
        output='screen',
        condition=IfCondition(publish_map),
        parameters=[{
            'use_sim_time': False,
            'topic': map_topic,
            'frame_id': 'map',
            'map_yaml': map_yaml,
            'width': 120,
            'height': 120,
            'resolution': 0.05,
            'border_occupied': True,
            'publish_period': 1.0,
        }],
    )

    scan_self_filter = Node(
        package='robot_gazebo',
        executable='scan_self_filter',
        namespace='robot1',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': 'scan_raw',
            'output_topic': 'scan',
            'self_filter_radius_m': 0.30,
        }],
    )

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        output='screen',
        condition=IfCondition(publish_map_to_odom_tf),
        namespace='robot1',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        arguments=[
            '--frame-id', 'map',
            '--child-frame-id', 'robot1/odom',
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        DeclareLaunchArgument('model', default_value=default_model),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_map', default_value='true'),
        DeclareLaunchArgument(
            'map_topic',
            default_value='map',
            description=(
                'Map topic for robot_gazebo static map publisher. Use /map '
                'when Nav2/RViz should consume a global map topic.'
            ),
        ),
        DeclareLaunchArgument(
            'publish_map_to_odom_tf',
            default_value='true',
            description=(
                'Publish static map -> robot1/odom TF. Disable when AMCL owns '
                'this transform.'
            ),
        ),
        DeclareLaunchArgument('map_yaml', default_value=default_map_yaml),
        DeclareLaunchArgument('x', default_value='-8.5'),
        DeclareLaunchArgument('y', default_value='-3.8'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gazebo,
        joint_state_publisher,
        robot_state_publisher,
        spawn_robot,
        unpause_physics,
        static_map_publisher,
        scan_self_filter,
        map_to_odom_tf,
    ])
