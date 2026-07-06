"""Direct dual robot Gazebo/Nav2 bringup for G0/G1 PC validation."""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _share_path(package_name, *relative_parts):
    try:
        return os.path.join(get_package_share_directory(package_name), *relative_parts)
    except PackageNotFoundError:
        return None


def _first_existing_path(*paths):
    for path in paths:
        if path and os.path.exists(path):
            return path
    return ''


def _rviz_environment():
    env = {
        'QT_QPA_PLATFORM': 'xcb',
        'LIBGL_ALWAYS_SOFTWARE': '1',
    }
    overlay_prefix = _share_path('robot_nav', 'rviz_rendering_overlay')
    if overlay_prefix and os.path.exists(overlay_prefix):
        current_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
        env['AMENT_PREFIX_PATH'] = (
            f'{overlay_prefix}{os.pathsep}{current_prefix}'
            if current_prefix else overlay_prefix
        )
    return env


def _robot_description(model):
    return ParameterValue(Command(['xacro ', model]), value_type=str)


def _nav2_navigation(namespace, params_file, condition):
    launch_path = _share_path('nav2_bringup', 'launch', 'navigation_launch.py')
    if launch_path is not None and os.path.exists(launch_path):
        return GroupAction(
            actions=[
                PushRosNamespace(namespace),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_path),
                    launch_arguments={
                        'namespace': namespace,
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'params_file': params_file,
                        'autostart': LaunchConfiguration('autostart'),
                        'use_composition': LaunchConfiguration('use_composition'),
                        'use_respawn': LaunchConfiguration('use_respawn'),
                        'log_level': LaunchConfiguration('log_level'),
                    }.items(),
                ),
            ],
            condition=condition,
        )

    return LogInfo(
        msg=(
            'dual_robot_gazebo_nav2: nav2_bringup navigation_launch.py is not '
            'available. Source the workspace or install ros-humble-nav2-bringup.'
        ),
        condition=condition,
    )


def _static_map_to_odom(namespace, child_frame, condition):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        namespace=namespace,
        output='screen',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', child_frame,
        ],
        condition=condition,
    )


def _state_publishers(namespace, robot_description, use_sim_time):
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
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
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'rate': 10,
        }],
    )

    return [joint_state_publisher, robot_state_publisher]


def generate_launch_description():
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    description_share = get_package_share_directory('yahboomcar_description')
    gazebo_model_path = os.pathsep.join(
        path for path in [
            os.path.dirname(description_share),
            os.environ.get('GAZEBO_MODEL_PATH', ''),
        ] if path
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    map_yaml = LaunchConfiguration('map')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    robot1_model = LaunchConfiguration('robot1_model')
    robot2_model = LaunchConfiguration('robot2_model')
    robot1_x = LaunchConfiguration('robot1_x')
    robot1_y = LaunchConfiguration('robot1_y')
    robot1_z = LaunchConfiguration('robot1_z')
    robot1_yaw = LaunchConfiguration('robot1_yaw')
    robot2_x = LaunchConfiguration('robot2_x')
    robot2_y = LaunchConfiguration('robot2_y')
    robot2_z = LaunchConfiguration('robot2_z')
    robot2_yaw = LaunchConfiguration('robot2_yaw')
    enable_robot1 = LaunchConfiguration('enable_robot1')
    enable_robot2 = LaunchConfiguration('enable_robot2')
    robot1_condition = IfCondition(enable_robot1)
    robot2_condition = IfCondition(enable_robot2)

    default_world = _first_existing_path(
        _share_path('robot_gazebo', 'worlds', 'pioneer_test_20x10.world'),
    )
    default_map = _first_existing_path(
        _share_path('robot_gazebo', 'maps', 'gazebo_odom_map.yaml'),
    )
    default_robot1_params = _share_path(
        'robot_nav', 'params', 'robot1_nav2_sim.yaml') or ''
    default_robot2_params = _share_path(
        'robot_nav', 'params', 'robot2_nav2_sim.yaml') or ''
    default_rviz = _share_path(
        'robot_nav', 'rviz', 'dual_robot_nav2.rviz') or ''

    default_robot1_model = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'urdf',
        'robot1_mecanum_gazebo_lidar_only.urdf.xacro',
    ])
    default_robot2_model = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'urdf',
        'robot2_ackermann_gazebo.urdf.xacro',
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [gazebo_ros_share, '/launch/gazebo.launch.py']
        ),
        launch_arguments={
            'world': world,
            'gui': gui,
            'verbose': verbose,
        }.items(),
    )

    static_map_publisher = Node(
        package='robot_gazebo',
        executable='static_map_publisher',
        name='g0_g1_static_map_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'topic': '/map',
            'frame_id': 'map',
            'map_yaml': map_yaml,
            'publish_period': 1.0,
        }],
    )

    tf_namespace_relay = Node(
        package='robot_gazebo',
        executable='tf_namespace_relay',
        name='g0_g1_tf_namespace_relay',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_names': ['robot1', 'robot2'],
        }],
    )

    rviz = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                additional_env=_rviz_environment(),
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(launch_rviz),
            ),
        ],
    )

    robot1_description = _robot_description(robot1_model)
    robot1_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', '/robot1/robot_description',
            '-entity', 'robot1',
            '-x', robot1_x,
            '-y', robot1_y,
            '-z', robot1_z,
            '-Y', robot1_yaw,
        ],
    )
    robot1_sim_base = Node(
        package='robot_gazebo',
        executable='sim_mecanum_base',
        namespace='robot1',
        output='screen',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'entity_name': 'robot1',
            'cmd_vel_topic': '/robot1/cmd_vel',
            'odom_topic': '/robot1/odom',
            'odom_frame': 'robot1/odom',
            'base_frame': 'robot1/base_footprint',
            'initial_x': robot1_x,
            'initial_y': robot1_y,
            'initial_z': robot1_z,
            'initial_yaw': robot1_yaw,
            'update_rate_hz': 60.0,
            'send_entity_twist': False,
        }],
    )
    robot1_group = GroupAction(
        condition=robot1_condition,
        actions=[
            *_state_publishers('robot1', robot1_description, use_sim_time),
            robot1_spawn,
            Node(
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
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=robot1_spawn,
                    on_exit=[robot1_sim_base],
                ),
            ),
            _static_map_to_odom('robot1', 'robot1/odom', robot1_condition),
            TimerAction(
                period=4.0,
                actions=[
                    _nav2_navigation(
                        'robot1',
                        LaunchConfiguration('robot1_nav2_params_file'),
                        robot1_condition,
                    )
                ],
            ),
        ],
    )

    robot2_description = _robot_description(robot2_model)
    robot2_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', '/robot2/robot_description',
            '-entity', 'robot2',
            '-x', robot2_x,
            '-y', robot2_y,
            '-z', robot2_z,
            '-Y', robot2_yaw,
        ],
    )
    robot2_sim_base = Node(
        package='robot_gazebo',
        executable='sim_ackermann_base',
        namespace='robot2',
        output='screen',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'entity_name': 'robot2',
            'cmd_vel_topic': '/robot2/cmd_vel',
            'odom_topic': '/robot2/odom',
            'odom_frame': 'robot2/odom',
            'base_frame': 'robot2/base_footprint',
            'initial_x': robot2_x,
            'initial_y': robot2_y,
            'initial_z': robot2_z,
            'initial_yaw': robot2_yaw,
            'update_rate_hz': 60.0,
            'send_entity_twist': False,
        }],
    )
    robot2_group = GroupAction(
        condition=robot2_condition,
        actions=[
            *_state_publishers('robot2', robot2_description, use_sim_time),
            robot2_spawn,
            Node(
                package='robot_gazebo',
                executable='scan_self_filter',
                namespace='robot2',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'input_topic': 'scan_raw',
                    'output_topic': 'scan',
                    'self_filter_radius_m': 0.35,
                }],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=robot2_spawn,
                    on_exit=[robot2_sim_base],
                ),
            ),
            _static_map_to_odom('robot2', 'robot2/odom', robot2_condition),
            TimerAction(
                period=8.0,
                actions=[
                    _nav2_navigation(
                        'robot2',
                        LaunchConfiguration('robot2_nav2_params_file'),
                        robot2_condition,
                    )
                ],
            ),
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

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        DeclareLaunchArgument(
            'enable_robot1',
            default_value='true',
            description='Start robot1 Gazebo model, scan filter, TF, and Nav2.',
        ),
        DeclareLaunchArgument(
            'enable_robot2',
            default_value='true',
            description='Start robot2 Gazebo model, scan filter, TF, and Nav2.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Gazebo world for G0/G1 PC validation.',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Shared /map YAML for both robots.',
        ),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Start RViz with the dual robot Nav2 view.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='RViz config path for visualizing dual robot Nav2.',
        ),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('robot1_model', default_value=default_robot1_model),
        DeclareLaunchArgument('robot2_model', default_value=default_robot2_model),
        DeclareLaunchArgument(
            'robot1_nav2_params_file',
            default_value=default_robot1_params,
        ),
        DeclareLaunchArgument(
            'robot2_nav2_params_file',
            default_value=default_robot2_params,
        ),
        DeclareLaunchArgument('robot1_x', default_value='-8.5'),
        DeclareLaunchArgument('robot1_y', default_value='-3.8'),
        DeclareLaunchArgument('robot1_z', default_value='-0.01'),
        DeclareLaunchArgument('robot1_yaw', default_value='0.0'),
        DeclareLaunchArgument('robot2_x', default_value='-8.5'),
        DeclareLaunchArgument('robot2_y', default_value='2.5'),
        DeclareLaunchArgument('robot2_z', default_value='0.0'),
        DeclareLaunchArgument('robot2_yaw', default_value='0.0'),
        LogInfo(
            msg=(
                'G0/G1 dual launch: shared /map, robot1 Nav2 DWB, robot2 Nav2 '
                'RPP, isolated /robot*/scan /robot*/odom /robot*/cmd_vel.'
            )
        ),
        gazebo,
        static_map_publisher,
        tf_namespace_relay,
        rviz,
        robot1_group,
        robot2_group,
        unpause_physics,
    ])
