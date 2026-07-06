"""PC-only Gazebo task-layer bringup for robot_dispatch validation."""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _share_path(package_name, *relative_parts):
    try:
        return os.path.join(get_package_share_directory(package_name), *relative_parts)
    except PackageNotFoundError:
        return ''


def _robot_mission_executor(robot_name):
    return TimerAction(
        period=12.0,
        actions=[
            GroupAction(
                actions=[
                    PushRosNamespace(robot_name),
                    Node(
                        package='robot_mission',
                        executable='mission_executor_node',
                        name='mission_executor',
                        output='screen',
                        parameters=[{
                            'use_sim_time': LaunchConfiguration('use_sim_time'),
                            'robot_name': robot_name,
                            'nav2_action_name': 'navigate_to_pose',
                            'navigation_timeout_sec': LaunchConfiguration(
                                'navigation_timeout_sec'),
                        }],
                    ),
                ],
                condition=IfCondition(
                    LaunchConfiguration(f'enable_{robot_name}')),
            ),
        ],
        condition=IfCondition(LaunchConfiguration('start_mission_executors')),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    task_points_file = LaunchConfiguration('task_points_file')

    dual_launch = _share_path(
        'robot_bringup', 'launch', 'dual_robot_gazebo_nav2.launch.py')
    default_task_points = _share_path(
        'robot_dispatch', 'config', 'task_points.yaml')
    default_rviz = _share_path(
        'robot_tools', 'rviz', 'dual_robot_task_monitor.rviz')

    dual_robot_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dual_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': LaunchConfiguration('gui'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'enable_robot1': LaunchConfiguration('enable_robot1'),
            'enable_robot2': LaunchConfiguration('enable_robot2'),
        }.items(),
    )

    dispatch_node = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='robot_dispatch',
                executable='robot_dispatch_node',
                name='robot_dispatch',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'task_points_file': task_points_file,
                    'robot_names': ['robot1', 'robot2'],
                    'robot1_execute_mission_action': '/robot1/execute_mission',
                    'robot2_execute_mission_action': '/robot2/execute_mission',
                    'marker_topic': '/robot_dispatch/markers',
                }],
            ),
        ],
        condition=IfCondition(LaunchConfiguration('start_dispatch')),
    )

    rviz_task_point_input_node = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='robot_dispatch',
                executable='rviz_task_point_input_node',
                name='rviz_task_point_input',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'add_task_point_service': '/robot_dispatch/add_task_point',
                    'pickup_topic': '/robot_dispatch/rviz/add_pickup_point',
                    'delivery_topic': '/robot_dispatch/rviz/add_delivery_point',
                    'inspection_topic': '/robot_dispatch/rviz/add_inspection_point',
                }],
            ),
        ],
        condition=IfCondition(LaunchConfiguration('start_rviz_task_point_input')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('enable_robot1', default_value='true'),
        DeclareLaunchArgument('enable_robot2', default_value='true'),
        DeclareLaunchArgument('start_dispatch', default_value='true'),
        DeclareLaunchArgument('start_rviz_task_point_input', default_value='true'),
        DeclareLaunchArgument('start_mission_executors', default_value='true'),
        DeclareLaunchArgument(
            'navigation_timeout_sec',
            default_value='180.0',
            description='Timeout used by each robot_mission Nav2 step.',
        ),
        DeclareLaunchArgument(
            'task_points_file',
            default_value=default_task_points,
            description='Task-layer business point configuration.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='RViz config for task dispatch monitoring.',
        ),
        LogInfo(
            msg=(
                'robot_dispatch Gazebo launch: dual Nav2 stack + dispatch + '
                'two mission executors. Start the console separately with: '
                'ros2 run robot_dispatch terminal_dispatch_console'
            )
        ),
        dual_robot_stack,
        _robot_mission_executor('robot1'),
        _robot_mission_executor('robot2'),
        dispatch_node,
        rviz_task_point_input_node,
    ])
