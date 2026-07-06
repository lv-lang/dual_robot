"""PC-side real robot control plane.

This launch intentionally does not start robot_web, Gazebo, or either RDK's
single-robot bringup. It owns the shared map and dispatch/control-plane nodes.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _share_path(package_name, *relative_parts):
    try:
        return os.path.join(get_package_share_directory(package_name), *relative_parts)
    except PackageNotFoundError:
        return ''


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    task_points_file = LaunchConfiguration('task_points_file')
    map_version = LaunchConfiguration('map_version')
    map_bundle_hash = LaunchConfiguration('map_bundle_hash')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    start_rviz_goal_relay = LaunchConfiguration('start_rviz_goal_relay')
    seed_initial_poses = LaunchConfiguration('seed_initial_poses')
    initial_pose_config = LaunchConfiguration('initial_pose_config')
    # ADR-0033 比赛精简版: 默认 true(完整门禁); 联调链路活时用 false 直接 READY。
    enforce_real_system_gates = LaunchConfiguration('enforce_real_system_gates')

    default_map = _share_path(
        'robot_bringup', 'maps', 'real_competition_map.yaml')
    default_task_points = _share_path(
        'robot_bringup', 'config', 'real_task_points.yaml')
    default_rviz = _share_path(
        'robot_nav', 'rviz', 'mecanum_ackermann_nav.rviz')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
            'frame_id': 'map',
        }],
    )

    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    dispatch_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='robot_dispatch',
                executable='robot_dispatch_node',
                name='robot_dispatch',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'task_points_file': task_points_file,
                    'map_version': map_version,
                    'map_bundle_hash': map_bundle_hash,
                    'enforce_real_system_gates': ParameterValue(
                        enforce_real_system_gates, value_type=bool),
                    'robot_ids': ['mecanum', 'ackermann'],
                    'mecanum_execute_mission_action': '/mecanum/execute_mission',
                    'ackermann_execute_mission_action': '/ackermann/execute_mission',
                    'marker_topic': '/robot_dispatch/markers',
                }],
            ),
        ],
    )

    rviz = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(launch_rviz),
            ),
        ],
    )

    rviz_goal_relay = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='robot_tools',
                executable='rviz_goal_to_nav2_action',
                name='rviz_goal_to_nav2_action',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robots': ['mecanum', 'ackermann'],
                    'wait_for_server_sec': 0.1,
                }],
                condition=IfCondition(start_rviz_goal_relay),
            ),
        ],
    )

    initial_pose_seed = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robot_tools',
                executable='seed_real_initial_poses',
                name='seed_real_initial_poses',
                output='screen',
                arguments=[
                    '--config', initial_pose_config,
                    '--wait-subscribers-sec', '180',
                    '--repeat', '8',
                    '--period-sec', '0.5',
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(seed_initial_poses),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('task_points_file', default_value=default_task_points),
        DeclareLaunchArgument('map_version', default_value='lv_home_v1'),
        DeclareLaunchArgument('map_bundle_hash', default_value=''),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('start_rviz_goal_relay', default_value='false'),
        DeclareLaunchArgument('seed_initial_poses', default_value='false'),
        DeclareLaunchArgument('initial_pose_config', default_value=default_task_points),
        DeclareLaunchArgument('enforce_real_system_gates', default_value='false'),
        LogInfo(msg='Starting real robot PC control plane: local map + robot_dispatch + low-bandwidth RViz'),
        map_server,
        TimerAction(period=1.0, actions=[map_lifecycle]),
        dispatch_node,
        rviz,
        rviz_goal_relay,
        initial_pose_seed,
    ])
