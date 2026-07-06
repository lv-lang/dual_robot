import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('ackermann_nav')
    nav2_bringup_dir = get_package_share_directory('ackermann_multi')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='ackermann')
    # ADR-0027/0033: 权威地图统一为 real_competition_map (ackermann_nav 内已安装, 与 PC 同一 .pgm)
    map_yaml_path = LaunchConfiguration(
        'maps', default=os.path.join(package_path, 'maps', 'real_competition_map.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(
        nav2_bringup_dir, 'param', 'ackermann_nav.yaml'))
    use_namespace = LaunchConfiguration('use_namespace', default='true')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'params_file': nav2_param_path,
            'use_namespace': use_namespace}.items(),
    )

    # ADR-0033 比赛精简集成合同: mission_executor 提供 ExecuteMission + heartbeat,
    # require_dispatch_lease=false (不下发租约). health_monitor 照跑供态势图 map_pose.
    mission_executor = Node(
        package='robot_mission',
        executable='mission_executor_node',
        namespace='ackermann',
        name='mission_executor',
        output='screen',
        parameters=[{
            'robot_name': 'ackermann',
            'robot_id': 'ackermann',
            'robot_namespace': '/ackermann',
            'map_version': 'real_competition_map_v1',
            'nav2_action_name': 'navigate_to_pose',
            'navigation_timeout_sec': 180.0,
            'enable_nav2_execution': True,
            'require_dispatch_lease': False,
        }],
    )

    health_monitor = Node(
        package='robot_mission',
        executable='robot_health_monitor_node',
        namespace='ackermann',
        name='robot_health_monitor',
        output='screen',
        parameters=[{
            'robot_id': 'ackermann',
            'robot_namespace': '/ackermann',
            'map_version': 'real_competition_map_v1',
            'map_frame': 'map',
            'odom_frame': 'ackermann/odom',
            'base_frame': 'ackermann/base_footprint',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),
        DeclareLaunchArgument('namespace', default_value=namespace),
        DeclareLaunchArgument('maps', default_value=map_yaml_path),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path),
        DeclareLaunchArgument('use_namespace', default_value=use_namespace),
        nav2,
        # 等 Nav2 起来再拉起任务层 (navigate_to_pose / lifecycle 服务可用)
        TimerAction(period=6.0, actions=[mission_executor, health_monitor]),
    ])
