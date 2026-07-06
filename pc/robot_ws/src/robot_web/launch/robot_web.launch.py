"""Launch robot_web App gateway for same-origin PWA access."""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _share_path(package_name, *relative_parts):
    try:
        return os.path.join(get_package_share_directory(package_name), *relative_parts)
    except PackageNotFoundError:
        return ''


def _optional_cli_arg(context, launch_name, cli_name):
    value = LaunchConfiguration(launch_name).perform(context)
    return [cli_name, value] if value else []


def _launch_app_gateway(context, *args, **kwargs):
    executable = os.path.join(
        get_package_prefix('robot_web'),
        'lib',
        'robot_web',
        'app_gateway',
    )
    cmd = [
        executable,
        '--host', LaunchConfiguration('host').perform(context),
        '--port', LaunchConfiguration('port').perform(context),
    ]
    cmd.extend(_optional_cli_arg(context, 'db_path', '--db-path'))
    cmd.extend(_optional_cli_arg(context, 'frontend_dist', '--frontend-dist'))
    cmd.extend(_optional_cli_arg(context, 'task_points_file', '--task-points-file'))
    cmd.extend(_optional_cli_arg(context, 'builtin_templates_file', '--builtin-templates-file'))
    cmd.extend(_optional_cli_arg(context, 'map_yaml', '--map-yaml'))
    cmd.extend(_optional_cli_arg(context, 'cameras_file', '--cameras-file'))
    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    default_map = _share_path('robot_bringup', 'maps', 'real_competition_map.yaml')
    default_cameras = _share_path('robot_bringup', 'config', 'cameras.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8000'),
        DeclareLaunchArgument('db_path', default_value=''),
        DeclareLaunchArgument('frontend_dist', default_value=''),
        DeclareLaunchArgument('task_points_file', default_value=''),
        DeclareLaunchArgument('builtin_templates_file', default_value=''),
        DeclareLaunchArgument('map_yaml', default_value=default_map),
        DeclareLaunchArgument('cameras_file', default_value=default_cameras),
        OpaqueFunction(function=_launch_app_gateway),
    ])
