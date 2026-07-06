from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('yolo_web'))
    models_dir = package_share / 'models'
    config_dir = package_share / 'config'
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/video0'),
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8088'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='15'),
        DeclareLaunchArgument('jpeg_quality', default_value='70'),
        DeclareLaunchArgument('enable_detection', default_value='true'),
        DeclareLaunchArgument('model_path', default_value=str(models_dir / 'box_camera_best_bayese_640x640_nv12.bin')),
        DeclareLaunchArgument('label_file', default_value=str(models_dir / 'classes.txt')),
        DeclareLaunchArgument('fire_smoke_model_path', default_value=str(models_dir / 'fire_smoke_best_bayese_640x640_nv12.bin')),
        DeclareLaunchArgument('fire_smoke_label_file', default_value=str(models_dir / 'fire_smoke.list')),
        DeclareLaunchArgument('score_threshold', default_value='0.35'),
        DeclareLaunchArgument('fire_smoke_score_threshold', default_value='0.35'),
        DeclareLaunchArgument('nms_threshold', default_value='0.45'),
        DeclareLaunchArgument('enable_box_order_check', default_value='true'),
        DeclareLaunchArgument('box_order_config', default_value=str(config_dir / 'box_order.yaml')),
        DeclareLaunchArgument('robot_id', default_value=''),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('stop_hold_sec', default_value='1.0'),
        DeclareLaunchArgument('stop_epsilon', default_value='0.001'),
        DeclareLaunchArgument('enable_activation_gate', default_value='true'),
        DeclareLaunchArgument('activation_enter_radius', default_value='0.5'),
        DeclareLaunchArgument('activation_exit_radius', default_value='0.7'),
        DeclareLaunchArgument('mission_feedback_topic', default_value=''),
        DeclareLaunchArgument('amcl_pose_topic', default_value=''),
        Node(
            package='yolo_web',
            executable='usb_camera_web_server',
            name='yolo_web',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                'enable_detection': LaunchConfiguration('enable_detection'),
                'model_path': LaunchConfiguration('model_path'),
                'label_file': LaunchConfiguration('label_file'),
                'fire_smoke_model_path': LaunchConfiguration('fire_smoke_model_path'),
                'fire_smoke_label_file': LaunchConfiguration('fire_smoke_label_file'),
                'score_threshold': LaunchConfiguration('score_threshold'),
                'fire_smoke_score_threshold': LaunchConfiguration('fire_smoke_score_threshold'),
                'nms_threshold': LaunchConfiguration('nms_threshold'),
                'enable_box_order_check': LaunchConfiguration('enable_box_order_check'),
                'box_order_config': LaunchConfiguration('box_order_config'),
                'robot_id': LaunchConfiguration('robot_id'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'stop_hold_sec': LaunchConfiguration('stop_hold_sec'),
                'stop_epsilon': LaunchConfiguration('stop_epsilon'),
                'enable_activation_gate': LaunchConfiguration('enable_activation_gate'),
                'activation_enter_radius': LaunchConfiguration('activation_enter_radius'),
                'activation_exit_radius': LaunchConfiguration('activation_exit_radius'),
                'mission_feedback_topic': LaunchConfiguration('mission_feedback_topic'),
                'amcl_pose_topic': LaunchConfiguration('amcl_pose_topic'),
            }],
        ),
    ])
