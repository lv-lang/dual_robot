import math
import re
from pathlib import Path

from geometry_msgs.msg import Twist
from robot_gazebo.sim_ackermann_base import (
    _ackermann_twist,
    _entity_state_twist as _ackermann_entity_state_twist,
)
from robot_gazebo.sim_mecanum_base import (
    _entity_state_twist as _mecanum_entity_state_twist,
)
from robot_gazebo.scan_self_filter import filter_self_ranges


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def test_filter_self_ranges_removes_finite_body_returns_only():
    filtered = filter_self_ranges(
        [0.12, 0.30, 0.31, math.inf, float('nan')],
        0.30,
    )

    assert math.isinf(filtered[0])
    assert math.isinf(filtered[1])
    assert filtered[2] == 0.31
    assert math.isinf(filtered[3])
    assert math.isnan(filtered[4])


def test_sim_ackermann_base_uses_only_forward_and_yaw_commands():
    msg = Twist()
    msg.linear.x = 0.4
    msg.linear.y = 0.3
    msg.linear.z = 0.2
    msg.angular.x = 0.1
    msg.angular.y = 0.2
    msg.angular.z = 0.5

    sanitized = _ackermann_twist(msg)

    assert sanitized.linear.x == 0.4
    assert sanitized.linear.y == 0.0
    assert sanitized.linear.z == 0.0
    assert sanitized.angular.x == 0.0
    assert sanitized.angular.y == 0.0
    assert sanitized.angular.z == 0.5


def test_sim_ackermann_base_pose_drives_gazebo_entity_by_default():
    cmd = Twist()
    cmd.linear.x = 0.4
    cmd.angular.z = 0.5

    entity_twist = _ackermann_entity_state_twist(cmd, send_entity_twist=False)

    assert entity_twist.linear.x == 0.0
    assert entity_twist.linear.y == 0.0
    assert entity_twist.linear.z == 0.0
    assert entity_twist.angular.x == 0.0
    assert entity_twist.angular.y == 0.0
    assert entity_twist.angular.z == 0.0


def test_sim_ackermann_base_can_optionally_forward_entity_twist():
    cmd = Twist()
    cmd.linear.x = 0.4
    cmd.angular.z = 0.5

    entity_twist = _ackermann_entity_state_twist(cmd, send_entity_twist=True)

    assert entity_twist is cmd


def test_sim_mecanum_base_pose_drives_gazebo_entity_by_default():
    cmd = Twist()
    cmd.linear.x = 0.4
    cmd.linear.y = 0.2
    cmd.angular.z = 0.5

    entity_twist = _mecanum_entity_state_twist(cmd, send_entity_twist=False)

    assert entity_twist.linear.x == 0.0
    assert entity_twist.linear.y == 0.0
    assert entity_twist.linear.z == 0.0
    assert entity_twist.angular.x == 0.0
    assert entity_twist.angular.y == 0.0
    assert entity_twist.angular.z == 0.0


def test_robot2_gazebo_wrapper_enables_lidar_but_not_camera_runtime():
    text = (
        PACKAGE_DIR
        / 'urdf'
        / 'robot2_ackermann_gazebo.urdf.xacro'
    ).read_text(encoding='utf-8')
    active_text = re.sub(r'<!--.*?-->', '', text, flags=re.DOTALL)

    assert 'yahboomcar_R2_robot2.urdf.xacro' in text
    assert '<kinematic>true</kinematic>' in active_text
    assert '<sensor name="robot2_laser" type="ray">' in active_text
    assert '<namespace>/robot2</namespace>' in active_text
    assert '<remapping>~/out:=scan_raw</remapping>' in active_text
    assert 'type="camera"' not in active_text
    assert 'type="depth"' not in active_text
    assert 'robot2_color_camera' in text
    assert 'robot2_depth_camera' in text


def test_robot1_lidar_only_wrapper_avoids_headless_camera_plugins():
    text = (
        PACKAGE_DIR
        / 'urdf'
        / 'robot1_mecanum_gazebo_lidar_only.urdf.xacro'
    ).read_text(encoding='utf-8')

    assert 'yahboomcar_X3_robot1.urdf' in text
    assert '<kinematic>true</kinematic>' not in text
    assert '<sensor name="robot1_laser" type="ray">' in text
    assert '<namespace>/robot1</namespace>' in text
    assert 'gazebo_ros_planar_move' not in text
    assert 'type="camera"' not in text
    assert 'type="depth"' not in text


def test_tf_namespace_relay_is_installed_for_dual_robot_rviz():
    setup_py = (PACKAGE_DIR / 'setup.py').read_text(encoding='utf-8')
    package_xml = (PACKAGE_DIR / 'package.xml').read_text(encoding='utf-8')
    relay = (
        PACKAGE_DIR
        / 'robot_gazebo'
        / 'tf_namespace_relay.py'
    ).read_text(encoding='utf-8')

    assert 'tf_namespace_relay = robot_gazebo.tf_namespace_relay:main' in setup_py
    assert '<exec_depend>tf2_msgs</exec_depend>' in package_xml
    assert "['robot1', 'robot2']" in relay
    assert "f'/{robot_name}/tf'" in relay
    assert "f'/{robot_name}/tf_static'" in relay
    assert "'/tf'" in relay
    assert "'/tf_static'" in relay
