import math
from typing import Optional

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster


class SimMecanumBase(Node):
    """Gazebo hardware adapter for robot1 mecanum cmd_vel and odometry."""

    def __init__(self) -> None:
        super().__init__('sim_mecanum_base')

        self.declare_parameter('entity_name', 'robot1')
        self.declare_parameter('cmd_vel_topic', '/robot1/cmd_vel')
        self.declare_parameter('odom_topic', '/robot1/odom')
        self.declare_parameter('odom_frame', 'robot1/odom')
        self.declare_parameter('base_frame', 'robot1/base_footprint')
        self.declare_parameter('set_entity_state_service', '/set_entity_state')
        self.declare_parameter('update_rate_hz', 30.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('move_entity', True)
        self.declare_parameter('send_entity_twist', False)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('initial_z', 0.0)

        self.entity_name = str(self.get_parameter('entity_name').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.cmd_timeout = Duration(
            seconds=float(self.get_parameter('cmd_timeout_sec').value))
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.move_entity = bool(self.get_parameter('move_entity').value)
        self.send_entity_twist = bool(
            self.get_parameter('send_entity_twist').value)
        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)
        self.z = float(self.get_parameter('initial_z').value)

        self.last_cmd = Twist()
        self.last_cmd_time: Optional[Time] = None
        self.last_update_time = self.get_clock().now()
        self.pending_state_request = None
        self.service_unavailable_warned = False
        self.state_failure_warned = False

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)

        service_name = str(self.get_parameter('set_entity_state_service').value)
        self.set_state_client = self.create_client(SetEntityState, service_name)

        update_rate = max(1.0, float(self.get_parameter('update_rate_hz').value))
        self.create_timer(1.0 / update_rate, self._on_timer)
        self.get_logger().info(
            f'sim_mecanum_base active: {self.cmd_vel_topic} -> {self.odom_topic} '
            f'at ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f})')

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        dt = max(0.0, (now - self.last_update_time).nanoseconds * 1e-9)
        self.last_update_time = now

        cmd = self.last_cmd
        if self.last_cmd_time is None or now - self.last_cmd_time > self.cmd_timeout:
            cmd = Twist()

        self._integrate(cmd, dt)
        self._publish_odom(now, cmd)
        self._send_entity_state(cmd)

    def _integrate(self, cmd: Twist, dt: float) -> None:
        if dt <= 0.0:
            return
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (cos_yaw * cmd.linear.x - sin_yaw * cmd.linear.y) * dt
        self.y += (sin_yaw * cmd.linear.x + cos_yaw * cmd.linear.y) * dt
        self.yaw = _normalize_angle(self.yaw + cmd.angular.z * dt)

    def _publish_odom(self, now: Time, cmd: Twist) -> None:
        q = _quaternion_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation = q
        odom.twist.twist = cmd
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = self.z
            transform.transform.rotation = q
            self.tf_broadcaster.sendTransform(transform)

    def _send_entity_state(self, cmd: Twist) -> None:
        if not self.move_entity:
            return
        if not self.set_state_client.service_is_ready():
            self.set_state_client.wait_for_service(timeout_sec=0.0)
            if not self.service_unavailable_warned:
                self.get_logger().warn(
                    'Gazebo /set_entity_state service is not available; '
                    'the odom/TF simulator is running but the Gazebo visual '
                    'entity will not move until libgazebo_ros_state.so is loaded.')
                self.service_unavailable_warned = True
            return
        if self.service_unavailable_warned:
            self.get_logger().info('Gazebo /set_entity_state service is available.')
            self.service_unavailable_warned = False
        if self.pending_state_request is not None and not self.pending_state_request.done():
            return
        if self.pending_state_request is not None:
            try:
                response = self.pending_state_request.result()
            except Exception as exc:  # pragma: no cover - depends on ROS middleware
                if not self.state_failure_warned:
                    self.get_logger().warn(
                        f'Gazebo set entity state request failed: {exc}')
                    self.state_failure_warned = True
            else:
                if not response.success and not self.state_failure_warned:
                    self.get_logger().warn('Gazebo rejected set entity state request.')
                    self.state_failure_warned = True
                elif response.success:
                    self.state_failure_warned = False

        state = EntityState()
        state.name = self.entity_name
        state.reference_frame = 'world'
        state.pose.position.x = self.x
        state.pose.position.y = self.y
        state.pose.position.z = self.z
        state.pose.orientation = _quaternion_from_yaw(self.yaw)
        state.twist = _entity_state_twist(cmd, self.send_entity_twist)

        request = SetEntityState.Request()
        request.state = state
        self.pending_state_request = self.set_state_client.call_async(request)


def _entity_state_twist(cmd: Twist, send_entity_twist: bool) -> Twist:
    if send_entity_twist:
        return cmd
    return Twist()


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimMecanumBase()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
