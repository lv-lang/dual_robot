from typing import List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage


class TfNamespaceRelay(Node):
    """Mirror robot-namespaced TF topics onto global TF topics for RViz."""

    def __init__(self):
        super().__init__('tf_namespace_relay')
        self.declare_parameter('robot_names', ['robot1', 'robot2'])
        robot_names = [
            str(name).strip().strip('/')
            for name in self.get_parameter('robot_names').value
            if str(name).strip()
        ]
        if not robot_names:
            raise ValueError('robot_names must contain at least one namespace')

        dynamic_qos = QoSProfile(depth=100)
        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.tf_pub = self.create_publisher(TFMessage, '/tf', dynamic_qos)
        self.tf_static_pub = self.create_publisher(
            TFMessage, '/tf_static', static_qos)
        self.static_cache = {}

        for robot_name in robot_names:
            self.create_subscription(
                TFMessage,
                f'/{robot_name}/tf',
                self._on_dynamic_tf,
                dynamic_qos,
            )
            self.create_subscription(
                TFMessage,
                f'/{robot_name}/tf_static',
                self._on_static_tf,
                static_qos,
            )

        self.timer = self.create_timer(2.0, self._republish_static_cache)
        self.get_logger().info(
            'relaying TF for namespaces: ' + ', '.join(robot_names))

    def _on_dynamic_tf(self, msg: TFMessage) -> None:
        if msg.transforms:
            self.tf_pub.publish(msg)

    def _on_static_tf(self, msg: TFMessage) -> None:
        if not msg.transforms:
            return
        for transform in msg.transforms:
            self.static_cache[transform.child_frame_id] = transform
        self._republish_static_cache()

    def _republish_static_cache(self) -> None:
        if self.static_cache:
            msg = TFMessage()
            msg.transforms = list(self.static_cache.values())
            self.tf_static_pub.publish(msg)


def main(args: List[str] = None):
    rclpy.init(args=args)
    node = TfNamespaceRelay()
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
