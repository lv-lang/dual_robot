import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanSelfFilter(Node):
    """Remove robot body returns from the Gazebo laser scan."""

    def __init__(self) -> None:
        super().__init__('scan_self_filter')

        self.declare_parameter('input_topic', 'scan_raw')
        self.declare_parameter('output_topic', 'scan')
        self.declare_parameter('self_filter_radius_m', 0.30)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.self_filter_radius_m = float(
            self.get_parameter('self_filter_radius_m').value)

        self.publisher = self.create_publisher(LaserScan, output_topic, 10)
        self.create_subscription(LaserScan, input_topic, self._scan_callback, 10)
        self.get_logger().info(
            f'scan_self_filter active: {input_topic} -> {output_topic}, '
            f'filter <= {self.self_filter_radius_m:.2f} m')

    def _scan_callback(self, msg: LaserScan) -> None:
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.ranges = filter_self_ranges(
            msg.ranges,
            max(float(msg.range_min), self.self_filter_radius_m),
        )
        filtered.intensities = msg.intensities
        self.publisher.publish(filtered)


def filter_self_ranges(ranges, self_filter_radius_m):
    return [
        math.inf if math.isfinite(value) and value <= self_filter_radius_m else value
        for value in ranges
    ]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanSelfFilter()
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
