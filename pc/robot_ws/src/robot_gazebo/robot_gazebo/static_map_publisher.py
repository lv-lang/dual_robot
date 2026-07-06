from pathlib import Path
import re

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class StaticMapPublisher(Node):
    """Publish a small static OccupancyGrid fixture for robot1 planners."""

    def __init__(self):
        super().__init__('static_map_publisher')
        self.declare_parameter('topic', 'map')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('width', 120)
        self.declare_parameter('height', 120)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('border_occupied', True)
        self.declare_parameter('publish_period', 1.0)

        topic = self.get_parameter('topic').value
        period = float(self.get_parameter('publish_period').value)
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.publisher = self.create_publisher(OccupancyGrid, topic, qos)
        self.map_msg = self._load_map_or_build_test_map()
        self.timer = self.create_timer(period, self._publish)
        self._publish()

    def _load_map_or_build_test_map(self):
        map_yaml = str(self.get_parameter('map_yaml').value).strip()
        if not map_yaml:
            return self._build_map()
        self.get_logger().info(f'loading OccupancyGrid from {map_yaml}')
        return self._load_map_from_yaml(Path(map_yaml))

    def _build_map(self):
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        resolution = float(self.get_parameter('resolution').value)
        border_occupied = bool(self.get_parameter('border_occupied').value)

        msg = OccupancyGrid()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = -0.5 * width * resolution
        msg.info.origin.position.y = -0.5 * height * resolution
        msg.info.origin.orientation.w = 1.0

        data = [0] * (width * height)
        if border_occupied and width > 1 and height > 1:
            for x in range(width):
                data[x] = 100
                data[(height - 1) * width + x] = 100
            for y in range(height):
                data[y * width] = 100
                data[y * width + width - 1] = 100
        msg.data = data
        return msg

    def _load_map_from_yaml(self, yaml_path):
        config = _read_simple_yaml(yaml_path)
        image_path = Path(config['image'])
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path

        width, height, pixels = _read_pgm(image_path)
        resolution = float(config.get('resolution', 0.05))
        origin = _parse_origin(config.get('origin', '[0.0, 0.0, 0.0]'))
        negate = int(config.get('negate', 0))
        occupied_thresh = float(config.get('occupied_thresh', 0.65))
        free_thresh = float(config.get('free_thresh', 0.196))

        msg = OccupancyGrid()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = origin[0]
        msg.info.origin.position.y = origin[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        data = [0] * (width * height)
        for y in range(height):
            src_row = height - 1 - y
            for x in range(width):
                pixel = pixels[src_row * width + x]
                if negate:
                    occupancy = pixel / 255.0
                else:
                    occupancy = (255 - pixel) / 255.0

                if occupancy > occupied_thresh:
                    value = 100
                elif occupancy < free_thresh:
                    value = 0
                else:
                    value = -1
                data[y * width + x] = value
        msg.data = data
        return msg

    def _publish(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.map_msg)


def _read_simple_yaml(path):
    config = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith('#') or ':' not in stripped:
            continue
        key, value = stripped.split(':', 1)
        config[key.strip()] = value.strip()
    if 'image' not in config:
        raise ValueError(f'map yaml {path} does not define image')
    return config


def _parse_origin(value):
    numbers = [float(item) for item in re.findall(r'[-+]?\d+(?:\.\d+)?', value)]
    if len(numbers) < 2:
        raise ValueError(f'invalid map origin: {value}')
    return numbers


def _read_pgm(path):
    tokens = []
    with path.open('rb') as stream:
        while len(tokens) < 4:
            line = stream.readline()
            if not line:
                raise ValueError(f'invalid PGM header in {path}')
            line = line.split(b'#', 1)[0].strip()
            if line:
                tokens.extend(line.split())

        magic = tokens[0]
        if magic != b'P5':
            raise ValueError(f'only binary P5 PGM maps are supported: {path}')
        width = int(tokens[1])
        height = int(tokens[2])
        max_value = int(tokens[3])
        if max_value != 255:
            raise ValueError(f'only 8-bit PGM maps are supported: {path}')

        pixels = stream.read(width * height)
    if len(pixels) != width * height:
        raise ValueError(f'PGM data size mismatch in {path}')
    return width, height, pixels


def main(args=None):
    rclpy.init(args=args)
    node = StaticMapPublisher()
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
