import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

import rclpy
import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from robot_tools.nav2_goal_utils import quaternion_from_yaw


DEFAULT_ROBOTS = ('mecanum', 'ackermann')


@dataclass(frozen=True)
class InitialPose:
    robot: str
    point_name: str
    frame_id: str
    x: float
    y: float
    yaw: float


def load_initial_poses(config_path: Path,
                       robots: Sequence[str] = DEFAULT_ROBOTS) -> Dict[str, InitialPose]:
    with config_path.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)

    frame_id = str(data.get('frame_id', 'map')).strip().lstrip('/') or 'map'
    points = data.get('points') or {}
    robot_config = data.get('robots') or {}
    poses: Dict[str, InitialPose] = {}

    for robot in robots:
        if robot not in robot_config:
            raise ValueError(f'{robot} is missing from robots in {config_path}')
        point_name = str(robot_config[robot].get('waiting_area', '')).strip()
        if not point_name:
            raise ValueError(f'{robot} has no waiting_area in {config_path}')
        if point_name not in points:
            raise ValueError(
                f'{robot} waiting_area {point_name} is missing from points')
        point = points[point_name]
        poses[robot] = InitialPose(
            robot=robot,
            point_name=point_name,
            frame_id=frame_id,
            x=float(point['x']),
            y=float(point['y']),
            yaw=float(point.get('yaw', 0.0)),
        )

    return poses


class InitialPoseSeeder(Node):
    def __init__(
        self,
        poses: Dict[str, InitialPose],
        covariance_x: float,
        covariance_y: float,
        covariance_yaw: float,
    ) -> None:
        super().__init__('seed_real_initial_poses')
        self.poses = poses
        self.covariance_x = covariance_x
        self.covariance_y = covariance_y
        self.covariance_yaw = covariance_yaw
        self.initial_pose_publishers = {
            robot: self.create_publisher(
                PoseWithCovarianceStamped,
                f'/{robot}/initialpose',
                5,
            )
            for robot in poses
        }

    def wait_for_subscribers(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        pending = set(self.initial_pose_publishers)
        while rclpy.ok() and pending and time.monotonic() < deadline:
            for robot in list(pending):
                if self.initial_pose_publishers[robot].get_subscription_count() > 0:
                    pending.remove(robot)
            if pending:
                rclpy.spin_once(self, timeout_sec=0.1)
        if pending:
            self.get_logger().warning(
                'publishing without AMCL subscribers for: '
                + ', '.join(sorted(pending)))
            return False
        return True

    def publish_once(self) -> None:
        now = self.get_clock().now().to_msg()
        for robot, pose in self.poses.items():
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = now
            msg.header.frame_id = pose.frame_id
            msg.pose.pose.position.x = pose.x
            msg.pose.pose.position.y = pose.y
            msg.pose.pose.orientation = quaternion_from_yaw(pose.yaw)
            msg.pose.covariance[0] = self.covariance_x
            msg.pose.covariance[7] = self.covariance_y
            msg.pose.covariance[35] = self.covariance_yaw
            self.initial_pose_publishers[robot].publish(msg)
            self.get_logger().info(
                f'published /{robot}/initialpose from {pose.point_name}: '
                f'frame={pose.frame_id}, x={pose.x:.3f}, '
                f'y={pose.y:.3f}, yaw={pose.yaw:.3f}')

    def run(
        self,
        repeat: int,
        period_sec: float,
        wait_subscribers_sec: float,
    ) -> None:
        if wait_subscribers_sec > 0.0:
            self.wait_for_subscribers(wait_subscribers_sec)
        for index in range(repeat):
            self.publish_once()
            if index + 1 < repeat:
                time.sleep(period_sec)
                rclpy.spin_once(self, timeout_sec=0.0)


def _default_config_path() -> Path:
    try:
        share = get_package_share_directory('robot_bringup')
        return Path(share) / 'config' / 'real_task_points.yaml'
    except PackageNotFoundError:
        return (
            Path(__file__).resolve().parents[2]
            / 'robot_bringup'
            / 'config'
            / 'real_task_points.yaml'
        )


def _positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError('must be greater than 0')
    return parsed


def _non_negative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError('must be greater than or equal to 0')
    return parsed


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError('must be greater than 0')
    return parsed


def _parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Publish real-robot AMCL initial poses from real_task_points.yaml. '
            'Defaults seed mecanum from W1 and ackermann from W2.'
        ))
    parser.add_argument(
        '--config',
        type=Path,
        default=_default_config_path(),
        help='Path to real_task_points.yaml.',
    )
    parser.add_argument(
        '--robots',
        nargs='+',
        choices=DEFAULT_ROBOTS,
        default=list(DEFAULT_ROBOTS),
        help='Robot namespaces to seed.',
    )
    parser.add_argument(
        '--repeat',
        type=_positive_int,
        default=5,
        help='Number of initial pose messages to publish per robot.',
    )
    parser.add_argument(
        '--period-sec',
        type=_positive_float,
        default=0.5,
        help='Delay between repeated publications.',
    )
    parser.add_argument(
        '--wait-subscribers-sec',
        type=_non_negative_float,
        default=60.0,
        help='Wait for AMCL initialpose subscribers before publishing.',
    )
    parser.add_argument(
        '--covariance-x',
        type=_non_negative_float,
        default=0.25,
        help='Initial pose x covariance.',
    )
    parser.add_argument(
        '--covariance-y',
        type=_non_negative_float,
        default=0.25,
        help='Initial pose y covariance.',
    )
    parser.add_argument(
        '--covariance-yaw',
        type=_non_negative_float,
        default=0.06853891909122467,
        help='Initial pose yaw covariance.',
    )
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv: Optional[List[str]] = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    try:
        poses = load_initial_poses(args.config, args.robots)
    except Exception as exc:  # noqa: BLE001
        print(f'failed to load initial poses: {exc}', file=sys.stderr)
        return 1

    rclpy.init()
    node = InitialPoseSeeder(
        poses,
        args.covariance_x,
        args.covariance_y,
        args.covariance_yaw,
    )
    try:
        node.run(args.repeat, args.period_sec, args.wait_subscribers_sec)
        return 0
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
