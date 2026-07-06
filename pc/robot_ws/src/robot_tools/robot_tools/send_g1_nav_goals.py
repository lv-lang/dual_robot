import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_tools.nav2_goal_utils import (
    ROBOT_NAMES,
    format_status,
    normalize_angle,
    normalize_frame_id,
    quaternion_from_yaw,
    raise_on_forbidden_topics,
    wait_future,
    yaw_from_quaternion,
)


ROBOT_ROUTES = ROBOT_NAMES


@dataclass(frozen=True)
class G1Point:
    x: float
    y: float
    yaw: float


class G1GoalRunner(Node):
    def __init__(self, config_path: Path, action_timeout_sec: float) -> None:
        super().__init__('send_g1_nav_goals')
        self.config_path = config_path
        self.action_timeout_sec = action_timeout_sec
        self.frame_id, self.position_tolerance, self.yaw_tolerance, self.points, self.routes = (
            _load_config(config_path)
        )
        self.odom_by_robot: Dict[str, Odometry] = {}
        self.action_clients = {
            robot: ActionClient(self, NavigateToPose, f'/{robot}/navigate_to_pose')
            for robot in ROBOT_ROUTES
        }
        self.lifecycle_clients = {
            robot: self.create_client(GetState, f'/{robot}/bt_navigator/get_state')
            for robot in ROBOT_ROUTES
        }
        for robot in ROBOT_ROUTES:
            self.create_subscription(
                Odometry,
                f'/{robot}/odom',
                lambda msg, robot_name=robot: self._on_odom(robot_name, msg),
                10,
            )

    def _on_odom(self, robot_name: str, msg: Odometry) -> None:
        self.odom_by_robot[robot_name] = msg

    def run(self) -> bool:
        self._raise_on_forbidden_topics('before goals')

        for robot in ROBOT_ROUTES:
            self._wait_for_action_server(robot)
            self._wait_for_bt_navigator_active(robot)

        all_ok = True
        for robot in ROBOT_ROUTES:
            route_ok = self._run_route(robot)
            final_ok = self._check_final_pose(robot)
            all_ok = all_ok and route_ok and final_ok

        self._raise_on_forbidden_topics('after goals')
        return all_ok

    def _wait_for_action_server(self, robot: str) -> None:
        client = self.action_clients[robot]
        self.get_logger().info(f'waiting for /{robot}/navigate_to_pose')
        if not client.wait_for_server(timeout_sec=self.action_timeout_sec):
            raise RuntimeError(f'/{robot}/navigate_to_pose action server not available')

    def _wait_for_bt_navigator_active(self, robot: str) -> None:
        client = self.lifecycle_clients[robot]
        self.get_logger().info(f'waiting for /{robot}/bt_navigator active')
        if not client.wait_for_service(timeout_sec=self.action_timeout_sec):
            raise RuntimeError(f'/{robot}/bt_navigator/get_state service not available')

        start = time.monotonic()
        while rclpy.ok():
            request = GetState.Request()
            future = client.call_async(request)
            self._wait_future(future, f'{robot} bt_navigator state')
            state_id = future.result().current_state.id
            if state_id == State.PRIMARY_STATE_ACTIVE:
                return
            if time.monotonic() - start > self.action_timeout_sec:
                raise TimeoutError(f'/{robot}/bt_navigator did not become active')
            time.sleep(0.2)

    def _run_route(self, robot: str) -> bool:
        ok = True
        route = self.routes[robot]
        self.get_logger().info(f'{robot} route: {" -> ".join(route)}')
        for point_name in route:
            point = self.points[point_name]
            status = self._send_goal(robot, point_name, point)
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(
                    f'{robot} goal {point_name} failed with '
                    f'{format_status(status)}')
                ok = False
                break
            self.get_logger().info(f'{robot} goal {point_name} succeeded')
        return ok

    def _send_goal(self, robot: str, point_name: str, point: G1Point) -> int:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = point.x
        goal.pose.pose.position.y = point.y
        goal.pose.pose.orientation = quaternion_from_yaw(point.yaw)

        self.get_logger().info(
            f'{robot} sending {point_name}: x={point.x:.2f}, '
            f'y={point.y:.2f}, yaw={point.yaw:.2f}')
        send_future = self.action_clients[robot].send_goal_async(goal)
        self._wait_future(send_future, f'{robot} send goal {point_name}')
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'{robot} goal {point_name} was rejected')

        result_future = goal_handle.get_result_async()
        self._wait_future(result_future, f'{robot} result {point_name}')
        return int(result_future.result().status)

    def _wait_future(self, future, label: str) -> None:
        wait_future(self, future, self.action_timeout_sec, label)

    def _check_final_pose(self, robot: str) -> bool:
        target_name = self.routes[robot][-1]
        target = self.points[target_name]
        odom = self._wait_for_odom(robot)
        pose = odom.pose.pose
        dx = pose.position.x - target.x
        dy = pose.position.y - target.y
        distance_error = math.hypot(dx, dy)
        yaw_error = abs(
            normalize_angle(yaw_from_quaternion(pose.orientation) - target.yaw))
        ok = (
            distance_error <= self.position_tolerance
            and yaw_error <= self.yaw_tolerance
        )
        message = (
            f'{robot} final error at {target_name}: '
            f'distance={distance_error:.3f} m, yaw={yaw_error:.3f} rad')
        if ok:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
        return ok

    def _wait_for_odom(self, robot: str) -> Odometry:
        start = time.monotonic()
        while rclpy.ok() and robot not in self.odom_by_robot:
            if time.monotonic() - start > 5.0:
                raise TimeoutError(f'no /{robot}/odom received')
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom_by_robot[robot]

    def _raise_on_forbidden_topics(self, phase: str) -> None:
        raise_on_forbidden_topics(self, phase, ROBOT_ROUTES)


def _load_config(path: Path):
    with path.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    frame_id = normalize_frame_id(data.get('frame_id', 'map'))
    position_tolerance = float(data.get('position_tolerance_m', 0.35))
    yaw_tolerance = float(data.get('yaw_tolerance_rad', 1.0))
    points = {
        name: G1Point(
            x=float(value['x']),
            y=float(value['y']),
            yaw=float(value.get('yaw', 0.0)),
        )
        for name, value in data['points'].items()
    }
    routes = {
        robot: [str(point_name) for point_name in data['routes'][robot]]
        for robot in ROBOT_ROUTES
    }
    missing = {
        point_name
        for route in routes.values()
        for point_name in route
        if point_name not in points
    }
    if missing:
        raise ValueError(f'route references unknown G1 points: {sorted(missing)}')
    return frame_id, position_tolerance, yaw_tolerance, points, routes


def _default_config_path() -> Path:
    try:
        share = get_package_share_directory('robot_bringup')
        return Path(share) / 'config' / 'g1_points.yaml'
    except PackageNotFoundError:
        return (
            Path(__file__).resolve().parents[2]
            / 'robot_bringup'
            / 'config'
            / 'g1_points.yaml'
        )


def _parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Send confirmed G1 Nav2 goals to robot1 and robot2.')
    parser.add_argument(
        '--config',
        type=Path,
        default=_default_config_path(),
        help='Path to g1_points.yaml.',
    )
    parser.add_argument(
        '--goal-timeout-sec',
        type=float,
        default=180.0,
        help='Timeout for action server waits and each NavigateToPose goal.',
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    node = G1GoalRunner(args.config, args.goal_timeout_sec)
    try:
        ok = node.run()
        return 0 if ok else 1
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
