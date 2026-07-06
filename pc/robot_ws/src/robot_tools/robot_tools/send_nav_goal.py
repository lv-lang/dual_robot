import argparse
import sys
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_tools.nav2_goal_utils import (
    ROBOT_NAMES,
    format_status,
    normalize_frame_id,
    quaternion_from_yaw,
    wait_future,
)


class NavGoalSender(Node):
    def __init__(
        self,
        robot: str,
        x: float,
        y: float,
        yaw: float,
        frame_id: str,
        timeout_sec: float,
    ) -> None:
        super().__init__('send_nav_goal')
        self.robot = robot
        self.x = x
        self.y = y
        self.yaw = yaw
        self.frame_id = normalize_frame_id(frame_id)
        self.timeout_sec = timeout_sec
        self.action_name = f'/{robot}/navigate_to_pose'
        self.client = ActionClient(self, NavigateToPose, self.action_name)

    def run(self) -> int:
        self.get_logger().info(
            f'waiting for {self.action_name} '
            f'(timeout {self.timeout_sec:.1f} sec)')
        if not self.client.wait_for_server(timeout_sec=self.timeout_sec):
            raise TimeoutError(f'{self.action_name} action server not available')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.x
        goal.pose.pose.position.y = self.y
        goal.pose.pose.orientation = quaternion_from_yaw(self.yaw)

        self.get_logger().info(
            f'sending {self.robot} goal: frame={self.frame_id}, '
            f'x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}')
        send_future = self.client.send_goal_async(goal)
        wait_future(self, send_future, self.timeout_sec, 'goal acceptance')
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print(f'{self.robot} NavigateToPose result: REJECTED')
            return GoalStatus.STATUS_UNKNOWN

        result_future = goal_handle.get_result_async()
        try:
            wait_future(self, result_future, self.timeout_sec, 'goal result')
        except TimeoutError:
            self.get_logger().error(
                f'timed out waiting for {self.robot} result; canceling goal')
            cancel_future = goal_handle.cancel_goal_async()
            wait_future(self, cancel_future, 5.0, 'goal cancel')
            raise

        status = int(result_future.result().status)
        print(f'{self.robot} NavigateToPose result: {format_status(status)}')
        return status


def _positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError('must be greater than 0')
    return parsed


def _parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Send one Nav2 NavigateToPose goal to one real robot.')
    parser.add_argument(
        '--robot',
        required=True,
        choices=ROBOT_NAMES,
        help='Robot namespace to target.',
    )
    parser.add_argument('--x', required=True, type=float, help='Goal x in map.')
    parser.add_argument('--y', required=True, type=float, help='Goal y in map.')
    parser.add_argument(
        '--yaw',
        required=True,
        type=float,
        help='Goal yaw in radians.',
    )
    parser.add_argument(
        '--frame-id',
        default='map',
        help='Goal frame id. Use map for the shared /map contract.',
    )
    parser.add_argument(
        '--timeout-sec',
        type=_positive_float,
        default=120.0,
        help='Timeout for action server wait and goal result.',
    )
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    node = NavGoalSender(
        args.robot,
        args.x,
        args.y,
        args.yaw,
        args.frame_id,
        args.timeout_sec,
    )
    try:
        status = node.run()
        return 0 if status == GoalStatus.STATUS_SUCCEEDED else 1
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
