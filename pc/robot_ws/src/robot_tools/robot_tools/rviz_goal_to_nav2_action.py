import sys
from typing import Dict, List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_tools.nav2_goal_utils import ROBOT_NAMES, format_status, normalize_frame_id


DEFAULT_ROBOTS = ('mecanum', 'ackermann')


class RvizGoalToNav2Action(Node):
    """Bridge low-rate RViz SetGoal topics to each robot's local Nav2 action."""

    def __init__(self) -> None:
        super().__init__('rviz_goal_to_nav2_action')
        self.declare_parameter('robots', list(DEFAULT_ROBOTS))
        self.declare_parameter('wait_for_server_sec', 0.1)
        self.robots = self._robots_from_parameter()
        self.wait_for_server_sec = float(
            self.get_parameter('wait_for_server_sec').value)
        self.nav2_clients: Dict[str, ActionClient] = {
            robot: ActionClient(self, NavigateToPose, f'/{robot}/navigate_to_pose')
            for robot in self.robots
        }
        self.nav2_goal_handles = {}
        for robot in self.robots:
            self.create_subscription(
                PoseStamped,
                f'/{robot}/goal_pose',
                lambda msg, robot_name=robot: self._on_goal(robot_name, msg),
                1,
            )
        self.get_logger().info(
            'RViz goal relay ready for: ' + ', '.join(self.robots))

    def _robots_from_parameter(self) -> List[str]:
        value = self.get_parameter('robots').value
        robots = [str(robot).strip().strip('/') for robot in value if str(robot).strip()]
        invalid = sorted(set(robots) - set(ROBOT_NAMES))
        if invalid:
            raise ValueError(f'unsupported robot namespace(s): {", ".join(invalid)}')
        if not robots:
            raise ValueError('robots parameter must not be empty')
        return robots

    def _on_goal(self, robot: str, msg: PoseStamped) -> None:
        client = self.nav2_clients[robot]
        action_name = f'/{robot}/navigate_to_pose'
        if not client.server_is_ready():
            if not client.wait_for_server(timeout_sec=self.wait_for_server_sec):
                self.get_logger().error(f'{action_name} action server is not ready')
                return

        previous_goal = self.nav2_goal_handles.get(robot)
        if previous_goal is not None:
            previous_goal.cancel_goal_async()

        goal = NavigateToPose.Goal()
        goal.pose = msg
        goal.pose.header.frame_id = normalize_frame_id(goal.pose.header.frame_id)
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f'sending RViz goal to {action_name}: '
            f'frame={goal.pose.header.frame_id}, '
            f'x={goal.pose.pose.position.x:.3f}, '
            f'y={goal.pose.pose.position.y:.3f}')
        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future, robot_name=robot: self._on_goal_response(robot_name, future))

    def _on_goal_response(self, robot: str, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'/{robot}/navigate_to_pose rejected RViz goal')
            return
        self.nav2_goal_handles[robot] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, robot_name=robot: self._on_goal_result(robot_name, result))

    def _on_goal_result(self, robot: str, future) -> None:
        status = int(future.result().status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'{robot} RViz goal result: {format_status(status)}')
        else:
            self.get_logger().warning(
                f'{robot} RViz goal result: {format_status(status)}')
        self.nav2_goal_handles.pop(robot, None)


def main(argv: Optional[List[str]] = None) -> int:
    _ = argv
    rclpy.init(args=sys.argv if argv is None else argv)
    node = RvizGoalToNav2Action()
    try:
        rclpy.spin(node)
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
