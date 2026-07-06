import math
import time
from typing import Iterable, Sequence, Set

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion


ROBOT_NAMES = ('mecanum', 'ackermann', 'robot1', 'robot2')
FORBIDDEN_GLOBAL_TOPICS = ('/cmd_vel', '/odom', '/scan')
NAMESPACED_TOPIC_SUFFIXES = ('cmd_vel', 'odom', 'scan')

STATUS_LABELS = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'FAILED',
}


def normalize_frame_id(frame_id: str) -> str:
    normalized = str(frame_id).strip().lstrip('/')
    return normalized or 'map'


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def status_to_label(status: int) -> str:
    return STATUS_LABELS.get(int(status), f'UNKNOWN_STATUS_{int(status)}')


def format_status(status: int) -> str:
    return f'{status_to_label(status)} (status={int(status)})'


def wait_future(node, future, timeout_sec: float, label: str) -> None:
    start = time.monotonic()
    while rclpy.ok() and not future.done():
        if time.monotonic() - start > timeout_sec:
            raise TimeoutError(f'timed out waiting for {label}')
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        raise RuntimeError(f'interrupted while waiting for {label}')


def topic_names(node) -> Set[str]:
    return {name for name, _ in node.get_topic_names_and_types()}


def forbidden_topics(robots: Iterable[str] = ROBOT_NAMES) -> Set[str]:
    topics = set(FORBIDDEN_GLOBAL_TOPICS)
    topics.update(f'/{robot}/map' for robot in robots)
    return topics


def required_robot_topics(robots: Iterable[str]) -> Set[str]:
    return {
        f'/{robot}/{suffix}'
        for robot in robots
        for suffix in NAMESPACED_TOPIC_SUFFIXES
    }


def raise_on_forbidden_topics(node, phase: str,
                              robots: Sequence[str] = ROBOT_NAMES) -> None:
    forbidden = sorted(topic_names(node) & forbidden_topics(robots))
    if forbidden:
        raise RuntimeError(
            f'forbidden topics present {phase}: {", ".join(forbidden)}')
