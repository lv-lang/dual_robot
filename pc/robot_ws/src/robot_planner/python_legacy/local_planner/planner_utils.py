#!/usr/bin/env python3

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Sequence, Tuple


Point2D = Tuple[float, float]


class PlannerState(str, Enum):
    TRACK_PATH = "TRACK_PATH"
    SIDESTEP_LEFT = "SIDESTEP_LEFT"
    SIDESTEP_RIGHT = "SIDESTEP_RIGHT"
    AVOID_OBSTACLE = "AVOID_OBSTACLE"
    REJOIN_PATH = "REJOIN_PATH"
    GOAL_REACHED = "GOAL_REACHED"
    BLOCKED_STOP = "BLOCKED_STOP"
    STALE_ODOM = "STALE_ODOM"
    STALE_SCAN = "STALE_SCAN"
    EMPTY_PATH = "EMPTY_PATH"


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Velocity2D:
    vx: float
    vy: float
    wz: float

    def planar_speed(self) -> float:
        return math.hypot(self.vx, self.vy)


@dataclass(frozen=True)
class Trajectory2D:
    command: Velocity2D
    poses: List[Pose2D]


@dataclass(frozen=True)
class PlannerResult:
    best_cmd: Velocity2D
    best_trajectory: List[Pose2D]
    planner_state: PlannerState
    valid: bool
    reason: str = ""
    debug: Dict[str, float] = field(default_factory=dict)


def zero_velocity() -> Velocity2D:
    return Velocity2D(0.0, 0.0, 0.0)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def distance_to_path(point: Point2D, path: Sequence[Point2D]) -> float:
    if not path:
        return math.inf
    if len(path) == 1:
        return distance(point, path[0])
    return min(_distance_to_segment(point, path[index], path[index + 1]) for index in range(len(path) - 1))


def nearest_path_index(point: Point2D, path: Sequence[Point2D]) -> int:
    if not path:
        return 0
    return min(range(len(path)), key=lambda index: distance(point, path[index]))


def lookahead_point(point: Point2D, path: Sequence[Point2D], lookahead_distance: float) -> Point2D:
    if not path:
        return point
    start_index = nearest_path_index(point, path)
    travelled = 0.0
    previous = path[start_index]
    for current in path[start_index + 1 :]:
        segment = distance(previous, current)
        travelled += segment
        if travelled >= lookahead_distance:
            return current
        previous = current
    return path[-1]


def path_heading_near(point: Point2D, path: Sequence[Point2D]) -> float:
    if len(path) < 2:
        return 0.0
    index = nearest_path_index(point, path)
    if index >= len(path) - 1:
        index = len(path) - 2
    start = path[index]
    end = path[index + 1]
    return math.atan2(end[1] - start[1], end[0] - start[0])


def robot_points_to_world(pose: Pose2D, points: Sequence[Point2D]) -> List[Point2D]:
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    return [
        (
            pose.x + cos_yaw * x - sin_yaw * y,
            pose.y + sin_yaw * x + cos_yaw * y,
        )
        for x, y in points
    ]


def world_points_to_body(pose: Pose2D, points: Sequence[Point2D]) -> List[Point2D]:
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    body_points = []
    for x_world, y_world in points:
        dx = x_world - pose.x
        dy = y_world - pose.y
        body_points.append((cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy))
    return body_points


def _distance_to_segment(point: Point2D, start: Point2D, end: Point2D) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared < 1e-12:
        return distance(point, start)
    ratio = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared
    ratio = clamp(ratio, 0.0, 1.0)
    projected = (start[0] + ratio * dx, start[1] + ratio * dy)
    return distance(point, projected)
