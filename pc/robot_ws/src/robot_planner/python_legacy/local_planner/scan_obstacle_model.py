#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Iterable, List, Tuple

from local_planner.dwa_lite_params import DwaLiteParams


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class ObstacleSummary:
    front_clearance: float
    left_clearance: float
    right_clearance: float
    nearest_clearance: float
    front_blocked: bool
    hard_stop: bool
    preferred_side: str


class ScanObstacleModel:
    """Summarises scan-derived obstacle points in the robot frame."""

    def __init__(self, params: DwaLiteParams) -> None:
        self.params = params
        self._half_width = 0.5 * params.robot_width + params.safety_margin
        self._half_length = 0.5 * params.robot_length + params.safety_margin

    def summarize(self, obstacle_points_robot: Iterable[Point2D]) -> ObstacleSummary:
        points = _finite_points(obstacle_points_robot)
        front_clearance = math.inf
        left_clearance = math.inf
        right_clearance = math.inf
        nearest_clearance = math.inf

        front_half_width = max(self._half_width, 0.5 * self.params.front_check_width)
        for x, y in points:
            nearest_clearance = min(nearest_clearance, math.hypot(x, y))
            if 0.0 <= x <= self.params.front_check_distance and abs(y) <= front_half_width:
                front_clearance = min(front_clearance, x)
            if abs(x) <= self.params.side_check_distance and y >= self._half_width:
                left_clearance = min(left_clearance, y - self._half_width)
            if abs(x) <= self.params.side_check_distance and y <= -self._half_width:
                right_clearance = min(right_clearance, -self._half_width - y)

        front_blocked = front_clearance <= self.params.front_check_distance
        hard_stop = front_clearance <= self.params.hard_stop_distance
        preferred_side = "left" if left_clearance >= right_clearance else "right"
        if math.isinf(left_clearance) and math.isinf(right_clearance):
            preferred_side = "left"
        return ObstacleSummary(
            front_clearance=front_clearance,
            left_clearance=left_clearance,
            right_clearance=right_clearance,
            nearest_clearance=nearest_clearance,
            front_blocked=front_blocked,
            hard_stop=hard_stop,
            preferred_side=preferred_side,
        )


def _finite_points(points: Iterable[Point2D]) -> List[Point2D]:
    finite = []
    for x, y in points:
        if math.isfinite(x) and math.isfinite(y):
            finite.append((float(x), float(y)))
    return finite


def downsample_obstacle_points(
    points: Iterable[Point2D],
    max_points: int,
) -> List[Point2D]:
    finite = _finite_points(points)
    if max_points <= 0:
        return []
    if len(finite) <= max_points:
        return finite

    nearest_quota = max(1, max_points // 2)
    nearest = sorted(finite, key=lambda point: point[0] * point[0] + point[1] * point[1])[
        :nearest_quota
    ]
    selected = []
    selected_keys = set()

    def add(point: Point2D) -> None:
        key = (round(point[0], 4), round(point[1], 4))
        if key in selected_keys or len(selected) >= max_points:
            return
        selected_keys.add(key)
        selected.append(point)

    for point in nearest:
        add(point)

    remaining = max(1, max_points - len(selected))
    stride = max(1, math.ceil(len(finite) / remaining))
    for point in finite[::stride]:
        add(point)
        if len(selected) >= max_points:
            break
    return selected
