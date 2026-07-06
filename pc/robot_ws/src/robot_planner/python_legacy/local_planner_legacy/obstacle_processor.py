#!/usr/bin/env python3

import math
from typing import List, Sequence, Tuple

from local_planner.trajectory_sampler import Pose2D


Point2D = Tuple[float, float]


def scan_to_obstacle_points(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    obstacle_range: float,
    angle_stride: int = 1,
) -> List[Point2D]:
    if angle_increment == 0.0:
        return []

    stride = max(1, int(angle_stride))
    points: List[Point2D] = []
    max_range = obstacle_range
    if math.isfinite(range_max) and range_max > 0.0:
        max_range = min(max_range, range_max)

    for index in range(0, len(ranges), stride):
        measured = ranges[index]
        if not _valid_range(measured, range_min, max_range):
            continue
        angle = angle_min + angle_increment * index
        points.append((measured * math.cos(angle), measured * math.sin(angle)))
    return points


def robot_to_world_points(
    pose: Pose2D,
    points_robot_frame: Sequence[Point2D],
) -> List[Point2D]:
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    return [
        (
            pose.x + cos_yaw * x - sin_yaw * y,
            pose.y + sin_yaw * x + cos_yaw * y,
        )
        for x, y in points_robot_frame
        if math.isfinite(x) and math.isfinite(y)
    ]


def _valid_range(value: float, range_min: float, range_max: float) -> bool:
    return (
        math.isfinite(value)
        and value >= max(0.0, range_min)
        and (range_max <= 0.0 or value <= range_max)
    )
