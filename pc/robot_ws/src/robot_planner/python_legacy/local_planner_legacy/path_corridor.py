#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

from local_planner.trajectory_sampler import Pose2D


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class PathCorridorState:
    nearest_index: int
    nearest_point: Point2D
    lookahead_index: int
    lookahead_point: Point2D
    cross_track_error: float
    abs_cross_track_error: float
    tangent_heading: float
    rejoined: bool


class PathCorridor:
    """Tracks robot progress along a 2D path centerline.

    The progress index is monotonic for one corridor instance. Reusing the same
    instance across planner ticks prevents a temporary pose estimate from
    snapping the local target backward on the global path.
    """

    def __init__(
        self,
        path: Sequence[Point2D],
        lookahead_distance: float = 0.5,
        rejoin_tolerance: float = 0.15,
    ) -> None:
        self.path: List[Point2D] = _validate_path(path)
        self.lookahead_distance = max(0.0, lookahead_distance)
        self.rejoin_tolerance = max(0.0, rejoin_tolerance)
        self.progress_index = 0

    def reset(self) -> None:
        self.progress_index = 0

    def update(self, pose: Pose2D) -> PathCorridorState:
        nearest_index = nearest_path_index(pose, self.path, self.progress_index)
        self.progress_index = max(self.progress_index, nearest_index)
        nearest_index = self.progress_index

        lookahead_index, target = lookahead_point(
            self.path,
            nearest_index,
            self.lookahead_distance,
        )
        signed_error = cross_track_error(pose, self.path, nearest_index, signed=True)
        abs_error = abs(signed_error)
        return PathCorridorState(
            nearest_index=nearest_index,
            nearest_point=self.path[nearest_index],
            lookahead_index=lookahead_index,
            lookahead_point=target,
            cross_track_error=signed_error,
            abs_cross_track_error=abs_error,
            tangent_heading=path_tangent_heading(self.path, nearest_index),
            rejoined=abs_error <= self.rejoin_tolerance,
        )


def nearest_path_index(
    pose: Pose2D,
    path: Sequence[Point2D],
    start_index: int = 0,
) -> int:
    checked_path = _validate_path(path)
    start = _clamp_index(start_index, len(checked_path))
    best_index = start
    best_distance = math.inf
    for index in range(start, len(checked_path)):
        point = checked_path[index]
        distance = _distance_sq((pose.x, pose.y), point)
        if distance < best_distance:
            best_index = index
            best_distance = distance
    return best_index


def lookahead_point(
    path: Sequence[Point2D],
    start_index: int,
    lookahead_distance: float,
) -> Tuple[int, Point2D]:
    checked_path = _validate_path(path)
    index = _clamp_index(start_index, len(checked_path))
    remaining = max(0.0, lookahead_distance)
    if remaining == 0.0 or index == len(checked_path) - 1:
        return index, checked_path[index]

    current = checked_path[index]
    for next_index in range(index + 1, len(checked_path)):
        next_point = checked_path[next_index]
        segment_length = math.dist(current, next_point)
        if segment_length <= 1e-9:
            current = next_point
            continue
        if remaining <= segment_length:
            ratio = remaining / segment_length
            return (
                next_index,
                (
                    current[0] + (next_point[0] - current[0]) * ratio,
                    current[1] + (next_point[1] - current[1]) * ratio,
                ),
            )
        remaining -= segment_length
        current = next_point
    return len(checked_path) - 1, checked_path[-1]


def cross_track_error(
    pose: Pose2D,
    path: Sequence[Point2D],
    index: int,
    signed: bool = True,
) -> float:
    checked_path = _validate_path(path)
    origin_index = _clamp_index(index, len(checked_path))
    origin, target = _tangent_points(checked_path, origin_index)
    tx = target[0] - origin[0]
    ty = target[1] - origin[1]
    length = math.hypot(tx, ty)
    if length <= 1e-9:
        error = math.dist((pose.x, pose.y), origin)
        return error if signed else abs(error)

    dx = pose.x - origin[0]
    dy = pose.y - origin[1]
    signed_error = (tx * dy - ty * dx) / length
    return signed_error if signed else abs(signed_error)


def path_tangent_heading(path: Sequence[Point2D], index: int) -> float:
    checked_path = _validate_path(path)
    origin, target = _tangent_points(checked_path, _clamp_index(index, len(checked_path)))
    return math.atan2(target[1] - origin[1], target[0] - origin[0])


def is_rejoined(
    pose: Pose2D,
    path: Sequence[Point2D],
    index: int,
    rejoin_tolerance: float,
) -> bool:
    return cross_track_error(pose, path, index, signed=False) <= max(0.0, rejoin_tolerance)


def _validate_path(path: Sequence[Point2D]) -> List[Point2D]:
    checked = list(path)
    if not checked:
        raise ValueError("path must contain at least one point")
    for x, y in checked:
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("path points must be finite")
    return checked


def _clamp_index(index: int, path_length: int) -> int:
    return max(0, min(path_length - 1, int(index)))


def _tangent_points(path: Sequence[Point2D], index: int) -> Tuple[Point2D, Point2D]:
    if len(path) == 1:
        point = path[0]
        return point, (point[0] + 1.0, point[1])

    if index < len(path) - 1:
        origin = path[index]
        for next_index in range(index + 1, len(path)):
            if math.dist(origin, path[next_index]) > 1e-9:
                return origin, path[next_index]

    target = path[index]
    for previous_index in range(index - 1, -1, -1):
        if math.dist(path[previous_index], target) > 1e-9:
            return path[previous_index], target

    return target, (target[0] + 1.0, target[1])


def _distance_sq(a: Point2D, b: Point2D) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy
