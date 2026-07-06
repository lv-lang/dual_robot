#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

from local_planner.trajectory_sampler import Pose2D


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class FootprintConfig:
    robot_length: float = 0.24
    robot_width: float = 0.20
    obstacle_margin: float = 0.05
    front_extension: float = 0.0
    side_extension: float = 0.0


class FootprintCollisionChecker:
    """Checks an inflated rectangular mecanum footprint against point obstacles."""

    def __init__(self, config: FootprintConfig) -> None:
        self.config = config
        self.half_length = 0.5 * max(0.0, config.robot_length) + config.obstacle_margin
        self.half_width = 0.5 * max(0.0, config.robot_width) + config.obstacle_margin
        self.front_length = self.half_length + max(0.0, config.front_extension)
        self.side_width = self.half_width + max(0.0, config.side_extension)

    def collides(self, pose: Pose2D, obstacles_world: Sequence[Point2D]) -> bool:
        return self.clearance(pose, obstacles_world) <= 0.0

    def trajectory_collides(
        self,
        trajectory: Sequence[Pose2D],
        obstacles_world: Sequence[Point2D],
    ) -> bool:
        return self.trajectory_clearance(trajectory, obstacles_world) <= 0.0

    def trajectory_clearance(
        self,
        trajectory: Sequence[Pose2D],
        obstacles_world: Sequence[Point2D],
    ) -> float:
        if not obstacles_world:
            return math.inf
        min_clearance = math.inf
        for pose in trajectory:
            min_clearance = min(min_clearance, self.clearance(pose, obstacles_world))
        return min_clearance

    def clearance(self, pose: Pose2D, obstacles_world: Sequence[Point2D]) -> float:
        if not obstacles_world:
            return math.inf
        min_clearance = math.inf
        for x_body, y_body in self.obstacles_in_body_frame(pose, obstacles_world):
            min_clearance = min(min_clearance, self._rectangle_clearance(x_body, y_body))
        return min_clearance

    def front_clearance(
        self,
        pose: Pose2D,
        obstacles_world: Sequence[Point2D],
        lateral_half_width: float = 0.0,
    ) -> float:
        half_width = lateral_half_width if lateral_half_width > 0.0 else self.half_width
        min_clearance = math.inf
        for x_body, y_body in self.obstacles_in_body_frame(pose, obstacles_world):
            if x_body < self.half_length or abs(y_body) > half_width:
                continue
            min_clearance = min(min_clearance, x_body - self.half_length)
        return min_clearance

    def side_clearance(
        self,
        pose: Pose2D,
        obstacles_world: Sequence[Point2D],
        side: str,
        longitudinal_half_length: float = 0.0,
    ) -> float:
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        half_length = longitudinal_half_length if longitudinal_half_length > 0.0 else self.front_length
        min_clearance = math.inf
        for x_body, y_body in self.obstacles_in_body_frame(pose, obstacles_world):
            if abs(x_body) > half_length:
                continue
            if side == "left" and y_body >= self.half_width:
                min_clearance = min(min_clearance, y_body - self.half_width)
            elif side == "right" and y_body <= -self.half_width:
                min_clearance = min(min_clearance, -self.half_width - y_body)
        return min_clearance

    def rectangle_corners(self, pose: Pose2D) -> Tuple[Point2D, Point2D, Point2D, Point2D]:
        corners_body = (
            (self.half_length, self.half_width),
            (self.half_length, -self.half_width),
            (-self.half_length, -self.half_width),
            (-self.half_length, self.half_width),
        )
        cos_yaw = math.cos(pose.yaw)
        sin_yaw = math.sin(pose.yaw)
        return tuple(
            (
                pose.x + cos_yaw * x_body - sin_yaw * y_body,
                pose.y + sin_yaw * x_body + cos_yaw * y_body,
            )
            for x_body, y_body in corners_body
        )

    def obstacles_in_body_frame(
        self,
        pose: Pose2D,
        obstacles_world: Sequence[Point2D],
    ) -> Tuple[Point2D, ...]:
        cos_yaw = math.cos(pose.yaw)
        sin_yaw = math.sin(pose.yaw)
        points = []
        for obstacle_x, obstacle_y in obstacles_world:
            dx = obstacle_x - pose.x
            dy = obstacle_y - pose.y
            points.append((cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy))
        return tuple(points)

    def _rectangle_clearance(self, x_body: float, y_body: float) -> float:
        dx = abs(x_body) - self.half_length
        dy = abs(y_body) - self.half_width
        outside_x = max(dx, 0.0)
        outside_y = max(dy, 0.0)
        if dx <= 0.0 and dy <= 0.0:
            return max(dx, dy)
        return math.hypot(outside_x, outside_y)
