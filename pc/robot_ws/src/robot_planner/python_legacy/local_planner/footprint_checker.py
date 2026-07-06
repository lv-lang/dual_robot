#!/usr/bin/env python3

import math
from typing import Sequence

from local_planner.dwa_lite_params import DwaLiteParams
from local_planner.planner_utils import Point2D, Pose2D


class DwaLiteFootprintChecker:
    """Inflated rectangular footprint collision checker."""

    def __init__(self, params: DwaLiteParams) -> None:
        self.params = params
        self.half_length = 0.5 * params.robot_length + params.safety_margin
        self.half_width = 0.5 * params.robot_width + params.safety_margin

    def collides(self, pose: Pose2D, obstacles_world: Sequence[Point2D]) -> bool:
        return self.clearance(pose, obstacles_world) <= 0.0

    def trajectory_collides(self, trajectory: Sequence[Pose2D], obstacles_world: Sequence[Point2D]) -> bool:
        return self.trajectory_clearance(trajectory, obstacles_world) <= 0.0

    def trajectory_clearance(self, trajectory: Sequence[Pose2D], obstacles_world: Sequence[Point2D]) -> float:
        if not obstacles_world:
            return math.inf
        minimum = math.inf
        for pose in trajectory:
            clearance = self.clearance(pose, obstacles_world)
            if clearance <= 0.0:
                return clearance
            minimum = min(minimum, clearance)
        return minimum

    def clearance(self, pose: Pose2D, obstacles_world: Sequence[Point2D]) -> float:
        if not obstacles_world:
            return math.inf
        cos_yaw = math.cos(pose.yaw)
        sin_yaw = math.sin(pose.yaw)
        minimum = math.inf
        for x_world, y_world in obstacles_world:
            dx = x_world - pose.x
            dy = y_world - pose.y
            x_body = cos_yaw * dx + sin_yaw * dy
            y_body = -sin_yaw * dx + cos_yaw * dy
            clearance = self._rectangle_clearance(x_body, y_body)
            if clearance <= 0.0:
                return clearance
            minimum = min(minimum, clearance)
        return minimum

    def _rectangle_clearance(self, x_body: float, y_body: float) -> float:
        dx = abs(x_body) - self.half_length
        dy = abs(y_body) - self.half_width
        if dx <= 0.0 and dy <= 0.0:
            return max(dx, dy)
        return math.hypot(max(dx, 0.0), max(dy, 0.0))
