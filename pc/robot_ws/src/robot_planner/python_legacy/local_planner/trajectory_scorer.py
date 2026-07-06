#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Sequence

from local_planner.dwa_lite_params import DwaLiteParams
from local_planner.footprint_checker import DwaLiteFootprintChecker
from local_planner.planner_utils import (
    Point2D,
    Trajectory2D,
    Velocity2D,
    distance,
    distance_to_path,
    lookahead_point,
    normalize_angle,
)
from local_planner.scan_obstacle_model import ObstacleSummary


@dataclass(frozen=True)
class DwaLiteTrajectoryScore:
    path_cost: float
    goal_cost: float
    obstacle_cost: float
    heading_cost: float
    velocity_cost: float
    smoothness_cost: float
    lateral_cost: float
    total_cost: float
    feasible: bool
    rejection_reason: str = ""


class DwaLiteTrajectoryScorer:
    """Scores feasible DWA Lite trajectories after hard collision rejection."""

    def __init__(self, params: DwaLiteParams, footprint_checker: DwaLiteFootprintChecker) -> None:
        self.params = params
        self.footprint_checker = footprint_checker

    def score(
        self,
        trajectory: Trajectory2D,
        path: Sequence[Point2D],
        goal: Point2D,
        obstacles_world: Sequence[Point2D],
        current_velocity: Velocity2D,
        obstacle_summary: ObstacleSummary,
    ) -> DwaLiteTrajectoryScore:
        if not trajectory.poses:
            return rejected("empty_trajectory")
        clearance = self.footprint_checker.trajectory_clearance(
            trajectory.poses,
            obstacles_world,
        )
        if clearance <= 0.0:
            return rejected("footprint_collision")
        if clearance < self.params.obstacle_min_dist:
            return rejected("clearance_below_min")

        end = trajectory.poses[-1]
        command = trajectory.command
        path_distances = _path_distances(trajectory.poses, path)
        if path_distances and max(path_distances) > self.params.max_path_deviation:
            return rejected("path_deviation_limit")
        path_cost = _path_distance_cost(path_distances)
        goal_cost = distance((end.x, end.y), goal)
        obstacle_cost = _obstacle_cost(clearance, self.params.obstacle_min_dist)
        heading_cost = _heading_cost(
            end,
            command,
            path,
            goal,
            self.params.heading_lookahead,
        )
        velocity_cost = abs(self.params.target_speed - command.planar_speed())
        smoothness_cost = (
            abs(command.vx - current_velocity.vx)
            + abs(command.vy - current_velocity.vy)
            + 0.5 * abs(command.wz - current_velocity.wz)
        )
        lateral_cost = self._lateral_cost(command, obstacle_summary)

        total = (
            self.params.path_weight * path_cost
            + self.params.goal_weight * goal_cost
            + self.params.obstacle_weight * obstacle_cost
            + self.params.heading_weight * heading_cost
            + self.params.velocity_weight * velocity_cost
            + self.params.smoothness_weight * smoothness_cost
            + self.params.lateral_weight * lateral_cost
        )
        return DwaLiteTrajectoryScore(
            path_cost=path_cost,
            goal_cost=goal_cost,
            obstacle_cost=obstacle_cost,
            heading_cost=heading_cost,
            velocity_cost=velocity_cost,
            smoothness_cost=smoothness_cost,
            lateral_cost=lateral_cost,
            total_cost=total,
            feasible=True,
        )

    def _lateral_cost(self, command: Velocity2D, obstacle_summary: ObstacleSummary) -> float:
        if not obstacle_summary.front_blocked:
            lateral = abs(command.vy)
            if lateral < max(0.02, self.params.vy_deadband):
                return 0.0
            forward = max(0.0, command.vx)
            planar = max(1e-6, forward + lateral)
            lateral_ratio = lateral / planar
            lateral_dominance = max(0.0, lateral - forward)
            return lateral + 1.5 * lateral * lateral_ratio + 6.0 * lateral_dominance
        preferred_sign = 1.0 if obstacle_summary.preferred_side == "left" else -1.0
        if abs(command.vy) < max(0.02, self.params.vy_deadband):
            return 2.0 + max(0.0, command.vx)
        wrong_side = command.vy * preferred_sign < 0.0
        side_penalty = 2.0 if wrong_side else 0.0
        forward_penalty = max(0.0, command.vx)
        lateral_reward = max(0.0, self.params.max_vy - abs(command.vy))
        return side_penalty + 0.5 * forward_penalty + 0.25 * lateral_reward


def rejected(reason: str) -> DwaLiteTrajectoryScore:
    return DwaLiteTrajectoryScore(
        path_cost=math.inf,
        goal_cost=math.inf,
        obstacle_cost=math.inf,
        heading_cost=math.inf,
        velocity_cost=math.inf,
        smoothness_cost=math.inf,
        lateral_cost=math.inf,
        total_cost=math.inf,
        feasible=False,
        rejection_reason=reason,
    )


def _obstacle_cost(clearance: float, obstacle_min_dist: float) -> float:
    if math.isinf(clearance):
        return 0.0
    desired_clearance = max(0.01, obstacle_min_dist)
    if clearance >= desired_clearance:
        return 0.0
    return (desired_clearance - clearance) / desired_clearance


def _path_distances(poses, path: Sequence[Point2D]) -> Sequence[float]:
    if not poses or not path:
        return []
    return [distance_to_path((pose.x, pose.y), path) for pose in poses]


def _path_distance_cost(distances: Sequence[float]) -> float:
    if not distances:
        return 0.0
    mean_distance = sum(distances) / float(len(distances))
    end_distance = distances[-1]
    max_distance = max(distances)
    return 0.35 * mean_distance + 0.45 * end_distance + 0.20 * max_distance


def _heading_cost(
    end,
    command: Velocity2D,
    path: Sequence[Point2D],
    goal: Point2D,
    lookahead_distance: float,
) -> float:
    target = lookahead_point((end.x, end.y), path, lookahead_distance) if path else goal
    if distance((end.x, end.y), target) < 1e-6:
        target = goal
    desired = math.atan2(target[1] - end.y, target[0] - end.x)
    body_heading_error = abs(normalize_angle(desired - end.yaw))
    if command.planar_speed() > 1e-4:
        world_vx = math.cos(end.yaw) * command.vx - math.sin(end.yaw) * command.vy
        world_vy = math.sin(end.yaw) * command.vx + math.cos(end.yaw) * command.vy
        motion_heading = math.atan2(world_vy, world_vx)
        motion_heading_error = abs(normalize_angle(desired - motion_heading))
        return 0.15 * motion_heading_error + 0.85 * body_heading_error
    return body_heading_error
