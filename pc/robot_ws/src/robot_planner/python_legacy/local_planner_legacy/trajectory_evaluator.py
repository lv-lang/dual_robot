#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

from local_planner.footprint_collision_checker import FootprintCollisionChecker
from local_planner.trajectory_sampler import Pose2D, Velocity2D, normalize_angle


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class CostWeights:
    path: float = 2.0
    goal: float = 3.0
    obstacle: float = 0.8
    heading: float = 0.8
    velocity: float = 0.15
    smoothness: float = 0.4
    lateral: float = 0.2
    oscillation: float = 0.8


@dataclass(frozen=True)
class TrajectoryCost:
    path_cost: float
    goal_cost: float
    obstacle_cost: float
    heading_cost: float
    velocity_cost: float
    smoothness_cost: float
    lateral_cost: float
    oscillation_cost: float
    total_cost: float


class TrajectoryEvaluator:
    def __init__(
        self,
        weights: CostWeights,
        collision_checker: FootprintCollisionChecker,
        target_velocity: float,
    ) -> None:
        self.weights = weights
        self.collision_checker = collision_checker
        self.target_velocity = max(0.0, target_velocity)

    def evaluate(
        self,
        trajectory: Sequence[Pose2D],
        velocity: Velocity2D,
        current_velocity: Velocity2D,
        previous_command: Velocity2D,
        path_points: Sequence[Point2D],
        goal: Point2D,
        obstacles_world: Sequence[Point2D],
    ) -> TrajectoryCost:
        if not trajectory:
            return _infinite_cost()

        clearance = self.collision_checker.trajectory_clearance(
            trajectory,
            obstacles_world,
        )
        if clearance <= 0.0:
            return _infinite_cost()

        end = trajectory[-1]
        path_cost = _distance_to_path((end.x, end.y), path_points)
        goal_cost = _distance((end.x, end.y), goal)
        target_heading = math.atan2(goal[1] - end.y, goal[0] - end.x)
        heading_cost = abs(normalize_angle(target_heading - end.yaw))
        obstacle_cost = 0.0 if math.isinf(clearance) else 1.0 / max(0.01, clearance)
        speed = math.hypot(velocity.vx, velocity.vy)
        velocity_cost = max(0.0, self.target_velocity - speed)
        smoothness_cost = (
            abs(velocity.vx - current_velocity.vx)
            + abs(velocity.vy - current_velocity.vy)
            + 0.5 * abs(velocity.wz - current_velocity.wz)
        )
        lateral_cost = abs(velocity.vy)
        oscillation_cost = _oscillation_cost(velocity, previous_command)

        total = (
            self.weights.path * path_cost
            + self.weights.goal * goal_cost
            + self.weights.obstacle * obstacle_cost
            + self.weights.heading * heading_cost
            + self.weights.velocity * velocity_cost
            + self.weights.smoothness * smoothness_cost
            + self.weights.lateral * lateral_cost
            + self.weights.oscillation * oscillation_cost
        )
        return TrajectoryCost(
            path_cost=path_cost,
            goal_cost=goal_cost,
            obstacle_cost=obstacle_cost,
            heading_cost=heading_cost,
            velocity_cost=velocity_cost,
            smoothness_cost=smoothness_cost,
            lateral_cost=lateral_cost,
            oscillation_cost=oscillation_cost,
            total_cost=total,
        )


def _distance_to_path(point: Point2D, path_points: Sequence[Point2D]) -> float:
    if not path_points:
        return 0.0
    return min(_distance(point, path_point) for path_point in path_points)


def _oscillation_cost(velocity: Velocity2D, previous_command: Velocity2D) -> float:
    cost = 0.0
    cost += _sign_flip_cost(velocity.vx, previous_command.vx)
    cost += _sign_flip_cost(velocity.vy, previous_command.vy)
    cost += 0.5 * _sign_flip_cost(velocity.wz, previous_command.wz)
    return cost


def _sign_flip_cost(value: float, previous: float) -> float:
    if abs(value) < 1e-6 or abs(previous) < 1e-6:
        return 0.0
    return 1.0 if value * previous < 0.0 else 0.0


def _distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _infinite_cost() -> TrajectoryCost:
    return TrajectoryCost(
        path_cost=math.inf,
        goal_cost=math.inf,
        obstacle_cost=math.inf,
        heading_cost=math.inf,
        velocity_cost=math.inf,
        smoothness_cost=math.inf,
        lateral_cost=math.inf,
        oscillation_cost=math.inf,
        total_cost=math.inf,
    )
