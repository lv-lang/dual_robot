#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from local_planner.footprint_collision_checker import FootprintCollisionChecker
from local_planner.motion_primitive_sampler import MotionPrimitive
from local_planner.trajectory_sampler import Pose2D, Velocity2D, normalize_angle


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class ScoringWeights:
    progress: float = 2.0
    path_distance: float = 2.5
    lateral_target: float = 1.2
    obstacle_clearance: float = 1.0
    heading: float = 0.8
    smoothness: float = 0.6
    jerk: float = 0.5
    side_switch: float = 0.8
    unnecessary_rotation: float = 0.7
    goal: float = 2.5


@dataclass(frozen=True)
class HardConstraintConfig:
    max_vx: float = 0.35
    max_vy: float = 0.20
    max_wz: float = 0.80
    max_acc_x: float = 0.40
    max_acc_y: float = 0.40
    max_acc_theta: float = 1.00
    max_jerk_x: float = 2.0
    max_jerk_y: float = 2.0
    max_jerk_theta: float = 4.0
    max_path_deviation: float = 0.80
    min_side_clearance: float = 0.04
    stop_zone_distance: float = 0.35
    forward_stop_speed: float = 0.08
    control_period: float = 0.1


@dataclass(frozen=True)
class ScoringContext:
    current_velocity: Velocity2D
    previous_command: Velocity2D = Velocity2D(0.0, 0.0, 0.0)
    previous_acceleration: Velocity2D = Velocity2D(0.0, 0.0, 0.0)
    previous_side: str = "none"
    preferred_side: str = "none"
    lateral_target_offset: float = 0.0
    path_heading: Optional[float] = None
    front_blocked: bool = False
    allow_rotation: bool = True


@dataclass(frozen=True)
class TrajectoryScore:
    progress_cost: float
    path_distance_cost: float
    lateral_target_cost: float
    obstacle_clearance_cost: float
    heading_cost: float
    smoothness_cost: float
    jerk_cost: float
    side_switch_cost: float
    unnecessary_rotation_cost: float
    goal_cost: float
    total_cost: float
    feasible: bool
    rejection_reason: str = ""


class TrajectoryScorer:
    """Applies hard constraints, then scores mecanum primitives."""

    def __init__(
        self,
        weights: ScoringWeights,
        hard_constraints: HardConstraintConfig,
        collision_checker: FootprintCollisionChecker,
    ) -> None:
        self.weights = weights
        self.hard_constraints = hard_constraints
        self.collision_checker = collision_checker

    def score(
        self,
        primitive: MotionPrimitive,
        path_points: Sequence[Point2D],
        goal: Point2D,
        obstacles_world: Sequence[Point2D],
        context: ScoringContext,
    ) -> TrajectoryScore:
        rejection = self._hard_constraint_rejection(
            primitive,
            path_points,
            obstacles_world,
            context,
        )
        if rejection:
            return infinite_score(rejection)

        trajectory = primitive.trajectory
        start = trajectory[0]
        end = trajectory[-1]
        clearance = self.collision_checker.trajectory_clearance(trajectory, obstacles_world)
        progress_cost = _progress_cost(start, end, goal)
        path_cost = _trajectory_path_distance(trajectory, path_points)
        lateral_cost = abs(_signed_lateral_error(end, path_points) - context.lateral_target_offset)
        obstacle_cost = 0.0 if math.isinf(clearance) else 1.0 / max(0.01, clearance)
        heading_cost = _heading_cost(end, goal, context.path_heading)
        smoothness_cost = _velocity_delta(primitive.command, context.current_velocity)
        jerk_cost = _jerk_cost(primitive.command, context, self.hard_constraints.control_period)
        side_switch_cost = _side_switch_cost(primitive.name, context.previous_side)
        rotation_cost = _unnecessary_rotation_cost(primitive.command, context)
        goal_cost = _distance((end.x, end.y), goal)

        total = (
            self.weights.progress * progress_cost
            + self.weights.path_distance * path_cost
            + self.weights.lateral_target * lateral_cost
            + self.weights.obstacle_clearance * obstacle_cost
            + self.weights.heading * heading_cost
            + self.weights.smoothness * smoothness_cost
            + self.weights.jerk * jerk_cost
            + self.weights.side_switch * side_switch_cost
            + self.weights.unnecessary_rotation * rotation_cost
            + self.weights.goal * goal_cost
        )
        return TrajectoryScore(
            progress_cost=progress_cost,
            path_distance_cost=path_cost,
            lateral_target_cost=lateral_cost,
            obstacle_clearance_cost=obstacle_cost,
            heading_cost=heading_cost,
            smoothness_cost=smoothness_cost,
            jerk_cost=jerk_cost,
            side_switch_cost=side_switch_cost,
            unnecessary_rotation_cost=rotation_cost,
            goal_cost=goal_cost,
            total_cost=total,
            feasible=True,
        )

    def _hard_constraint_rejection(
        self,
        primitive: MotionPrimitive,
        path_points: Sequence[Point2D],
        obstacles_world: Sequence[Point2D],
        context: ScoringContext,
    ) -> str:
        command = primitive.command
        cfg = self.hard_constraints
        if not primitive.trajectory:
            return "empty_trajectory"
        if abs(command.vx) > cfg.max_vx or abs(command.vy) > cfg.max_vy or abs(command.wz) > cfg.max_wz:
            return "velocity_limit"
        if _axis_accel(command.vx, context.current_velocity.vx, cfg.control_period) > cfg.max_acc_x:
            return "acceleration_limit"
        if _axis_accel(command.vy, context.current_velocity.vy, cfg.control_period) > cfg.max_acc_y:
            return "acceleration_limit"
        if _axis_accel(command.wz, context.current_velocity.wz, cfg.control_period) > cfg.max_acc_theta:
            return "acceleration_limit"
        if _jerk_axis(command.vx, context.current_velocity.vx, context.previous_acceleration.vx, cfg.control_period) > cfg.max_jerk_x:
            return "jerk_limit"
        if _jerk_axis(command.vy, context.current_velocity.vy, context.previous_acceleration.vy, cfg.control_period) > cfg.max_jerk_y:
            return "jerk_limit"
        if _jerk_axis(command.wz, context.current_velocity.wz, context.previous_acceleration.wz, cfg.control_period) > cfg.max_jerk_theta:
            return "jerk_limit"
        if self.collision_checker.trajectory_collides(primitive.trajectory, obstacles_world):
            return "footprint_collision"
        if _trajectory_path_distance(primitive.trajectory, path_points) > cfg.max_path_deviation:
            return "path_deviation"
        if context.front_blocked and command.vx > cfg.forward_stop_speed:
            front_clearance = self.collision_checker.front_clearance(primitive.trajectory[0], obstacles_world)
            if front_clearance <= cfg.stop_zone_distance:
                return "front_stop_zone"
        if command.vy > 1e-6 and self.collision_checker.side_clearance(primitive.trajectory[0], obstacles_world, "left") < cfg.min_side_clearance:
            return "side_clearance"
        if command.vy < -1e-6 and self.collision_checker.side_clearance(primitive.trajectory[0], obstacles_world, "right") < cfg.min_side_clearance:
            return "side_clearance"
        return ""


def infinite_score(reason: str) -> TrajectoryScore:
    return TrajectoryScore(
        progress_cost=math.inf,
        path_distance_cost=math.inf,
        lateral_target_cost=math.inf,
        obstacle_clearance_cost=math.inf,
        heading_cost=math.inf,
        smoothness_cost=math.inf,
        jerk_cost=math.inf,
        side_switch_cost=math.inf,
        unnecessary_rotation_cost=math.inf,
        goal_cost=math.inf,
        total_cost=math.inf,
        feasible=False,
        rejection_reason=reason,
    )


def _progress_cost(start: Pose2D, end: Pose2D, goal: Point2D) -> float:
    start_distance = _distance((start.x, start.y), goal)
    end_distance = _distance((end.x, end.y), goal)
    return max(0.0, start_distance - end_distance) * -1.0 + max(0.0, end_distance - start_distance)


def _trajectory_path_distance(trajectory: Sequence[Pose2D], path_points: Sequence[Point2D]) -> float:
    if not trajectory or not path_points:
        return 0.0
    return max(_distance_to_path((pose.x, pose.y), path_points) for pose in trajectory)


def _distance_to_path(point: Point2D, path_points: Sequence[Point2D]) -> float:
    return min(_distance(point, path_point) for path_point in path_points) if path_points else 0.0


def _signed_lateral_error(pose: Pose2D, path_points: Sequence[Point2D]) -> float:
    if len(path_points) < 2:
        return 0.0
    start = path_points[0]
    end = path_points[-1]
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return 0.0
    return ((pose.x - start[0]) * -dy + (pose.y - start[1]) * dx) / length


def _heading_cost(end: Pose2D, goal: Point2D, path_heading: Optional[float]) -> float:
    target = path_heading
    if target is None:
        target = math.atan2(goal[1] - end.y, goal[0] - end.x)
    return abs(normalize_angle(target - end.yaw))


def _velocity_delta(a: Velocity2D, b: Velocity2D) -> float:
    return abs(a.vx - b.vx) + abs(a.vy - b.vy) + 0.5 * abs(a.wz - b.wz)


def _jerk_cost(command: Velocity2D, context: ScoringContext, control_period: float) -> float:
    period = max(1e-6, control_period)
    ax = (command.vx - context.current_velocity.vx) / period
    ay = (command.vy - context.current_velocity.vy) / period
    aw = (command.wz - context.current_velocity.wz) / period
    return (
        abs(ax - context.previous_acceleration.vx)
        + abs(ay - context.previous_acceleration.vy)
        + 0.5 * abs(aw - context.previous_acceleration.wz)
    )


def _side_switch_cost(name: str, previous_side: str) -> float:
    side = _primitive_side(name)
    if side == "none" or previous_side in ("", "none", side):
        return 0.0
    return 1.0


def _primitive_side(name: str) -> str:
    if "left" in name:
        return "left"
    if "right" in name:
        return "right"
    return "none"


def _unnecessary_rotation_cost(command: Velocity2D, context: ScoringContext) -> float:
    if context.allow_rotation:
        return 0.0
    if abs(command.vy) > 1e-6:
        return 0.25 * abs(command.wz)
    return abs(command.wz)


def _axis_accel(value: float, previous: float, period: float) -> float:
    return abs(value - previous) / max(1e-6, period)


def _jerk_axis(value: float, velocity: float, previous_accel: float, period: float) -> float:
    accel = (value - velocity) / max(1e-6, period)
    return abs(accel - previous_accel) / max(1e-6, period)


def _distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
