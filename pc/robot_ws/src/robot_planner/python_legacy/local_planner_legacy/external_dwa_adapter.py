#!/usr/bin/env python3

"""Internal DWA algorithm adapter.

This module adapts the algorithmic shape of the external amslabtech
``dwa_planner`` package into a pure Python planner core. It deliberately owns
no ROS2 publishers, subscriptions, nodes, launch files, or topic names.
"""

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class ExternalDwaConfig:
    min_vx: float
    max_vx: float
    max_wz: float
    acc_lim_x: float
    acc_lim_theta: float
    vx_samples: int
    wz_samples: int
    sim_time: float
    dt: float
    robot_radius: float
    obstacle_margin: float
    obstacle_range: float
    goal_tolerance: float
    target_velocity: float
    angle_resolution: float
    angle_to_goal_turn_threshold: float
    min_in_place_wz: float
    max_in_place_wz: float
    weight_path_distance: float
    weight_target_distance: float
    weight_heading: float
    weight_obstacle: float
    weight_smoothness: float
    weight_speed: float
    enable_vy: bool


@dataclass(frozen=True)
class ExternalDwaPose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ExternalDwaVelocity:
    vx: float
    vy: float
    wz: float


@dataclass(frozen=True)
class ExternalDwaDebugCost:
    path_distance: float
    target_distance: float
    heading_error: float
    obstacle_cost: float
    smoothness: float
    speed_reward: float
    total: float


@dataclass(frozen=True)
class ExternalDwaResult:
    velocity: ExternalDwaVelocity
    trajectory: List[ExternalDwaPose]
    debug_cost: Optional[ExternalDwaDebugCost]
    valid: bool
    reason: str


@dataclass(frozen=True)
class _Candidate:
    velocity: ExternalDwaVelocity
    trajectory: List[ExternalDwaPose]
    cost: ExternalDwaDebugCost


class ExternalDwaCore:
    """Differential-drive DWA core adapted for the robot1 ROS2 adapter."""

    def __init__(self, config: ExternalDwaConfig) -> None:
        self.config = config

    def plan(
        self,
        pose: ExternalDwaPose,
        current_velocity: ExternalDwaVelocity,
        target: Point2D,
        path_points: Sequence[Point2D],
        obstacles_robot_frame: Sequence[Point2D],
    ) -> ExternalDwaResult:
        if not path_points:
            return self._stop("empty_path", pose)
        if not math.isfinite(pose.x) or not math.isfinite(pose.y):
            return self._stop("invalid_pose", pose)

        if _distance((pose.x, pose.y), path_points[-1]) <= self.config.goal_tolerance:
            return self._stop("goal_reached", pose, valid=True)

        obstacles_world = [
            _body_to_world(pose, obstacle)
            for obstacle in obstacles_robot_frame
            if _distance((0.0, 0.0), obstacle) <= self.config.obstacle_range
        ]

        turn_result = self._in_place_turn_if_needed(
            pose,
            target,
            obstacles_world,
        )
        if turn_result is not None:
            return turn_result

        candidates: List[_Candidate] = []
        for velocity in self._sample_velocity_space(current_velocity):
            trajectory = self._rollout(pose, velocity)
            cost = self._score_trajectory(
                trajectory,
                velocity,
                current_velocity,
                target,
                path_points,
                obstacles_world,
            )
            if math.isfinite(cost.total):
                candidates.append(_Candidate(velocity, trajectory, cost))

        if not candidates:
            return self._stop("no_valid_external_dwa_trajectory", pose)

        best = min(candidates, key=lambda candidate: candidate.cost.total)
        return ExternalDwaResult(
            velocity=best.velocity,
            trajectory=best.trajectory,
            debug_cost=best.cost,
            valid=True,
            reason="ok",
        )

    def _in_place_turn_if_needed(
        self,
        pose: ExternalDwaPose,
        target: Point2D,
        obstacles_world: Sequence[Point2D],
    ) -> Optional[ExternalDwaResult]:
        target_heading = math.atan2(target[1] - pose.y, target[0] - pose.x)
        heading_error = _normalize_angle(target_heading - pose.yaw)
        if abs(heading_error) <= self.config.angle_to_goal_turn_threshold:
            return None

        wz = _clamp(
            heading_error,
            -self.config.max_in_place_wz,
            self.config.max_in_place_wz,
        )
        if 0.0 < abs(wz) < self.config.min_in_place_wz:
            wz = math.copysign(self.config.min_in_place_wz, wz)
        velocity = ExternalDwaVelocity(vx=0.0, vy=0.0, wz=wz)
        trajectory = self._rollout(pose, velocity)
        clearance = self._trajectory_clearance(trajectory, obstacles_world)
        if clearance <= 0.0:
            return None
        cost = ExternalDwaDebugCost(
            path_distance=0.0,
            target_distance=_distance((pose.x, pose.y), target),
            heading_error=abs(heading_error),
            obstacle_cost=0.0 if math.isinf(clearance) else 1.0 / max(0.01, clearance),
            smoothness=abs(wz),
            speed_reward=0.0,
            total=abs(heading_error),
        )
        return ExternalDwaResult(
            velocity=velocity,
            trajectory=trajectory,
            debug_cost=cost,
            valid=True,
            reason="turn_in_place",
        )

    def _sample_velocity_space(
        self, current_velocity: ExternalDwaVelocity
    ) -> Iterable[ExternalDwaVelocity]:
        period = max(0.02, self.config.dt)
        current_vx = _clamp(current_velocity.vx, self.config.min_vx, self.config.max_vx)
        current_wz = _clamp(current_velocity.wz, -self.config.max_wz, self.config.max_wz)

        vx_lower = max(self.config.min_vx, current_vx - self.config.acc_lim_x * period)
        vx_upper = min(
            min(self.config.max_vx, self.config.target_velocity),
            current_vx + self.config.acc_lim_x * period,
        )
        wz_lower = max(-self.config.max_wz, current_wz - self.config.acc_lim_theta * period)
        wz_upper = min(self.config.max_wz, current_wz + self.config.acc_lim_theta * period)

        vx_values = _sample_axis(vx_lower, vx_upper, self.config.vx_samples, include_zero=True)
        wz_values = _sample_axis(wz_lower, wz_upper, self.config.wz_samples, include_zero=True)
        for vx in vx_values:
            for wz in wz_values:
                yield ExternalDwaVelocity(vx=vx, vy=0.0, wz=wz)

    def _rollout(
        self, pose: ExternalDwaPose, velocity: ExternalDwaVelocity
    ) -> List[ExternalDwaPose]:
        trajectory = [pose]
        state = pose
        steps = max(1, int(math.ceil(self.config.sim_time / max(0.02, self.config.dt))))
        for _ in range(steps):
            yaw = _normalize_angle(state.yaw + velocity.wz * self.config.dt)
            state = ExternalDwaPose(
                x=state.x + velocity.vx * math.cos(yaw) * self.config.dt,
                y=state.y + velocity.vx * math.sin(yaw) * self.config.dt,
                yaw=yaw,
            )
            trajectory.append(state)
        return trajectory

    def _score_trajectory(
        self,
        trajectory: Sequence[ExternalDwaPose],
        velocity: ExternalDwaVelocity,
        current_velocity: ExternalDwaVelocity,
        target: Point2D,
        path_points: Sequence[Point2D],
        obstacles_world: Sequence[Point2D],
    ) -> ExternalDwaDebugCost:
        end = trajectory[-1]
        clearance = self._trajectory_clearance(trajectory, obstacles_world)
        if clearance <= 0.0:
            return ExternalDwaDebugCost(
                path_distance=math.inf,
                target_distance=math.inf,
                heading_error=math.inf,
                obstacle_cost=math.inf,
                smoothness=math.inf,
                speed_reward=0.0,
                total=math.inf,
            )

        path_distance = min(
            _distance((end.x, end.y), path_point)
            for path_point in path_points
        )
        target_distance = _distance((end.x, end.y), target)
        target_heading = math.atan2(target[1] - end.y, target[0] - end.x)
        heading_error = abs(_normalize_angle(target_heading - end.yaw))
        obstacle_cost = 0.0 if math.isinf(clearance) else 1.0 / max(0.01, clearance)
        smoothness = abs(velocity.vx - current_velocity.vx) + 0.5 * abs(
            velocity.wz - current_velocity.wz
        )
        speed_reward = velocity.vx / max(0.01, self.config.max_vx)

        total = (
            self.config.weight_path_distance * path_distance
            + self.config.weight_target_distance * target_distance
            + self.config.weight_heading * heading_error
            + self.config.weight_obstacle * obstacle_cost
            + self.config.weight_smoothness * smoothness
            - self.config.weight_speed * speed_reward
        )
        return ExternalDwaDebugCost(
            path_distance=path_distance,
            target_distance=target_distance,
            heading_error=heading_error,
            obstacle_cost=obstacle_cost,
            smoothness=smoothness,
            speed_reward=speed_reward,
            total=total,
        )

    def _trajectory_clearance(
        self,
        trajectory: Sequence[ExternalDwaPose],
        obstacles_world: Sequence[Point2D],
    ) -> float:
        if not obstacles_world:
            return math.inf

        radius = self.config.robot_radius + self.config.obstacle_margin
        min_clearance = math.inf
        for state in trajectory[1:]:
            for obstacle in obstacles_world:
                clearance = _distance((state.x, state.y), obstacle) - radius
                min_clearance = min(min_clearance, clearance)
        return min_clearance

    @staticmethod
    def _stop(
        reason: str,
        pose: ExternalDwaPose,
        valid: bool = False,
    ) -> ExternalDwaResult:
        return ExternalDwaResult(
            velocity=ExternalDwaVelocity(0.0, 0.0, 0.0),
            trajectory=[pose],
            debug_cost=None,
            valid=valid,
            reason=reason,
        )


def scan_points_from_ranges(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    angle_resolution: float,
    obstacle_range: float,
) -> List[Point2D]:
    if angle_increment == 0.0:
        return []

    step = max(1, int(round(max(angle_resolution, abs(angle_increment)) / abs(angle_increment))))
    points: List[Point2D] = []
    max_range = obstacle_range
    if math.isfinite(range_max) and range_max > 0.0:
        max_range = min(max_range, range_max)

    for index in range(0, len(ranges), step):
        measured = ranges[index]
        if not math.isfinite(measured):
            continue
        if measured < max(0.0, range_min) or measured > max_range:
            continue
        angle = angle_min + angle_increment * index
        points.append((measured * math.cos(angle), measured * math.sin(angle)))
    return points


def _sample_axis(lower: float, upper: float, count: int, include_zero: bool) -> List[float]:
    if upper < lower:
        lower, upper = upper, lower
    values = {lower, upper}
    if include_zero and lower <= 0.0 <= upper:
        values.add(0.0)
    if count > 1 and upper > lower:
        step = (upper - lower) / float(count - 1)
        for index in range(count):
            values.add(lower + step * index)
    return sorted(round(value, 4) for value in values)


def _body_to_world(pose: ExternalDwaPose, point: Point2D) -> Point2D:
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    return (
        pose.x + cos_yaw * point[0] - sin_yaw * point[1],
        pose.y + sin_yaw * point[0] + cos_yaw * point[1],
    )


def _distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
