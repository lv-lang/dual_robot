#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence

from local_planner.trajectory_sampler import Pose2D, Velocity2D, clamp, normalize_angle


PRIMITIVE_NAMES = (
    "track_center",
    "soft_left",
    "hard_left",
    "soft_right",
    "hard_right",
    "diagonal_left",
    "diagonal_right",
    "rejoin",
    "slow_stop",
)


@dataclass(frozen=True)
class MotionPrimitiveConfig:
    max_vx: float = 0.35
    max_vy: float = 0.20
    max_wz: float = 0.80
    acc_lim_x: float = 0.40
    acc_lim_y: float = 0.40
    acc_lim_theta: float = 1.00
    sim_time: float = 1.0
    dt: float = 0.1
    control_period: float = 0.1
    track_vx: float = 0.24
    bypass_vx: float = 0.14
    soft_vy: float = 0.08
    hard_vy: float = 0.16
    rejoin_gain: float = 0.70
    heading_gain: float = 1.20
    perturb_vx: float = 0.04
    perturb_vy: float = 0.04
    perturb_wz: float = 0.12
    use_dynamic_window: bool = True


@dataclass(frozen=True)
class PrimitiveContext:
    path_heading: float = 0.0
    heading_error: float = 0.0
    cross_track_error: float = 0.0
    lateral_target_error: float = 0.0
    rejoin_direction: float = 0.0
    allow_reverse: bool = False


@dataclass(frozen=True)
class MotionPrimitive:
    name: str
    command: Velocity2D
    trajectory: List[Pose2D]


class MotionPrimitiveSampler:
    """Generates path-relative mecanum motion primitives.

    This is a pure algorithm class. It has no ROS2 imports, publishers,
    subscriptions, or topic names.
    """

    def __init__(self, config: MotionPrimitiveConfig = MotionPrimitiveConfig()) -> None:
        self.config = config

    def sample(
        self,
        start: Pose2D,
        current_velocity: Velocity2D,
        context: PrimitiveContext = PrimitiveContext(),
    ) -> List[MotionPrimitive]:
        primitives: List[MotionPrimitive] = []
        for name in PRIMITIVE_NAMES:
            reference = self.reference_velocity(name, context)
            for command in self._perturb(reference):
                bounded = self._bound_command(command, current_velocity, context.allow_reverse)
                trajectory = self.rollout(start, bounded)
                primitives.append(MotionPrimitive(name=name, command=bounded, trajectory=trajectory))
        return _deduplicate(primitives)

    def reference_velocity(
        self,
        name: str,
        context: PrimitiveContext = PrimitiveContext(),
    ) -> Velocity2D:
        cfg = self.config
        heading_wz = clamp(cfg.heading_gain * context.heading_error, -cfg.max_wz, cfg.max_wz)
        rejoin_vy = clamp(cfg.rejoin_gain * context.rejoin_direction, -cfg.max_vy, cfg.max_vy)
        target_vy = clamp(cfg.rejoin_gain * context.lateral_target_error, -cfg.max_vy, cfg.max_vy)

        if name == "track_center":
            return Velocity2D(cfg.track_vx, rejoin_vy, heading_wz)
        if name == "soft_left":
            return Velocity2D(cfg.bypass_vx, cfg.soft_vy, 0.5 * heading_wz)
        if name == "hard_left":
            return Velocity2D(0.5 * cfg.bypass_vx, cfg.hard_vy, 0.35 * heading_wz)
        if name == "soft_right":
            return Velocity2D(cfg.bypass_vx, -cfg.soft_vy, 0.5 * heading_wz)
        if name == "hard_right":
            return Velocity2D(0.5 * cfg.bypass_vx, -cfg.hard_vy, 0.35 * heading_wz)
        if name == "diagonal_left":
            return Velocity2D(cfg.track_vx, cfg.soft_vy, 0.5 * heading_wz)
        if name == "diagonal_right":
            return Velocity2D(cfg.track_vx, -cfg.soft_vy, 0.5 * heading_wz)
        if name == "rejoin":
            return Velocity2D(cfg.bypass_vx, target_vy or rejoin_vy, heading_wz)
        if name == "slow_stop":
            return Velocity2D(0.0, 0.0, 0.0)
        raise ValueError(f"unknown motion primitive: {name}")

    def rollout(self, start: Pose2D, velocity: Velocity2D) -> List[Pose2D]:
        dt = max(0.02, self.config.dt)
        steps = max(1, int(math.ceil(max(dt, self.config.sim_time) / dt)))
        pose = start
        trajectory = [pose]
        for _ in range(steps):
            cos_yaw = math.cos(pose.yaw)
            sin_yaw = math.sin(pose.yaw)
            x_dot = cos_yaw * velocity.vx - sin_yaw * velocity.vy
            y_dot = sin_yaw * velocity.vx + cos_yaw * velocity.vy
            pose = Pose2D(
                x=pose.x + x_dot * dt,
                y=pose.y + y_dot * dt,
                yaw=normalize_angle(pose.yaw + velocity.wz * dt),
            )
            trajectory.append(pose)
        return trajectory

    def _perturb(self, reference: Velocity2D) -> Iterable[Velocity2D]:
        vx_values = (reference.vx - self.config.perturb_vx, reference.vx, reference.vx + self.config.perturb_vx)
        vy_values = (reference.vy - self.config.perturb_vy, reference.vy, reference.vy + self.config.perturb_vy)
        wz_values = (reference.wz - self.config.perturb_wz, reference.wz, reference.wz + self.config.perturb_wz)
        for vx in vx_values:
            for vy in vy_values:
                for wz in wz_values:
                    yield Velocity2D(vx=vx, vy=vy, wz=wz)

    def _bound_command(
        self,
        command: Velocity2D,
        current_velocity: Velocity2D,
        allow_reverse: bool,
    ) -> Velocity2D:
        min_vx = -self.config.max_vx if allow_reverse else 0.0
        vx = clamp(command.vx, min_vx, self.config.max_vx)
        vy = clamp(command.vy, -self.config.max_vy, self.config.max_vy)
        wz = clamp(command.wz, -self.config.max_wz, self.config.max_wz)

        if self.config.use_dynamic_window:
            period = max(0.0, self.config.control_period)
            vx = clamp(vx, current_velocity.vx - self.config.acc_lim_x * period, current_velocity.vx + self.config.acc_lim_x * period)
            vy = clamp(vy, current_velocity.vy - self.config.acc_lim_y * period, current_velocity.vy + self.config.acc_lim_y * period)
            wz = clamp(wz, current_velocity.wz - self.config.acc_lim_theta * period, current_velocity.wz + self.config.acc_lim_theta * period)
            vx = clamp(vx, min_vx, self.config.max_vx)
            vy = clamp(vy, -self.config.max_vy, self.config.max_vy)
            wz = clamp(wz, -self.config.max_wz, self.config.max_wz)
        return Velocity2D(round(vx, 6), round(vy, 6), round(wz, 6))


def _deduplicate(primitives: Sequence[MotionPrimitive]) -> List[MotionPrimitive]:
    seen = set()
    unique: List[MotionPrimitive] = []
    for primitive in primitives:
        key = (
            primitive.name,
            primitive.command.vx,
            primitive.command.vy,
            primitive.command.wz,
        )
        if key in seen:
            continue
        seen.add(key)
        unique.append(primitive)
    return unique
