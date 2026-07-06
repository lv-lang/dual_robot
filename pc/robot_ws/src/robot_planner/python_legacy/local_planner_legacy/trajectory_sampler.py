#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Iterable, List


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Velocity2D:
    vx: float
    vy: float
    wz: float


@dataclass(frozen=True)
class SamplingConfig:
    min_vx: float = 0.0
    max_vx: float = 0.35
    min_vy: float = -0.20
    max_vy: float = 0.20
    max_wz: float = 0.80
    acc_lim_x: float = 0.40
    acc_lim_y: float = 0.40
    acc_lim_theta: float = 1.00
    vx_samples: int = 5
    vy_samples: int = 5
    wz_samples: int = 7
    sim_time: float = 1.5
    dt: float = 0.1
    control_period: float = 0.1
    use_dynamic_window: bool = True


class TrajectorySampler:
    """Samples mecanum vx/vy/wz commands and rolls them forward."""

    def __init__(self, config: SamplingConfig) -> None:
        self.config = config

    def sample(self, current_velocity: Velocity2D) -> Iterable[Velocity2D]:
        vx_values = _sample_axis(
            self.config.min_vx,
            self.config.max_vx,
            current_velocity.vx,
            self.config.acc_lim_x,
            self.config.vx_samples,
            self.config.control_period,
            self.config.use_dynamic_window,
        )
        vy_values = _sample_axis(
            self.config.min_vy,
            self.config.max_vy,
            current_velocity.vy,
            self.config.acc_lim_y,
            self.config.vy_samples,
            self.config.control_period,
            self.config.use_dynamic_window,
        )
        wz_values = _sample_axis(
            -self.config.max_wz,
            self.config.max_wz,
            current_velocity.wz,
            self.config.acc_lim_theta,
            self.config.wz_samples,
            self.config.control_period,
            self.config.use_dynamic_window,
        )

        for vx in vx_values:
            for vy in vy_values:
                for wz in wz_values:
                    yield Velocity2D(vx=vx, vy=vy, wz=wz)

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


def _sample_axis(
    absolute_min: float,
    absolute_max: float,
    current: float,
    acc_limit: float,
    count: int,
    control_period: float,
    use_dynamic_window: bool,
) -> List[float]:
    lower = min(absolute_min, absolute_max)
    upper = max(absolute_min, absolute_max)
    if use_dynamic_window:
        current = clamp(current, lower, upper)
        window = max(0.0, acc_limit) * max(0.0, control_period)
        lower = max(lower, current - window)
        upper = min(upper, current + window)

    values = {lower, upper}
    if lower <= 0.0 <= upper:
        values.add(0.0)
    if count > 1 and upper > lower:
        step = (upper - lower) / float(count - 1)
        for index in range(count):
            values.add(lower + step * index)
    return sorted(round(value, 6) for value in values)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
