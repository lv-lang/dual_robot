#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Optional

from local_planner.trajectory_sampler import Velocity2D, clamp


@dataclass(frozen=True)
class CommandSmootherConfig:
    min_vx: float = 0.0
    max_vx: float = 0.35
    min_vy: float = -0.20
    max_vy: float = 0.20
    max_wz: float = 0.80
    acc_lim_x: float = 0.40
    acc_lim_y: float = 0.40
    acc_lim_theta: float = 1.00
    jerk_lim_x: float = 2.0
    jerk_lim_y: float = 2.0
    jerk_lim_theta: float = 5.0
    vy_deadband: float = 0.0
    wz_deadband: float = 0.0
    low_pass_alpha: float = 1.0
    default_dt: float = 0.1


class CommandSmoother:
    """Pure vx/vy/wz command smoother for robot1 mecanum local planning."""

    def __init__(
        self,
        config: CommandSmootherConfig = CommandSmootherConfig(),
        initial_command: Velocity2D = Velocity2D(0.0, 0.0, 0.0),
    ) -> None:
        self.config = config
        self._last_cmd = self._clamp_velocity(initial_command)
        self._last_accel = Velocity2D(0.0, 0.0, 0.0)

    @property
    def last_command(self) -> Velocity2D:
        return self._last_cmd

    @property
    def last_acceleration(self) -> Velocity2D:
        return self._last_accel

    def reset(
        self,
        command: Velocity2D = Velocity2D(0.0, 0.0, 0.0),
        acceleration: Velocity2D = Velocity2D(0.0, 0.0, 0.0),
    ) -> None:
        self._last_cmd = self._apply_deadband(self._clamp_velocity(command))
        self._last_accel = acceleration

    def smooth(self, command: Velocity2D, dt: Optional[float] = None) -> Velocity2D:
        period = self._period(dt)
        target = self._apply_deadband(self._clamp_velocity(command))

        accel_limited = Velocity2D(
            vx=self._limit_axis(
                target=target.vx,
                last=self._last_cmd.vx,
                last_accel=self._last_accel.vx,
                acc_limit=self.config.acc_lim_x,
                jerk_limit=self.config.jerk_lim_x,
                dt=period,
            ),
            vy=self._limit_axis(
                target=target.vy,
                last=self._last_cmd.vy,
                last_accel=self._last_accel.vy,
                acc_limit=self.config.acc_lim_y,
                jerk_limit=self.config.jerk_lim_y,
                dt=period,
            ),
            wz=self._limit_axis(
                target=target.wz,
                last=self._last_cmd.wz,
                last_accel=self._last_accel.wz,
                acc_limit=self.config.acc_lim_theta,
                jerk_limit=self.config.jerk_lim_theta,
                dt=period,
            ),
        )
        cmd_limited = self._apply_deadband(self._clamp_velocity(accel_limited))
        alpha = clamp(self.config.low_pass_alpha, 0.0, 1.0)
        smoothed = self._apply_deadband(
            self._clamp_velocity(
                Velocity2D(
                    vx=alpha * cmd_limited.vx + (1.0 - alpha) * self._last_cmd.vx,
                    vy=alpha * cmd_limited.vy + (1.0 - alpha) * self._last_cmd.vy,
                    wz=alpha * cmd_limited.wz + (1.0 - alpha) * self._last_cmd.wz,
                )
            )
        )
        self._last_accel = Velocity2D(
            vx=(smoothed.vx - self._last_cmd.vx) / period,
            vy=(smoothed.vy - self._last_cmd.vy) / period,
            wz=(smoothed.wz - self._last_cmd.wz) / period,
        )
        self._last_cmd = smoothed
        return smoothed

    def _limit_axis(
        self,
        target: float,
        last: float,
        last_accel: float,
        acc_limit: float,
        jerk_limit: float,
        dt: float,
    ) -> float:
        target_accel = (target - last) / dt
        target_accel = clamp(target_accel, -abs(acc_limit), abs(acc_limit))
        accel_delta = target_accel - last_accel
        limited_delta = clamp(accel_delta, -abs(jerk_limit) * dt, abs(jerk_limit) * dt)
        limited_accel = clamp(
            last_accel + limited_delta,
            -abs(acc_limit),
            abs(acc_limit),
        )
        return last + limited_accel * dt

    def _clamp_velocity(self, command: Velocity2D) -> Velocity2D:
        return Velocity2D(
            vx=clamp(_finite_or_zero(command.vx), self.config.min_vx, self.config.max_vx),
            vy=clamp(_finite_or_zero(command.vy), self.config.min_vy, self.config.max_vy),
            wz=clamp(_finite_or_zero(command.wz), -self.config.max_wz, self.config.max_wz),
        )

    def _apply_deadband(self, command: Velocity2D) -> Velocity2D:
        vy = 0.0 if abs(command.vy) < abs(self.config.vy_deadband) else command.vy
        wz = 0.0 if abs(command.wz) < abs(self.config.wz_deadband) else command.wz
        return Velocity2D(command.vx, vy, wz)

    def _period(self, dt: Optional[float]) -> float:
        period = self.config.default_dt if dt is None else dt
        if not math.isfinite(period) or period <= 0.0:
            return max(1e-6, self.config.default_dt)
        return period


def _finite_or_zero(value: float) -> float:
    return value if math.isfinite(value) else 0.0
