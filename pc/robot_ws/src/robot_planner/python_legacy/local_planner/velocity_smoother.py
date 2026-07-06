#!/usr/bin/env python3

from dataclasses import dataclass

from local_planner.dwa_lite_params import DwaLiteParams
from local_planner.planner_utils import Velocity2D, clamp, zero_velocity


@dataclass(frozen=True)
class Acceleration2D:
    ax: float
    ay: float
    aw: float


class DwaLiteVelocitySmoother:
    """Stateful command smoother with velocity, acceleration, jerk and low-pass limits."""

    def __init__(self, params: DwaLiteParams) -> None:
        self.params = params
        self.last_cmd = zero_velocity()
        self.last_accel = Acceleration2D(0.0, 0.0, 0.0)

    def reset(self) -> None:
        self.last_cmd = zero_velocity()
        self.last_accel = Acceleration2D(0.0, 0.0, 0.0)

    def smooth(self, target: Velocity2D) -> Velocity2D:
        limited = self._limit_velocity(target)
        dt = max(1e-6, self.params.control_period)
        ax = self._limit_axis_accel(
            (limited.vx - self.last_cmd.vx) / dt,
            self.last_accel.ax,
            self.params.acc_lim_x,
            self.params.jerk_lim_x,
            dt,
        )
        ay = self._limit_axis_accel(
            (limited.vy - self.last_cmd.vy) / dt,
            self.last_accel.ay,
            self.params.acc_lim_y,
            self.params.jerk_lim_y,
            dt,
        )
        aw = self._limit_axis_accel(
            (limited.wz - self.last_cmd.wz) / dt,
            self.last_accel.aw,
            self.params.acc_lim_theta,
            self.params.jerk_lim_theta,
            dt,
        )
        cmd_limited = Velocity2D(
            vx=self.last_cmd.vx + ax * dt,
            vy=self.last_cmd.vy + ay * dt,
            wz=self.last_cmd.wz + aw * dt,
        )
        alpha = clamp(self.params.cmd_filter_alpha, 0.0, 1.0)
        filtered = Velocity2D(
            vx=alpha * cmd_limited.vx + (1.0 - alpha) * self.last_cmd.vx,
            vy=alpha * cmd_limited.vy + (1.0 - alpha) * self.last_cmd.vy,
            wz=alpha * cmd_limited.wz + (1.0 - alpha) * self.last_cmd.wz,
        )
        deadbanded = Velocity2D(
            vx=self._apply_deadband(filtered.vx, limited.vx, self.params.vx_deadband),
            vy=self._apply_deadband(filtered.vy, limited.vy, self.params.vy_deadband),
            wz=self._apply_deadband(filtered.wz, limited.wz, self.params.wz_deadband),
        )
        self.last_accel = Acceleration2D(
            ax=(deadbanded.vx - self.last_cmd.vx) / dt,
            ay=(deadbanded.vy - self.last_cmd.vy) / dt,
            aw=(deadbanded.wz - self.last_cmd.wz) / dt,
        )
        self.last_cmd = self._limit_velocity(deadbanded)
        return self.last_cmd

    def _limit_velocity(self, target: Velocity2D) -> Velocity2D:
        return Velocity2D(
            vx=clamp(target.vx, self.params.min_vx, self.params.max_vx),
            vy=clamp(target.vy, self.params.min_vy, self.params.max_vy),
            wz=clamp(target.wz, self.params.min_wz, self.params.max_wz),
        )

    @staticmethod
    def _limit_axis_accel(
        desired_accel: float,
        previous_accel: float,
        accel_limit: float,
        jerk_limit: float,
        dt: float,
    ) -> float:
        accel = clamp(desired_accel, -abs(accel_limit), abs(accel_limit))
        max_delta = abs(jerk_limit) * dt
        return clamp(accel, previous_accel - max_delta, previous_accel + max_delta)

    @staticmethod
    def _apply_deadband(value: float, target: float, deadband: float) -> float:
        if abs(target) < deadband and abs(value) < deadband:
            return 0.0
        return value
