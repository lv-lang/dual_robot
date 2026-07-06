#!/usr/bin/env python3

from dataclasses import dataclass


@dataclass(frozen=True)
class DwaLiteParams:
    min_vx: float = 0.0
    max_vx: float = 0.40
    min_vy: float = -0.30
    max_vy: float = 0.30
    min_wz: float = -0.90
    max_wz: float = 0.90

    acc_lim_x: float = 0.75
    acc_lim_y: float = 0.75
    acc_lim_theta: float = 1.40
    jerk_lim_x: float = 4.0
    jerk_lim_y: float = 4.0
    jerk_lim_theta: float = 5.0

    vx_samples: int = 5
    vy_samples: int = 5
    wz_samples: int = 5
    sim_time: float = 1.2
    dt: float = 0.1
    control_period: float = 0.1
    use_dynamic_window: bool = True

    robot_length: float = 0.24
    robot_width: float = 0.20
    safety_margin: float = 0.05

    obstacle_min_dist: float = 0.10
    hard_stop_distance: float = 0.10
    front_check_distance: float = 0.75
    front_check_width: float = 0.34
    side_check_distance: float = 0.75
    goal_tolerance: float = 0.12
    rejoin_path_distance: float = 0.16
    max_path_deviation: float = 0.18
    heading_lookahead: float = 0.45
    target_speed: float = 0.30

    path_weight: float = 2.2
    goal_weight: float = 3.0
    obstacle_weight: float = 4.0
    heading_weight: float = 0.5
    velocity_weight: float = 0.5
    smoothness_weight: float = 0.45
    lateral_weight: float = 0.15

    cmd_filter_alpha: float = 0.85
    vx_deadband: float = 0.01
    vy_deadband: float = 0.01
    wz_deadband: float = 0.02

    @classmethod
    def from_mapping(cls, values: dict) -> "DwaLiteParams":
        valid = {field.name for field in cls.__dataclass_fields__.values()}
        return cls(**{key: value for key, value in values.items() if key in valid})

    def validate(self) -> None:
        if self.max_vx < self.min_vx:
            raise ValueError("max_vx must be >= min_vx")
        if self.max_vy < self.min_vy:
            raise ValueError("max_vy must be >= min_vy")
        if self.max_wz < self.min_wz:
            raise ValueError("max_wz must be >= min_wz")
        if self.vx_samples < 1 or self.vy_samples < 1 or self.wz_samples < 1:
            raise ValueError("velocity sample counts must be >= 1")
        if self.sim_time <= 0.0 or self.dt <= 0.0 or self.control_period <= 0.0:
            raise ValueError("sim_time, dt and control_period must be positive")
        if self.robot_length <= 0.0 or self.robot_width <= 0.0:
            raise ValueError("robot footprint dimensions must be positive")
