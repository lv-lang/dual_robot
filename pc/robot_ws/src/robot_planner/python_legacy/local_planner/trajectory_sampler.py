#!/usr/bin/env python3

from typing import Iterable, List

from local_planner.dwa_lite_params import DwaLiteParams
from local_planner.planner_utils import Velocity2D, clamp


class DwaLiteTrajectorySampler:
    """Samples vx, vy and wz for a holonomic mecanum DWA window."""

    def __init__(self, params: DwaLiteParams) -> None:
        self.params = params

    def sample(self, current_velocity: Velocity2D) -> Iterable[Velocity2D]:
        vx_values = _sample_axis(
            self.params.min_vx,
            self.params.max_vx,
            current_velocity.vx,
            self.params.acc_lim_x,
            self.params.vx_samples,
            self.params.control_period,
            self.params.use_dynamic_window,
        )
        vy_values = _sample_axis(
            self.params.min_vy,
            self.params.max_vy,
            current_velocity.vy,
            self.params.acc_lim_y,
            self.params.vy_samples,
            self.params.control_period,
            self.params.use_dynamic_window,
        )
        wz_values = _sample_axis(
            self.params.min_wz,
            self.params.max_wz,
            current_velocity.wz,
            self.params.acc_lim_theta,
            self.params.wz_samples,
            self.params.control_period,
            self.params.use_dynamic_window,
        )
        for vx in vx_values:
            for vy in vy_values:
                for wz in wz_values:
                    yield Velocity2D(vx=vx, vy=vy, wz=wz)


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
        window = abs(acc_limit) * max(0.0, control_period)
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
