#!/usr/bin/env python3

import math
from typing import List

from local_planner.planner_utils import Pose2D, Velocity2D, normalize_angle


def rollout_mecanum(start: Pose2D, command: Velocity2D, sim_time: float, dt: float) -> List[Pose2D]:
    step = max(0.02, dt)
    steps = max(1, int(math.ceil(max(step, sim_time) / step)))
    pose = start
    trajectory = [pose]
    for _ in range(steps):
        cos_yaw = math.cos(pose.yaw)
        sin_yaw = math.sin(pose.yaw)
        x_dot = cos_yaw * command.vx - sin_yaw * command.vy
        y_dot = sin_yaw * command.vx + cos_yaw * command.vy
        pose = Pose2D(
            x=pose.x + x_dot * step,
            y=pose.y + y_dot * step,
            yaw=normalize_angle(pose.yaw + command.wz * step),
        )
        trajectory.append(pose)
    return trajectory
