#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from local_planner.footprint_collision_checker import (
    FootprintCollisionChecker,
    FootprintConfig,
)
from local_planner.obstacle_processor import robot_to_world_points
from local_planner.trajectory_evaluator import (
    CostWeights,
    TrajectoryCost,
    TrajectoryEvaluator,
)
from local_planner.trajectory_sampler import (
    Pose2D,
    SamplingConfig,
    TrajectorySampler,
    Velocity2D,
)


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class MecanumOmniConfig:
    sampling: SamplingConfig = SamplingConfig()
    footprint: FootprintConfig = FootprintConfig()
    weights: CostWeights = CostWeights()
    goal_tolerance: float = 0.12
    obstacle_range: float = 2.5
    target_velocity: float = 0.25


@dataclass(frozen=True)
class CandidateEvaluation:
    cmd: Velocity2D
    trajectory: List[Pose2D]
    cost: TrajectoryCost


@dataclass(frozen=True)
class MecanumOmniResult:
    best_cmd: Velocity2D
    best_trajectory: List[Pose2D]
    best_cost: Optional[TrajectoryCost]
    candidates: List[CandidateEvaluation]
    valid: bool
    reason: str


class MecanumOmniPlanner:
    """Pure robot1 mecanum DWA core.

    Inputs and outputs are plain Python objects. This class does not create ROS2
    nodes, publishers, subscriptions, messages, or topic names.
    """

    def __init__(self, config: MecanumOmniConfig) -> None:
        self.config = config
        self.sampler = TrajectorySampler(config.sampling)
        self.collision_checker = FootprintCollisionChecker(config.footprint)
        self.evaluator = TrajectoryEvaluator(
            config.weights,
            self.collision_checker,
            config.target_velocity,
        )

    def plan(
        self,
        pose: Pose2D,
        current_velocity: Velocity2D,
        path_points: Sequence[Point2D],
        goal: Optional[Point2D] = None,
        obstacles_robot_frame: Sequence[Point2D] = (),
        previous_command: Optional[Velocity2D] = None,
    ) -> MecanumOmniResult:
        if not _finite_pose(pose):
            return self._zero_result(pose, "invalid_pose")
        if not path_points and goal is None:
            return self._zero_result(pose, "empty_path")

        active_goal = goal if goal is not None else path_points[-1]
        if _distance((pose.x, pose.y), active_goal) <= self.config.goal_tolerance:
            return self._zero_result(pose, "goal_reached", valid=True)

        obstacles_world = robot_to_world_points(
            pose,
            _filter_obstacles(obstacles_robot_frame, self.config.obstacle_range),
        )
        previous = previous_command if previous_command is not None else Velocity2D(0.0, 0.0, 0.0)
        active_path = path_points if path_points else [active_goal]

        feasible: List[CandidateEvaluation] = []
        for cmd in self.sampler.sample(current_velocity):
            trajectory = self.sampler.rollout(pose, cmd)
            cost = self.evaluator.evaluate(
                trajectory=trajectory,
                velocity=cmd,
                current_velocity=current_velocity,
                previous_command=previous,
                path_points=active_path,
                goal=active_goal,
                obstacles_world=obstacles_world,
            )
            if math.isfinite(cost.total_cost):
                feasible.append(CandidateEvaluation(cmd=cmd, trajectory=trajectory, cost=cost))

        if not feasible:
            return self._zero_result(pose, "no_feasible_trajectory")

        best = min(feasible, key=lambda candidate: candidate.cost.total_cost)
        return MecanumOmniResult(
            best_cmd=best.cmd,
            best_trajectory=best.trajectory,
            best_cost=best.cost,
            candidates=feasible,
            valid=True,
            reason="ok",
        )

    @staticmethod
    def _zero_result(
        pose: Pose2D,
        reason: str,
        valid: bool = False,
    ) -> MecanumOmniResult:
        return MecanumOmniResult(
            best_cmd=Velocity2D(0.0, 0.0, 0.0),
            best_trajectory=[pose],
            best_cost=None,
            candidates=[],
            valid=valid,
            reason=reason,
        )


def _filter_obstacles(points: Sequence[Point2D], obstacle_range: float) -> List[Point2D]:
    filtered: List[Point2D] = []
    for x, y in points:
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        if _distance((0.0, 0.0), (x, y)) <= obstacle_range:
            filtered.append((x, y))
    return filtered


def _finite_pose(pose: Pose2D) -> bool:
    return math.isfinite(pose.x) and math.isfinite(pose.y) and math.isfinite(pose.yaw)


def _distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
