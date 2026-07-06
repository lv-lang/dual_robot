#!/usr/bin/env python3

from typing import Sequence

from local_planner.dwa_lite_params import DwaLiteParams
from local_planner.footprint_checker import DwaLiteFootprintChecker
from local_planner.planner_utils import (
    PlannerResult,
    PlannerState,
    Point2D,
    Pose2D,
    Trajectory2D,
    Velocity2D,
    distance,
    distance_to_path,
    lookahead_point,
    robot_points_to_world,
    zero_velocity,
)
from local_planner.scan_obstacle_model import ScanObstacleModel
from local_planner.trajectory_rollout import rollout_mecanum
from local_planner.trajectory_sampler import DwaLiteTrajectorySampler
from local_planner.trajectory_scorer import DwaLiteTrajectoryScorer
from local_planner.velocity_smoother import DwaLiteVelocitySmoother


class DwaLitePlannerCore:
    """Pure DWA Lite planner for robot1's mecanum base."""

    def __init__(self, params: DwaLiteParams | None = None) -> None:
        self.params = params or DwaLiteParams()
        self.params.validate()
        self.sampler = DwaLiteTrajectorySampler(self.params)
        self.footprint_checker = DwaLiteFootprintChecker(self.params)
        self.scorer = DwaLiteTrajectoryScorer(self.params, self.footprint_checker)
        self.obstacle_model = ScanObstacleModel(self.params)
        self.smoother = DwaLiteVelocitySmoother(self.params)

    def reset(self) -> None:
        self.smoother.reset()

    def plan(
        self,
        current_pose: Pose2D | None,
        current_velocity: Velocity2D | None,
        global_path: Sequence[Point2D],
        scan_obstacle_points: Sequence[Point2D],
        *,
        odom_fresh: bool = True,
        scan_fresh: bool = True,
    ) -> PlannerResult:
        if not odom_fresh or current_pose is None or current_velocity is None:
            return self._stop(PlannerState.STALE_ODOM, "stale_odom")
        if not scan_fresh:
            return self._stop(PlannerState.STALE_SCAN, "stale_scan")
        if not global_path:
            return self._stop(PlannerState.EMPTY_PATH, "empty_path")

        final_goal = global_path[-1]
        if distance((current_pose.x, current_pose.y), final_goal) <= self.params.goal_tolerance:
            return self._stop(PlannerState.GOAL_REACHED, "goal_reached", [current_pose])
        local_goal = lookahead_point(
            (current_pose.x, current_pose.y),
            global_path,
            self.params.heading_lookahead,
        )

        obstacle_summary = self.obstacle_model.summarize(scan_obstacle_points)
        if obstacle_summary.hard_stop:
            return self._stop(
                PlannerState.BLOCKED_STOP,
                "hard_stop_obstacle",
                [current_pose],
                {"front_clearance": obstacle_summary.front_clearance},
            )

        obstacles_world = robot_points_to_world(current_pose, scan_obstacle_points)
        best = None
        best_score = None
        for command in self.sampler.sample(current_velocity):
            trajectory = Trajectory2D(
                command=command,
                poses=rollout_mecanum(current_pose, command, self.params.sim_time, self.params.dt),
            )
            score = self.scorer.score(
                trajectory,
                global_path,
                local_goal,
                obstacles_world,
                current_velocity,
                obstacle_summary,
            )
            if not score.feasible:
                continue
            if best_score is None or score.total_cost < best_score.total_cost:
                best = trajectory
                best_score = score

        if best is None or best_score is None:
            return self._stop(
                PlannerState.BLOCKED_STOP,
                "no_feasible_trajectory",
                [current_pose],
                {"front_clearance": obstacle_summary.front_clearance},
            )

        smoothed_cmd = self.smoother.smooth(best.command)
        smoothed_trajectory = rollout_mecanum(current_pose, smoothed_cmd, self.params.sim_time, self.params.dt)
        state = self._state_for(current_pose, smoothed_cmd, global_path, obstacle_summary.front_blocked)
        return PlannerResult(
            best_cmd=smoothed_cmd,
            best_trajectory=smoothed_trajectory,
            planner_state=state,
            valid=True,
            debug={
                "cost": best_score.total_cost,
                "path_cost": best_score.path_cost,
                "goal_cost": best_score.goal_cost,
                "obstacle_cost": best_score.obstacle_cost,
                "heading_cost": best_score.heading_cost,
                "velocity_cost": best_score.velocity_cost,
                "smoothness_cost": best_score.smoothness_cost,
                "lateral_cost": best_score.lateral_cost,
                "front_clearance": obstacle_summary.front_clearance,
            },
        )

    def _state_for(
        self,
        pose: Pose2D,
        command: Velocity2D,
        path: Sequence[Point2D],
        front_blocked: bool,
    ) -> PlannerState:
        if front_blocked and command.vy > max(0.02, self.params.vy_deadband):
            return PlannerState.SIDESTEP_LEFT
        if front_blocked and command.vy < -max(0.02, self.params.vy_deadband):
            return PlannerState.SIDESTEP_RIGHT
        if front_blocked:
            return PlannerState.AVOID_OBSTACLE
        if distance_to_path((pose.x, pose.y), path) > self.params.rejoin_path_distance:
            return PlannerState.REJOIN_PATH
        return PlannerState.TRACK_PATH

    def _stop(
        self,
        state: PlannerState,
        reason: str,
        trajectory=None,
        debug=None,
    ) -> PlannerResult:
        self.smoother.reset()
        return PlannerResult(
            best_cmd=zero_velocity(),
            best_trajectory=list(trajectory or []),
            planner_state=state,
            valid=False,
            reason=reason,
            debug=dict(debug or {}),
        )
