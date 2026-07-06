#!/usr/bin/env python3

import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Sequence, Tuple

from local_planner.trajectory_sampler import (
    Pose2D,
    SamplingConfig,
    TrajectorySampler,
    Velocity2D,
    clamp,
    normalize_angle,
)


Point2D = Tuple[float, float]


class PlannerState(Enum):
    WAITING = "WAITING"
    TRACK_PATH = "TRACK_PATH"
    SIDESTEP_LEFT = "SIDESTEP_LEFT"
    SIDESTEP_RIGHT = "SIDESTEP_RIGHT"
    BYPASS_FORWARD = "BYPASS_FORWARD"
    REJOIN_PATH = "REJOIN_PATH"
    GOAL_APPROACH = "GOAL_APPROACH"
    BLOCKED_STOP = "BLOCKED_STOP"


class BypassSide(Enum):
    LEFT = "left"
    RIGHT = "right"
    NONE = "none"


@dataclass(frozen=True)
class MecanumCompetitionPlannerConfig:
    max_vx: float = 0.35
    min_vx: float = 0.0
    max_vy: float = 0.25
    max_wz: float = 0.8
    track_vx: float = 0.25
    bypass_vx: float = 0.12
    sidestep_vy: float = 0.20
    rejoin_vy: float = 0.16
    heading_gain: float = 1.2
    lateral_gain: float = 0.8
    robot_length: float = 0.24
    robot_width: float = 0.20
    obstacle_margin: float = 0.08
    front_check_distance: float = 0.75
    front_stop_distance: float = 0.32
    side_check_distance: float = 0.55
    side_min_clearance: float = 0.28
    max_path_deviation: float = 0.75
    preferred_bypass_side: BypassSide = BypassSide.LEFT
    obstacle_confirm_frames: int = 2
    obstacle_clear_frames: int = 4
    min_state_duration: float = 0.4
    side_switch_cooldown: float = 1.2
    rejoin_tolerance: float = 0.12
    goal_approach_distance: float = 0.45
    xy_goal_tolerance: float = 0.12
    goal_max_vx: float = 0.12
    goal_max_vy: float = 0.08
    goal_max_wz: float = 0.35
    control_period: float = 0.1
    sim_time: float = 1.0
    dt: float = 0.1


@dataclass(frozen=True)
class SectorModel:
    front_blocked: bool
    front_stop: bool
    left_clear: bool
    right_clear: bool
    left_clearance: float
    right_clearance: float


@dataclass(frozen=True)
class PlannerStatus:
    state: PlannerState
    obstacle_confirm_count: int
    obstacle_clear_count: int
    active_side: BypassSide
    state_age: float
    side_switch_age: float
    reason: str


@dataclass(frozen=True)
class MecanumCompetitionPlannerResult:
    best_cmd: Velocity2D
    local_trajectory: List[Pose2D]
    planner_state: PlannerStatus
    valid: bool


class MecanumCompetitionPlannerCore:
    """Pure state-machine core for robot1 mecanum competition planning."""

    def __init__(self, config: Optional[MecanumCompetitionPlannerConfig] = None) -> None:
        self.config = config if config is not None else MecanumCompetitionPlannerConfig()
        self._state = PlannerState.WAITING
        self._active_side = BypassSide.NONE
        self._obstacle_confirm_count = 0
        self._obstacle_clear_count = 0
        self._state_age = 0.0
        self._side_switch_age = self.config.side_switch_cooldown
        self._sampler = TrajectorySampler(
            SamplingConfig(
                min_vx=self.config.min_vx,
                max_vx=self.config.max_vx,
                min_vy=-self.config.max_vy,
                max_vy=self.config.max_vy,
                max_wz=self.config.max_wz,
                sim_time=self.config.sim_time,
                dt=self.config.dt,
                use_dynamic_window=False,
            )
        )

    @property
    def state(self) -> PlannerState:
        return self._state

    def reset(self) -> None:
        self._state = PlannerState.WAITING
        self._active_side = BypassSide.NONE
        self._obstacle_confirm_count = 0
        self._obstacle_clear_count = 0
        self._state_age = 0.0
        self._side_switch_age = self.config.side_switch_cooldown

    def plan(
        self,
        pose: Pose2D,
        current_velocity: Velocity2D,
        path: Sequence[object],
        obstacles_robot_frame: Sequence[Point2D] = (),
        dt: Optional[float] = None,
    ) -> MecanumCompetitionPlannerResult:
        step_dt = max(0.0, self.config.control_period if dt is None else dt)
        self._state_age += step_dt
        self._side_switch_age += step_dt

        path_points = _coerce_path(path)
        if not _finite_pose(pose) or not path_points:
            self._transition(PlannerState.WAITING, "waiting_for_inputs")
            return self._result(pose, Velocity2D(0.0, 0.0, 0.0), False, "waiting_for_inputs")

        sectors = self._build_sector_model(obstacles_robot_frame)
        self._update_obstacle_hysteresis(sectors.front_blocked)
        obstacle_active = self._obstacle_confirm_count >= max(1, self.config.obstacle_confirm_frames)
        obstacle_clear = self._obstacle_clear_count >= max(1, self.config.obstacle_clear_frames)
        distance_to_goal = _distance((pose.x, pose.y), path_points[-1])
        cross_track = _distance_to_path((pose.x, pose.y), path_points)
        can_leave = (
            self._state in (PlannerState.WAITING, PlannerState.BLOCKED_STOP)
            or self._state_age >= max(0.0, self.config.min_state_duration)
        )

        reason = "tracking_path"
        if distance_to_goal <= self.config.xy_goal_tolerance:
            self._transition(PlannerState.GOAL_APPROACH, "goal_reached")
            return self._result(pose, Velocity2D(0.0, 0.0, 0.0), True, "goal_reached")

        if self._state == PlannerState.WAITING:
            self._transition(PlannerState.TRACK_PATH, "path_ready")

        if can_leave and distance_to_goal <= self.config.goal_approach_distance:
            self._transition(PlannerState.GOAL_APPROACH, "goal_approach")
        elif can_leave and sectors.front_stop and not (sectors.left_clear or sectors.right_clear):
            self._transition(PlannerState.BLOCKED_STOP, "blocked_all_sides")
        elif can_leave and self._state == PlannerState.TRACK_PATH and obstacle_active:
            side = self._select_side(sectors)
            if side == BypassSide.NONE:
                self._transition(PlannerState.BLOCKED_STOP, "blocked_all_sides")
            else:
                self._set_side(side)
                self._transition(_sidestep_state(side), "front_obstacle_confirmed")
        elif can_leave and self._state in (PlannerState.SIDESTEP_LEFT, PlannerState.SIDESTEP_RIGHT):
            if not self._side_is_clear(self._active_side, sectors):
                side = self._select_side(sectors)
                if side == BypassSide.NONE or side != self._active_side:
                    self._transition(PlannerState.BLOCKED_STOP, "side_blocked")
            elif obstacle_clear:
                self._transition(PlannerState.BYPASS_FORWARD, "front_clear")
        elif can_leave and self._state == PlannerState.BYPASS_FORWARD and obstacle_clear:
            self._transition(PlannerState.REJOIN_PATH, "rejoin_after_bypass")
        elif can_leave and self._state == PlannerState.REJOIN_PATH:
            if obstacle_active:
                side = self._select_side(sectors)
                if side == BypassSide.NONE:
                    self._transition(PlannerState.BLOCKED_STOP, "blocked_all_sides")
                else:
                    self._set_side(side)
                    self._transition(_sidestep_state(side), "new_obstacle")
            elif cross_track <= self.config.rejoin_tolerance:
                self._active_side = BypassSide.NONE
                self._transition(PlannerState.TRACK_PATH, "path_rejoined")
        elif self._state == PlannerState.BLOCKED_STOP:
            if not obstacle_active and obstacle_clear:
                self._active_side = BypassSide.NONE
                self._transition(PlannerState.TRACK_PATH, "obstacle_cleared")
            elif obstacle_active:
                side = self._select_side(sectors)
                if side != BypassSide.NONE:
                    self._set_side(side)
                    self._transition(_sidestep_state(side), "side_available")

        cmd = self._command_for_state(pose, current_velocity, path_points)
        if self._state == PlannerState.BLOCKED_STOP:
            reason = "blocked_stop"
        elif self._state == PlannerState.GOAL_APPROACH:
            reason = "goal_approach"
        elif self._state in (PlannerState.SIDESTEP_LEFT, PlannerState.SIDESTEP_RIGHT):
            reason = "sidestep"
        elif self._state == PlannerState.BYPASS_FORWARD:
            reason = "bypass_forward"
        elif self._state == PlannerState.REJOIN_PATH:
            reason = "rejoin_path"
        return self._result(pose, cmd, True, reason)

    def _update_obstacle_hysteresis(self, front_blocked: bool) -> None:
        if front_blocked:
            self._obstacle_confirm_count += 1
            self._obstacle_clear_count = 0
        else:
            self._obstacle_clear_count += 1
            if self._obstacle_clear_count >= max(1, self.config.obstacle_clear_frames):
                self._obstacle_confirm_count = 0

    def _build_sector_model(self, obstacles_robot_frame: Sequence[Point2D]) -> SectorModel:
        half_width = 0.5 * max(0.0, self.config.robot_width)
        front_half_width = half_width + self.config.obstacle_margin
        left_clearance = math.inf
        right_clearance = math.inf
        front_blocked = False
        front_stop = False

        for x, y in obstacles_robot_frame:
            if not math.isfinite(x) or not math.isfinite(y):
                continue
            if 0.0 <= x <= self.config.front_check_distance and abs(y) <= front_half_width:
                front_blocked = True
                if x <= self.config.front_stop_distance:
                    front_stop = True
            if -half_width <= x <= self.config.side_check_distance and y > 0.0:
                left_clearance = min(left_clearance, max(0.0, y - half_width))
            if -half_width <= x <= self.config.side_check_distance and y < 0.0:
                right_clearance = min(right_clearance, max(0.0, abs(y) - half_width))

        left_clear = left_clearance >= self.config.side_min_clearance
        right_clear = right_clearance >= self.config.side_min_clearance
        return SectorModel(
            front_blocked=front_blocked,
            front_stop=front_stop,
            left_clear=left_clear,
            right_clear=right_clear,
            left_clearance=left_clearance,
            right_clearance=right_clearance,
        )

    def _select_side(self, sectors: SectorModel) -> BypassSide:
        if self._active_side != BypassSide.NONE and self._side_is_clear(self._active_side, sectors):
            return self._active_side

        left_available = sectors.left_clear
        right_available = sectors.right_clear
        if not left_available and not right_available:
            return BypassSide.NONE
        if left_available and not right_available:
            return self._cooldown_side(BypassSide.LEFT)
        if right_available and not left_available:
            return self._cooldown_side(BypassSide.RIGHT)

        if math.isinf(sectors.left_clearance) and math.isinf(sectors.right_clearance):
            return self._cooldown_side(self.config.preferred_bypass_side)
        if abs(sectors.left_clearance - sectors.right_clearance) < 0.05:
            return self._cooldown_side(self.config.preferred_bypass_side)
        if sectors.left_clearance > sectors.right_clearance:
            return self._cooldown_side(BypassSide.LEFT)
        return self._cooldown_side(BypassSide.RIGHT)

    def _cooldown_side(self, requested: BypassSide) -> BypassSide:
        if self._active_side in (BypassSide.NONE, requested):
            return requested
        if self._side_switch_age >= max(0.0, self.config.side_switch_cooldown):
            return requested
        return BypassSide.NONE

    def _side_is_clear(self, side: BypassSide, sectors: SectorModel) -> bool:
        if side == BypassSide.LEFT:
            return sectors.left_clear
        if side == BypassSide.RIGHT:
            return sectors.right_clear
        return False

    def _set_side(self, side: BypassSide) -> None:
        if side != BypassSide.NONE and side != self._active_side:
            self._active_side = side
            self._side_switch_age = 0.0

    def _command_for_state(
        self,
        pose: Pose2D,
        current_velocity: Velocity2D,
        path_points: Sequence[Point2D],
    ) -> Velocity2D:
        if self._state in (PlannerState.WAITING, PlannerState.BLOCKED_STOP):
            return Velocity2D(0.0, 0.0, 0.0)

        tracking = self._path_tracking_command(pose, path_points, self.config.track_vx)
        if self._state == PlannerState.TRACK_PATH:
            return self._clamp_cmd(tracking)
        if self._state == PlannerState.GOAL_APPROACH:
            return self._clamp_cmd(
                Velocity2D(
                    min(tracking.vx, self.config.goal_max_vx),
                    clamp(tracking.vy, -self.config.goal_max_vy, self.config.goal_max_vy),
                    clamp(tracking.wz, -self.config.goal_max_wz, self.config.goal_max_wz),
                )
            )
        if self._state == PlannerState.SIDESTEP_LEFT:
            return self._clamp_cmd(Velocity2D(self.config.bypass_vx * 0.4, self.config.sidestep_vy, 0.0))
        if self._state == PlannerState.SIDESTEP_RIGHT:
            return self._clamp_cmd(Velocity2D(self.config.bypass_vx * 0.4, -self.config.sidestep_vy, 0.0))
        if self._state == PlannerState.BYPASS_FORWARD:
            side_sign = 1.0 if self._active_side == BypassSide.LEFT else -1.0
            return self._clamp_cmd(
                Velocity2D(self.config.bypass_vx, side_sign * self.config.sidestep_vy * 0.35, tracking.wz)
            )
        if self._state == PlannerState.REJOIN_PATH:
            rejoin = self._path_tracking_command(pose, path_points, self.config.bypass_vx)
            return self._clamp_cmd(
                Velocity2D(
                    rejoin.vx,
                    clamp(rejoin.vy, -self.config.rejoin_vy, self.config.rejoin_vy),
                    rejoin.wz,
                )
            )
        return self._clamp_cmd(current_velocity)

    def _path_tracking_command(
        self,
        pose: Pose2D,
        path_points: Sequence[Point2D],
        vx_limit: float,
    ) -> Velocity2D:
        target = _lookahead_point((pose.x, pose.y), path_points)
        dx = target[0] - pose.x
        dy = target[1] - pose.y
        cos_yaw = math.cos(pose.yaw)
        sin_yaw = math.sin(pose.yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        heading_error = normalize_angle(math.atan2(dy, dx) - pose.yaw) if dx or dy else 0.0
        vx = clamp(local_x, self.config.min_vx, vx_limit)
        vy = clamp(self.config.lateral_gain * local_y, -self.config.max_vy, self.config.max_vy)
        wz = clamp(self.config.heading_gain * heading_error, -self.config.max_wz, self.config.max_wz)
        return Velocity2D(vx, vy, wz)

    def _clamp_cmd(self, cmd: Velocity2D) -> Velocity2D:
        return Velocity2D(
            clamp(cmd.vx, self.config.min_vx, self.config.max_vx),
            clamp(cmd.vy, -self.config.max_vy, self.config.max_vy),
            clamp(cmd.wz, -self.config.max_wz, self.config.max_wz),
        )

    def _transition(self, next_state: PlannerState, reason: str) -> None:
        if next_state != self._state:
            self._state = next_state
            self._state_age = 0.0

    def _result(
        self,
        pose: Pose2D,
        cmd: Velocity2D,
        valid: bool,
        reason: str,
    ) -> MecanumCompetitionPlannerResult:
        local_trajectory = self._sampler.rollout(pose, cmd) if valid else [pose]
        return MecanumCompetitionPlannerResult(
            best_cmd=cmd,
            local_trajectory=local_trajectory,
            planner_state=PlannerStatus(
                state=self._state,
                obstacle_confirm_count=self._obstacle_confirm_count,
                obstacle_clear_count=self._obstacle_clear_count,
                active_side=self._active_side,
                state_age=self._state_age,
                side_switch_age=self._side_switch_age,
                reason=reason,
            ),
            valid=valid,
        )


def _sidestep_state(side: BypassSide) -> PlannerState:
    return PlannerState.SIDESTEP_LEFT if side == BypassSide.LEFT else PlannerState.SIDESTEP_RIGHT


def _coerce_path(path: Sequence[object]) -> List[Point2D]:
    points: List[Point2D] = []
    for item in path:
        if isinstance(item, Pose2D):
            points.append((item.x, item.y))
        else:
            try:
                x = float(item[0])  # type: ignore[index]
                y = float(item[1])  # type: ignore[index]
            except (TypeError, ValueError, IndexError):
                continue
            if math.isfinite(x) and math.isfinite(y):
                points.append((x, y))
    return points


def _finite_pose(pose: Pose2D) -> bool:
    return math.isfinite(pose.x) and math.isfinite(pose.y) and math.isfinite(pose.yaw)


def _lookahead_point(current: Point2D, path_points: Sequence[Point2D]) -> Point2D:
    nearest_index = min(
        range(len(path_points)),
        key=lambda index: _distance(current, path_points[index]),
    )
    return path_points[min(len(path_points) - 1, nearest_index + 1)]


def _distance_to_path(point: Point2D, path_points: Sequence[Point2D]) -> float:
    return min(_distance(point, path_point) for path_point in path_points) if path_points else math.inf


def _distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
