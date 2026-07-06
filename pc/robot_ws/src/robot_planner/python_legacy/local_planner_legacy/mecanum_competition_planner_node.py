#!/usr/bin/env python3

import json
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from local_planner.competition_core import (
    BypassSide,
    MecanumCompetitionPlannerConfig,
    MecanumCompetitionPlannerCore,
)
from local_planner.external_dwa_adapter import (
    ExternalDwaConfig,
    ExternalDwaCore,
    ExternalDwaPose,
    ExternalDwaVelocity,
    scan_points_from_ranges,
)
from local_planner.trajectory_sampler import Pose2D as CorePose2D
from local_planner.trajectory_sampler import Velocity2D as CoreVelocity2D


ROBOT1_LOCAL_PATH_TOPIC = "/robot1/local_path"
ROBOT1_CMD_VEL_RAW_TOPIC = "/robot1/cmd_vel_raw"
ROBOT1_PLANNER_STATE_TOPIC = "/robot1/planner_state"


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


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def finite_range(value: float, scan: LaserScan) -> bool:
    return (
        math.isfinite(value)
        and value >= max(0.0, scan.range_min)
        and (scan.range_max <= 0.0 or value <= scan.range_max)
    )


class MecanumCompetitionPlannerNode(Node):
    """ROS2 adapter for the pure robot1 mecanum competition planner core."""

    VALID_MODES = {"simple_tracker", "external_dwa", "mecanum_competition"}

    def __init__(self) -> None:
        super().__init__("robot1_mecanum_competition_planner")

        self.global_path_topic = self._declare_robot1_topic(
            "global_path_topic", "/robot1/global_path"
        )
        self.odom_topic = self._declare_robot1_topic("odom_topic", "/robot1/odom")
        self.scan_topic = self._declare_robot1_topic("scan_topic", "/robot1/scan")
        self.local_path_topic = self._declare_fixed_output_topic(
            "local_path_topic", ROBOT1_LOCAL_PATH_TOPIC
        )
        self.cmd_vel_raw_topic = self._declare_fixed_output_topic(
            "cmd_vel_raw_topic", ROBOT1_CMD_VEL_RAW_TOPIC
        )
        self.planner_state_topic = self._declare_fixed_output_topic(
            "planner_state_topic", ROBOT1_PLANNER_STATE_TOPIC
        )

        self.planner_mode = self._declare_str("planner_mode", "mecanum_competition")
        self.fallback_planner_mode = self._declare_str(
            "fallback_planner_mode", "simple_tracker"
        )
        self.enable_lateral_motion = self._declare_bool("enable_lateral_motion", True)
        enable_vy_param = self._declare_bool("enable_vy", self.enable_lateral_motion)
        self.enable_vy = self.enable_lateral_motion and enable_vy_param
        self.control_frequency = self._declare_float("control_frequency", 15.0)
        self.lookahead_distance = self._declare_float("lookahead_distance", 0.55)
        self.goal_tolerance = self._declare_float("goal_tolerance", 0.12)
        self.path_timeout = self._declare_float("path_timeout", 0.0)
        self.odom_timeout = self._declare_float("odom_timeout", 0.5)
        self.scan_timeout = self._declare_float("scan_timeout", 0.5)
        self.require_fresh_scan = self._declare_bool("require_fresh_scan", True)
        self.velocity_feedback_mode = self._declare_str(
            "velocity_feedback_mode", "last_command"
        )

        self.min_vx = self._declare_float("min_vx", 0.0)
        self.max_vx = self._declare_float("max_vx", 0.35)
        self.max_vy = self._declare_float("max_vy", 0.25)
        self.max_wz = self._declare_float("max_wz", 0.8)
        self.track_vx = self._declare_float("track_vx", 0.25)
        self.bypass_vx = self._declare_float("bypass_vx", 0.12)
        self.sidestep_vy = self._declare_float("sidestep_vy", 0.20)
        self.rejoin_vy = self._declare_float("rejoin_vy", 0.16)
        self.heading_gain = self._declare_float("heading_gain", 1.2)
        self.lateral_gain = self._declare_float("lateral_gain", 0.8)
        self.k_linear = self._declare_float("k_linear", 0.75)
        self.k_lateral = self._declare_float("k_lateral", 0.75)
        self.k_angular = self._declare_float("k_angular", 1.20)

        self.acc_lim_x = self._declare_float("acc_lim_x", 0.50)
        self.acc_lim_y = self._declare_float("acc_lim_y", 0.45)
        self.acc_lim_theta = self._declare_float("acc_lim_theta", 1.00)
        self.vx_samples = self._declare_int("vx_samples", 5)
        self.wz_samples = self._declare_int("wz_samples", 7)
        self.sim_time = self._declare_float("sim_time", 1.0)
        self.sim_dt = self._declare_float("dt", 0.1)
        self.target_velocity = self._declare_float("target_velocity", self.track_vx)

        self.robot_length = self._declare_float("robot_length", 0.24)
        self.robot_width = self._declare_float("robot_width", 0.20)
        self.obstacle_margin = self._declare_float("obstacle_margin", 0.08)
        self.obstacle_range = self._declare_float("obstacle_range", 2.5)
        self.front_check_distance = self._declare_float("front_check_distance", 0.75)
        self.front_stop_distance = self._declare_float("front_stop_distance", 0.32)
        self.hard_stop_distance_m = self._declare_float("hard_stop_distance_m", 0.10)
        self.front_stop_angle = self._declare_float("front_stop_angle", 0.70)
        self.side_check_distance = self._declare_float("side_check_distance", 0.55)
        self.side_min_clearance = self._declare_float("side_min_clearance", 0.28)
        self.max_path_deviation = self._declare_float("max_path_deviation", 0.75)
        self.preferred_bypass_side = self._declare_str("preferred_bypass_side", "left")
        self.obstacle_confirm_frames = self._declare_int("obstacle_confirm_frames", 2)
        self.obstacle_clear_frames = self._declare_int("obstacle_clear_frames", 4)
        self.min_state_duration = self._declare_float("min_state_duration", 0.4)
        self.side_switch_cooldown = self._declare_float("side_switch_cooldown", 1.2)
        self.rejoin_tolerance = self._declare_float("rejoin_tolerance", 0.12)
        self.goal_approach_distance = self._declare_float("goal_approach_distance", 0.45)
        self.xy_goal_tolerance = self._declare_float("xy_goal_tolerance", 0.12)
        self.goal_max_vx = self._declare_float("goal_max_vx", 0.12)
        self.goal_max_vy = self._declare_float("goal_max_vy", 0.08)
        self.goal_max_wz = self._declare_float("goal_max_wz", 0.35)

        self.external_dwa_angle_resolution = self._declare_float(
            "external_dwa_angle_resolution", 0.087
        )
        self.external_dwa_angle_to_goal_turn_threshold = self._declare_float(
            "external_dwa_angle_to_goal_turn_threshold", 0.35
        )
        self.external_dwa_min_in_place_wz = self._declare_float(
            "external_dwa_min_in_place_wz", 0.20
        )
        self.external_dwa_max_in_place_wz = self._declare_float(
            "external_dwa_max_in_place_wz", self.max_wz
        )
        self.weight_path_distance = self._declare_float("weight_path_distance", 2.0)
        self.weight_target_distance = self._declare_float("weight_target_distance", 3.0)
        self.weight_heading = self._declare_float("weight_heading", 0.8)
        self.weight_obstacle = self._declare_float("weight_obstacle", 0.6)
        self.weight_smoothness = self._declare_float("weight_smoothness", 0.4)
        self.weight_speed = self._declare_float("weight_speed", 0.15)

        self.path_points: List[Tuple[float, float]] = []
        self.path_progress_index = 0
        self.path_frame = "map"
        self.odom: Optional[Odometry] = None
        self.scan: Optional[LaserScan] = None
        self.last_path_time = None
        self.last_odom_time = None
        self.last_scan_time = None
        self.last_status_reason = ""
        self.last_command = Velocity2D(0.0, 0.0, 0.0)
        self._competition_core = self._make_competition_core()

        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        path_qos.reliability = ReliabilityPolicy.RELIABLE

        self.path_sub = self.create_subscription(
            Path, self.global_path_topic, self._on_path, path_qos
        )
        self.path_volatile_sub = self.create_subscription(
            Path, self.global_path_topic, self._on_path, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, qos_profile_sensor_data
        )
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.local_path_pub = self.create_publisher(Path, self.local_path_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_raw_topic, 10)
        self.state_pub = self.create_publisher(String, self.planner_state_topic, 10)

        period = 1.0 / max(1.0, self.control_frequency)
        self.timer = self.create_timer(period, self._on_timer)

    def _declare_bool(self, name: str, default: bool) -> bool:
        return bool(self.declare_parameter(name, default).value)

    def _declare_float(self, name: str, default: float) -> float:
        return float(self.declare_parameter(name, default).value)

    def _declare_int(self, name: str, default: int) -> int:
        return int(self.declare_parameter(name, default).value)

    def _declare_str(self, name: str, default: str) -> str:
        return str(self.declare_parameter(name, default).value)

    def _declare_robot1_topic(self, name: str, default: str) -> str:
        value = self._declare_str(name, default)
        if value.startswith("/robot1/"):
            return value
        self.get_logger().warning(
            f"Ignoring non-/robot1 topic parameter {name}={value}; using {default}"
        )
        return default

    def _declare_fixed_output_topic(self, name: str, allowed: str) -> str:
        value = self._declare_str(name, allowed)
        if value == allowed:
            return value
        self.get_logger().warning(
            f"Ignoring output topic parameter {name}={value}; this node only publishes {allowed}"
        )
        return allowed

    def _on_path(self, msg: Path) -> None:
        self.path_frame = msg.header.frame_id or "map"
        self.path_points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
            if math.isfinite(pose.pose.position.x)
            and math.isfinite(pose.pose.position.y)
        ]
        self.path_progress_index = 0
        self.last_path_time = self.get_clock().now()

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg
        self.last_odom_time = self.get_clock().now()

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg
        self.last_scan_time = self.get_clock().now()

    def _on_timer(self) -> None:
        mode = self._selected_planner_mode()
        reason = self._not_ready_reason()
        if reason:
            self._publish_stop(reason, mode=mode)
            return

        pose = self._current_pose()
        if pose is None:
            self._publish_stop("waiting for valid odom pose", mode=mode)
            return

        if self._front_obstacle_too_close():
            self._publish_stop("front obstacle inside hard stop distance", mode=mode)
            self._publish_local_path([pose])
            return

        target = self._select_lookahead_point(pose)
        if target is None:
            self._publish_stop("empty global path", mode=mode)
            self._publish_local_path([pose])
            return

        try:
            if mode == "mecanum_competition":
                self._run_mecanum_competition(pose)
            elif mode == "external_dwa":
                self._run_external_dwa(pose, target)
            else:
                self._run_simple_tracker(pose, target)
        except Exception as exc:
            self._publish_stop(f"{mode} adapter error: {exc}", mode=mode)
            self._publish_local_path([pose])

    def _selected_planner_mode(self) -> str:
        if self.planner_mode in self.VALID_MODES:
            return self.planner_mode
        return self._selected_fallback_mode()

    def _selected_fallback_mode(self) -> str:
        if self.fallback_planner_mode in self.VALID_MODES:
            return self.fallback_planner_mode
        return "simple_tracker"

    def _run_mecanum_competition(self, pose: Pose2D) -> None:
        scan = self._active_scan()
        if scan is None:
            self._publish_stop("mecanum_competition waiting for fresh scan")
            self._publish_local_path([pose])
            return

        current = self._planner_velocity_feedback()
        result = self._competition_core.plan(
            pose=CorePose2D(pose.x, pose.y, pose.yaw),
            current_velocity=CoreVelocity2D(current.vx, current.vy, current.wz),
            path=self.path_points[self.path_progress_index:],
            obstacles_robot_frame=self._scan_obstacles(scan),
            dt=1.0 / max(1.0, self.control_frequency),
        )
        status = self._competition_status_dict(result)
        trajectory = [
            Pose2D(state.x, state.y, state.yaw)
            for state in getattr(result, "local_trajectory", [])
            if math.isfinite(state.x) and math.isfinite(state.y) and math.isfinite(state.yaw)
        ]
        if not trajectory:
            trajectory = [pose]

        if not bool(getattr(result, "valid", False)):
            reason = self._competition_reason(result, "invalid_competition_result")
            self._publish_stop(
                f"mecanum_competition {reason}",
                mode="mecanum_competition",
                extra=status,
            )
            self._publish_local_path(trajectory)
            return

        velocity = getattr(result, "best_cmd", CoreVelocity2D(0.0, 0.0, 0.0))
        if not (
            math.isfinite(velocity.vx)
            and math.isfinite(velocity.vy)
            and math.isfinite(velocity.wz)
        ):
            self._publish_stop(
                "mecanum_competition non-finite command",
                mode="mecanum_competition",
                extra=status,
            )
            self._publish_local_path(trajectory)
            return

        reason = self._competition_reason(result, "ok")
        stopped = self._is_zero_velocity(velocity) or reason == "goal_reached"
        self._publish_cmd(
            Velocity2D(velocity.vx, velocity.vy, velocity.wz),
            "mecanum_competition",
            reason,
            stopped=stopped,
            extra=status,
        )
        self._publish_local_path(trajectory)

    def _run_external_dwa(self, pose: Pose2D, target: Tuple[float, float]) -> None:
        scan = self._active_scan()
        if scan is None:
            self._publish_stop("external_dwa waiting for fresh scan", mode="external_dwa")
            self._publish_local_path([pose])
            return

        current = self._planner_velocity_feedback()
        obstacles = scan_points_from_ranges(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            self.external_dwa_angle_resolution,
            self.obstacle_range,
        )
        core = ExternalDwaCore(self._external_dwa_config())
        result = core.plan(
            ExternalDwaPose(pose.x, pose.y, pose.yaw),
            ExternalDwaVelocity(current.vx, 0.0, current.wz),
            target,
            self.path_points[self.path_progress_index:],
            obstacles,
        )
        if result.reason == "goal_reached":
            self._publish_stop("goal reached", mode="external_dwa")
            self._publish_local_path([pose])
            return
        if not result.valid:
            self._publish_stop(f"external_dwa {result.reason}", mode="external_dwa")
            self._publish_local_path([pose])
            return
        self._publish_cmd(
            Velocity2D(result.velocity.vx, 0.0, result.velocity.wz),
            "external_dwa",
            result.reason,
        )
        self._publish_local_path(
            [Pose2D(state.x, state.y, state.yaw) for state in result.trajectory]
        )

    def _run_simple_tracker(self, pose: Pose2D, target: Tuple[float, float]) -> None:
        velocity = self._desired_velocity(pose, target)
        trajectory = self._rollout(pose, velocity)
        if self._trajectory_clearance(trajectory) <= 0.0:
            self._publish_stop("simple_tracker no collision-free trajectory", mode="simple_tracker")
            self._publish_local_path([pose])
            return
        self._publish_cmd(velocity, "simple_tracker", "ok")
        self._publish_local_path(trajectory)

    def _make_competition_core(self) -> MecanumCompetitionPlannerCore:
        preferred_side = (
            BypassSide.RIGHT
            if self.preferred_bypass_side.lower() == "right"
            else BypassSide.LEFT
        )
        max_vy = self.max_vy if self.enable_vy else 0.0
        return MecanumCompetitionPlannerCore(
            MecanumCompetitionPlannerConfig(
                max_vx=self.max_vx,
                min_vx=self.min_vx,
                max_vy=max_vy,
                max_wz=self.max_wz,
                track_vx=self.track_vx,
                bypass_vx=self.bypass_vx,
                sidestep_vy=min(self.sidestep_vy, max_vy),
                rejoin_vy=min(self.rejoin_vy, max_vy),
                heading_gain=self.heading_gain,
                lateral_gain=self.lateral_gain,
                robot_length=self.robot_length,
                robot_width=self.robot_width,
                obstacle_margin=self.obstacle_margin,
                front_check_distance=self.front_check_distance,
                front_stop_distance=self.front_stop_distance,
                side_check_distance=self.side_check_distance,
                side_min_clearance=self.side_min_clearance,
                max_path_deviation=self.max_path_deviation,
                preferred_bypass_side=preferred_side,
                obstacle_confirm_frames=self.obstacle_confirm_frames,
                obstacle_clear_frames=self.obstacle_clear_frames,
                min_state_duration=self.min_state_duration,
                side_switch_cooldown=self.side_switch_cooldown,
                rejoin_tolerance=self.rejoin_tolerance,
                goal_approach_distance=self.goal_approach_distance,
                xy_goal_tolerance=self.xy_goal_tolerance,
                goal_max_vx=self.goal_max_vx,
                goal_max_vy=min(self.goal_max_vy, max_vy),
                goal_max_wz=self.goal_max_wz,
                control_period=1.0 / max(1.0, self.control_frequency),
                sim_time=self.sim_time,
                dt=self.sim_dt,
            )
        )

    def _external_dwa_config(self) -> ExternalDwaConfig:
        robot_radius = 0.5 * math.hypot(self.robot_length, self.robot_width)
        return ExternalDwaConfig(
            min_vx=self.min_vx,
            max_vx=self.max_vx,
            max_wz=self.max_wz,
            acc_lim_x=self.acc_lim_x,
            acc_lim_theta=self.acc_lim_theta,
            vx_samples=self.vx_samples,
            wz_samples=self.wz_samples,
            sim_time=self.sim_time,
            dt=self.sim_dt,
            robot_radius=robot_radius,
            obstacle_margin=self.obstacle_margin,
            obstacle_range=self.obstacle_range,
            goal_tolerance=self.goal_tolerance,
            target_velocity=min(self.target_velocity, self.max_vx),
            angle_resolution=self.external_dwa_angle_resolution,
            angle_to_goal_turn_threshold=self.external_dwa_angle_to_goal_turn_threshold,
            min_in_place_wz=self.external_dwa_min_in_place_wz,
            max_in_place_wz=min(self.external_dwa_max_in_place_wz, self.max_wz),
            weight_path_distance=self.weight_path_distance,
            weight_target_distance=self.weight_target_distance,
            weight_heading=self.weight_heading,
            weight_obstacle=self.weight_obstacle,
            weight_smoothness=self.weight_smoothness,
            weight_speed=self.weight_speed,
            enable_vy=False,
        )

    def _not_ready_reason(self) -> str:
        now = self.get_clock().now()
        if self.odom is None or self.last_odom_time is None:
            return "waiting for odom"
        if self.path_points == [] or self.last_path_time is None:
            return "waiting for global_path"
        if self.scan is None or self.last_scan_time is None:
            return "waiting for scan"
        if self._age_seconds(now, self.last_odom_time) > self.odom_timeout:
            return "stale odom"
        if (
            self.path_timeout > 0.0
            and self._age_seconds(now, self.last_path_time) > self.path_timeout
        ):
            return "stale global_path"
        if (
            self.require_fresh_scan
            and self.scan_timeout > 0.0
            and self._age_seconds(now, self.last_scan_time) > self.scan_timeout
        ):
            return "stale scan"
        return ""

    @staticmethod
    def _age_seconds(now: Any, then: Any) -> float:
        return (now - then).nanoseconds * 1e-9

    def _current_pose(self) -> Optional[Pose2D]:
        if self.odom is None:
            return None
        pose = self.odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        if not (
            math.isfinite(pose.position.x)
            and math.isfinite(pose.position.y)
            and math.isfinite(yaw)
        ):
            return None
        return Pose2D(pose.position.x, pose.position.y, yaw)

    def _current_velocity(self) -> Velocity2D:
        if self.odom is None:
            return Velocity2D(0.0, 0.0, 0.0)
        twist = self.odom.twist.twist
        velocity = Velocity2D(twist.linear.x, twist.linear.y, twist.angular.z)
        if not (
            math.isfinite(velocity.vx)
            and math.isfinite(velocity.vy)
            and math.isfinite(velocity.wz)
        ):
            return Velocity2D(0.0, 0.0, 0.0)
        return velocity

    def _planner_velocity_feedback(self) -> Velocity2D:
        if self.velocity_feedback_mode == "odom":
            return self._current_velocity()
        return self.last_command

    def _select_lookahead_point(self, pose: Pose2D) -> Optional[Tuple[float, float]]:
        if not self.path_points:
            return None
        nearest_index = self._nearest_forward_path_index(pose)
        self.path_progress_index = max(self.path_progress_index, nearest_index)

        distance = 0.0
        previous = self.path_points[self.path_progress_index]
        for point in self.path_points[self.path_progress_index + 1:]:
            distance += self._distance_xy(previous, point)
            if distance >= self.lookahead_distance:
                return point
            previous = point
        return self.path_points[-1]

    def _nearest_forward_path_index(self, pose: Pose2D) -> int:
        start_index = min(self.path_progress_index, len(self.path_points) - 1)
        return min(
            range(start_index, len(self.path_points)),
            key=lambda i: self._distance_xy((pose.x, pose.y), self.path_points[i]),
        )

    def _desired_velocity(self, pose: Pose2D, target: Tuple[float, float]) -> Velocity2D:
        dx = target[0] - pose.x
        dy = target[1] - pose.y
        cos_yaw = math.cos(pose.yaw)
        sin_yaw = math.sin(pose.yaw)
        target_x_body = cos_yaw * dx + sin_yaw * dy
        target_y_body = -sin_yaw * dx + cos_yaw * dy
        heading_error = math.atan2(target_y_body, target_x_body)
        vy = clamp(self.k_lateral * target_y_body, -self.max_vy, self.max_vy)
        if not self.enable_vy:
            vy = 0.0
        return Velocity2D(
            clamp(self.k_linear * target_x_body, self.min_vx, self.max_vx),
            vy,
            clamp(self.k_angular * heading_error, -self.max_wz, self.max_wz),
        )

    def _rollout(self, start: Pose2D, velocity: Velocity2D) -> List[Pose2D]:
        trajectory = [start]
        pose = start
        steps = max(1, int(math.ceil(self.sim_time / max(0.02, self.sim_dt))))
        for _ in range(steps):
            cos_yaw = math.cos(pose.yaw)
            sin_yaw = math.sin(pose.yaw)
            pose = Pose2D(
                pose.x + (cos_yaw * velocity.vx - sin_yaw * velocity.vy) * self.sim_dt,
                pose.y + (sin_yaw * velocity.vx + cos_yaw * velocity.vy) * self.sim_dt,
                normalize_angle(pose.yaw + velocity.wz * self.sim_dt),
            )
            trajectory.append(pose)
        return trajectory

    def _front_obstacle_too_close(self) -> bool:
        scan = self._active_scan()
        if scan is None:
            return False
        half_angle = 0.5 * self.front_stop_angle
        angle = scan.angle_min
        min_front = math.inf
        for value in scan.ranges:
            if abs(normalize_angle(angle)) <= half_angle and finite_range(value, scan):
                min_front = min(min_front, value)
            angle += scan.angle_increment
        return min_front <= max(0.0, self.hard_stop_distance_m)

    def _active_scan(self, now: Any = None) -> Optional[LaserScan]:
        if self.scan is None or self.last_scan_time is None:
            return None
        now = now if now is not None else self.get_clock().now()
        if (
            self.scan_timeout > 0.0
            and self._age_seconds(now, self.last_scan_time) > self.scan_timeout
        ):
            return None
        return self.scan

    def _scan_obstacles(self, scan: LaserScan) -> List[Tuple[float, float]]:
        return scan_points_from_ranges(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            self.external_dwa_angle_resolution,
            self.obstacle_range,
        )

    def _trajectory_clearance(self, trajectory: Sequence[Pose2D]) -> float:
        scan = self._active_scan()
        if scan is None:
            return math.inf
        start = trajectory[0]
        radius = 0.5 * math.hypot(self.robot_length, self.robot_width) + self.obstacle_margin
        min_clearance = math.inf
        cos_start = math.cos(start.yaw)
        sin_start = math.sin(start.yaw)
        for pose in trajectory[1:]:
            dx = pose.x - start.x
            dy = pose.y - start.y
            x_body = cos_start * dx + sin_start * dy
            y_body = -sin_start * dx + cos_start * dy
            center_distance = math.hypot(x_body, y_body)
            if center_distance < 0.02:
                continue
            measured = self._scan_range_at(math.atan2(y_body, x_body), scan)
            if measured is None:
                continue
            min_clearance = min(min_clearance, measured - center_distance - radius)
        return min_clearance

    @staticmethod
    def _scan_range_at(angle: float, scan: LaserScan) -> Optional[float]:
        if scan.angle_increment == 0.0:
            return None
        index = int(round((angle - scan.angle_min) / scan.angle_increment))
        if index < 0 or index >= len(scan.ranges):
            return None
        value = scan.ranges[index]
        if not finite_range(value, scan):
            return None
        return value

    @staticmethod
    def _distance_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _publish_cmd(
        self,
        velocity: Velocity2D,
        mode: str,
        reason: str,
        stopped: bool = False,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        msg = Twist()
        msg.linear.x = clamp(velocity.vx, self.min_vx, self.max_vx)
        if self.enable_vy:
            msg.linear.y = clamp(velocity.vy, -self.max_vy, self.max_vy)
        msg.angular.z = clamp(velocity.wz, -self.max_wz, self.max_wz)
        self.cmd_pub.publish(msg)
        self.last_command = Velocity2D(msg.linear.x, msg.linear.y, msg.angular.z)
        self._publish_state(mode, reason, stopped=stopped, extra=extra)
        self.last_status_reason = ""

    def _publish_stop(
        self,
        reason: str,
        mode: Optional[str] = None,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        if reason != self.last_status_reason:
            self.get_logger().info(f"Publishing zero cmd_vel_raw: {reason}")
            self.last_status_reason = reason
        self.cmd_pub.publish(Twist())
        self.last_command = Velocity2D(0.0, 0.0, 0.0)
        self._publish_state(
            mode or self._selected_planner_mode(),
            reason,
            stopped=True,
            extra=extra,
        )

    def _publish_state(
        self,
        mode: str,
        reason: str,
        stopped: bool,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        payload: Dict[str, Any] = {
            "schema": "robot_planner.planner_state.string.v1",
            "node": self.get_name(),
            "mode": self.planner_mode,
            "active_mode": mode,
            "fallback_planner_mode": self.fallback_planner_mode,
            "stopped": stopped,
            "reason": reason,
        }
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.state_pub.publish(msg)

    def _publish_local_path(self, trajectory: Sequence[Pose2D]) -> None:
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.path_frame
        for pose in trajectory:
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose.position.x = pose.x
            pose_msg.pose.position.y = pose.y
            qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw
            msg.poses.append(pose_msg)
        self.local_path_pub.publish(msg)

    def _competition_reason(self, result: Any, default: str) -> str:
        status = getattr(result, "planner_state", None)
        return str(getattr(status, "reason", default))

    def _competition_status_dict(self, result: Any) -> Dict[str, Any]:
        status = getattr(result, "planner_state", None)
        if status is None:
            return {}
        state = getattr(status, "state", None)
        active_side = getattr(status, "active_side", None)
        return {
            "core_state": getattr(state, "value", str(state)),
            "active_side": getattr(active_side, "value", str(active_side)),
            "obstacle_confirm_count": getattr(status, "obstacle_confirm_count", 0),
            "obstacle_clear_count": getattr(status, "obstacle_clear_count", 0),
            "state_age": round(float(getattr(status, "state_age", 0.0)), 3),
            "side_switch_age": round(float(getattr(status, "side_switch_age", 0.0)), 3),
        }

    @staticmethod
    def _is_zero_velocity(velocity: Any) -> bool:
        return (
            abs(float(getattr(velocity, "vx", 0.0))) < 1e-6
            and abs(float(getattr(velocity, "vy", 0.0))) < 1e-6
            and abs(float(getattr(velocity, "wz", 0.0))) < 1e-6
        )


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = MecanumCompetitionPlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
