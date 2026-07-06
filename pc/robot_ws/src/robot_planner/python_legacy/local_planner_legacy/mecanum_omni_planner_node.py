#!/usr/bin/env python3

import json
import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from local_planner.external_dwa_adapter import (
    ExternalDwaConfig,
    ExternalDwaCore,
    ExternalDwaPose,
    ExternalDwaVelocity,
    scan_points_from_ranges,
)
from local_planner.footprint_collision_checker import FootprintConfig
from local_planner.mecanum_omni_core import MecanumOmniConfig, MecanumOmniPlanner
from local_planner.obstacle_processor import scan_to_obstacle_points
from local_planner.trajectory_evaluator import CostWeights
from local_planner.trajectory_sampler import (
    Pose2D as CorePose2D,
    SamplingConfig,
    Velocity2D as CoreVelocity2D,
)


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
class Candidate:
    velocity: Velocity2D
    trajectory: List[Pose2D]
    score: float


def choose_velocity_feedback(
    mode: str,
    odom_velocity: Velocity2D,
    last_command: Velocity2D,
) -> Velocity2D:
    if mode == "odom":
        return odom_velocity
    if mode == "last_command":
        return last_command
    return last_command


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


class MecanumOmniPlannerNode(Node):
    """ROS2 adapter for robot1 mecanum local planning.

    The planner core owns no ROS2 publishers or subscriptions. This adapter is
    the only place that subscribes robot1 inputs and publishes raw planner
    outputs.
    """

    VALID_MODES = {"simple_tracker", "external_dwa", "mecanum_omni"}

    def __init__(self) -> None:
        super().__init__("robot1_mecanum_omni_planner")

        self.global_path_topic = self._declare_str("global_path_topic", "/robot1/global_path")
        self.odom_topic = self._declare_str("odom_topic", "/robot1/odom")
        self.scan_topic = self._declare_str("scan_topic", "/robot1/scan")
        self.local_path_topic = self._declare_str("local_path_topic", "/robot1/local_path")
        self.cmd_vel_raw_topic = self._declare_str("cmd_vel_raw_topic", "/robot1/cmd_vel_raw")
        self.planner_state_topic = self._declare_str("planner_state_topic", "/robot1/planner_state")

        self.planner_mode = self._declare_str("planner_mode", "mecanum_omni")
        self.fallback_planner_mode = self._declare_str("fallback_planner_mode", "simple_tracker")
        self.enable_lateral_motion = self._declare_bool("enable_lateral_motion", True)
        self.enable_vy = self._declare_bool("enable_vy", self.enable_lateral_motion)

        self.control_frequency = self._declare_float("control_frequency", 10.0)
        self.lookahead_distance = self._declare_float("lookahead_distance", 0.45)
        self.goal_tolerance = self._declare_float("goal_tolerance", 0.12)
        self.path_timeout = self._declare_float("path_timeout", 0.0)
        self.odom_timeout = self._declare_float("odom_timeout", 0.5)
        self.scan_timeout = self._declare_float("scan_timeout", 0.5)
        self.require_fresh_scan = self._declare_bool("require_fresh_scan", False)
        self.velocity_feedback_mode = self._declare_str(
            "velocity_feedback_mode", "last_command"
        )

        self.min_vx = self._declare_float("min_vx", 0.0)
        self.max_vx = self._declare_float("max_vx", 0.25)
        self.min_vy = self._declare_float("min_vy", -0.20)
        self.max_vy = self._declare_float("max_vy", 0.20)
        self.max_wz = self._declare_float("max_wz", 0.80)
        self.acc_lim_x = self._declare_float("acc_lim_x", 0.40)
        self.acc_lim_y = self._declare_float("acc_lim_y", 0.40)
        self.acc_lim_theta = self._declare_float("acc_lim_theta", 1.00)

        self.vx_samples = self._declare_int("vx_samples", 5)
        self.vy_samples = self._declare_int("vy_samples", 5)
        self.wz_samples = self._declare_int("wz_samples", 7)
        self.sim_time = self._declare_float("sim_time", 1.5)
        self.sim_dt = self._declare_float("dt", 0.1)
        self.use_dynamic_window = self._declare_bool("use_dynamic_window", True)
        self.target_velocity = self._declare_float("target_velocity", self.max_vx)

        self.robot_length = self._declare_float("robot_length", 0.24)
        self.robot_width = self._declare_float("robot_width", 0.20)
        self.obstacle_margin = self._declare_float("obstacle_margin", 0.05)
        self.obstacle_range = self._declare_float("obstacle_range", 2.5)
        self.front_stop_distance = self._declare_float("front_stop_distance", 0.35)
        self.front_stop_angle = self._declare_float("front_stop_angle", 0.70)
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

        self.k_linear = self._declare_float("k_linear", 0.75)
        self.k_lateral = self._declare_float("k_lateral", 0.75)
        self.k_angular = self._declare_float("k_angular", 1.20)

        self.weight_path_distance = self._declare_float("weight_path_distance", 2.0)
        self.weight_target_distance = self._declare_float("weight_target_distance", 3.0)
        self.weight_heading = self._declare_float("weight_heading", 0.8)
        self.weight_obstacle = self._declare_float("weight_obstacle", 0.6)
        self.weight_lateral_velocity = self._declare_float("weight_lateral_velocity", 0.2)
        self.weight_smoothness = self._declare_float("weight_smoothness", 0.4)
        self.weight_speed = self._declare_float("weight_speed", 0.15)
        self.weight_velocity = self._declare_float("weight_velocity", self.weight_speed)
        self.weight_oscillation = self._declare_float("weight_oscillation", 0.8)
        self.obstacle_angle_stride = self._declare_int("obstacle_angle_stride", 1)

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
        self._mecanum_core = self._make_mecanum_core()

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
        reason = self._not_ready_reason()
        if reason:
            self._publish_stop(reason)
            return

        pose = self._current_pose()
        if pose is None:
            self._publish_stop("waiting for valid odom pose")
            return

        if self._front_obstacle_too_close():
            self._publish_stop("front obstacle inside local stop distance")
            return

        target = self._select_lookahead_point(pose)
        if target is None:
            self._publish_stop("empty global path")
            return

        if self._is_goal_reached(pose):
            self._publish_stop("goal reached")
            self._publish_local_path([pose])
            return

        mode = self._selected_planner_mode()
        try:
            if mode == "mecanum_omni":
                if self._run_mecanum_omni(pose, target):
                    return
                mode = self._selected_fallback_mode()
            if mode == "external_dwa":
                self._run_external_dwa(pose, target)
                return
            self._run_simple_tracker(pose, target)
        except Exception as exc:
            self._publish_stop(f"{mode} adapter error: {exc}")
            self._publish_local_path([pose])

    def _selected_planner_mode(self) -> str:
        if self.planner_mode in self.VALID_MODES:
            return self.planner_mode
        return self._selected_fallback_mode()

    def _selected_fallback_mode(self) -> str:
        if self.fallback_planner_mode in ("simple_tracker", "external_dwa"):
            return self.fallback_planner_mode
        return "simple_tracker"

    def _run_mecanum_omni(self, pose: Pose2D, target: Tuple[float, float]) -> bool:
        if self._mecanum_core is None:
            self._publish_state(
                "fallback",
                "mecanum_omni core unavailable",
                active_mode=self._selected_fallback_mode(),
            )
            return False

        scan = self._active_scan()
        obstacles = self._scan_obstacles(scan) if scan is not None else []
        result = self._call_mecanum_core(
            pose,
            self._planner_velocity_feedback(),
            target,
            self.path_points[self.path_progress_index:],
            obstacles,
        )
        if result is None:
            self._publish_stop("mecanum_omni returned no result")
            self._publish_local_path([pose])
            return True

        valid = bool(self._result_field(result, "valid", True))
        reason = str(self._result_field(result, "reason", "ok"))
        if not valid:
            self._publish_stop(f"mecanum_omni {reason}")
            self._publish_local_path([pose])
            return True

        velocity = self._extract_velocity(result)
        trajectory = self._extract_trajectory(result, pose)
        if velocity is None or not trajectory:
            self._publish_stop("mecanum_omni no valid trajectory")
            self._publish_local_path([pose])
            return True

        self._publish_cmd(velocity, "mecanum_omni", reason)
        self._publish_local_path(trajectory)
        return True

    def _make_mecanum_core(self) -> Optional[Any]:
        allow_lateral = self.enable_lateral_motion and self.enable_vy
        config = MecanumOmniConfig(
            sampling=SamplingConfig(
                min_vx=self.min_vx,
                max_vx=self.max_vx,
                min_vy=self.min_vy if allow_lateral else 0.0,
                max_vy=self.max_vy if allow_lateral else 0.0,
                max_wz=self.max_wz,
                acc_lim_x=self.acc_lim_x,
                acc_lim_y=self.acc_lim_y,
                acc_lim_theta=self.acc_lim_theta,
                vx_samples=self.vx_samples,
                vy_samples=self.vy_samples if allow_lateral else 1,
                wz_samples=self.wz_samples,
                sim_time=self.sim_time,
                dt=self.sim_dt,
                control_period=1.0 / max(1.0, self.control_frequency),
                use_dynamic_window=self.use_dynamic_window,
            ),
            footprint=FootprintConfig(
                robot_length=self.robot_length,
                robot_width=self.robot_width,
                obstacle_margin=self.obstacle_margin,
            ),
            weights=CostWeights(
                path=self.weight_path_distance,
                goal=self.weight_target_distance,
                obstacle=self.weight_obstacle,
                heading=self.weight_heading,
                velocity=self.weight_velocity,
                smoothness=self.weight_smoothness,
                lateral=self.weight_lateral_velocity,
                oscillation=self.weight_oscillation,
            ),
            goal_tolerance=self.goal_tolerance,
            obstacle_range=self.obstacle_range,
            target_velocity=min(self.target_velocity, self.max_vx),
        )
        return MecanumOmniPlanner(config)

    def _call_mecanum_core(
        self,
        pose: Pose2D,
        velocity: Velocity2D,
        target: Tuple[float, float],
        path_points: Sequence[Tuple[float, float]],
        obstacles: Sequence[Tuple[float, float]],
    ) -> Any:
        plan = getattr(self._mecanum_core, "plan")
        return plan(
            pose=CorePose2D(pose.x, pose.y, pose.yaw),
            current_velocity=CoreVelocity2D(velocity.vx, velocity.vy, velocity.wz),
            path_points=path_points,
            goal=target,
            obstacles_robot_frame=obstacles,
            previous_command=CoreVelocity2D(
                self.last_command.vx,
                self.last_command.vy,
                self.last_command.wz,
            ),
        )

    def _core_config(self) -> Dict[str, Any]:
        return {
            "enable_lateral_motion": self.enable_lateral_motion and self.enable_vy,
            "min_vx": self.min_vx,
            "max_vx": self.max_vx,
            "min_vy": self.min_vy,
            "max_vy": self.max_vy,
            "max_wz": self.max_wz,
            "acc_lim_x": self.acc_lim_x,
            "acc_lim_y": self.acc_lim_y,
            "acc_lim_theta": self.acc_lim_theta,
            "vx_samples": self.vx_samples,
            "vy_samples": self.vy_samples,
            "wz_samples": self.wz_samples,
            "sim_time": self.sim_time,
            "dt": self.sim_dt,
            "robot_length": self.robot_length,
            "robot_width": self.robot_width,
            "obstacle_margin": self.obstacle_margin,
            "obstacle_range": self.obstacle_range,
            "goal_tolerance": self.goal_tolerance,
            "target_velocity": self.target_velocity,
            "weight_path_distance": self.weight_path_distance,
            "weight_target_distance": self.weight_target_distance,
            "weight_heading": self.weight_heading,
            "weight_obstacle": self.weight_obstacle,
            "weight_lateral_velocity": self.weight_lateral_velocity,
            "weight_smoothness": self.weight_smoothness,
            "weight_speed": self.weight_speed,
        }

    def _run_external_dwa(self, pose: Pose2D, target: Tuple[float, float]) -> None:
        scan = self._active_scan()
        if scan is None:
            self._publish_stop("external_dwa waiting for fresh scan")
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
            self._publish_stop("goal reached")
            self._publish_local_path([pose])
            return
        if not result.valid:
            self._publish_stop(f"external_dwa {result.reason}")
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

    def _run_simple_tracker(self, pose: Pose2D, target: Tuple[float, float]) -> None:
        desired = self._desired_velocity(pose, target)
        current = self._planner_velocity_feedback()
        candidates = self._generate_candidates(pose, target, desired, current)
        if not candidates:
            self._publish_stop("simple_tracker no valid trajectory")
            self._publish_local_path([pose])
            return
        best = min(candidates, key=lambda c: c.score)
        self._publish_cmd(best.velocity, "simple_tracker", "ok")
        self._publish_local_path(best.trajectory)

    def _not_ready_reason(self) -> str:
        now = self.get_clock().now()
        if self.odom is None or self.last_odom_time is None:
            return "waiting for odom"
        if self.path_points == [] or self.last_path_time is None:
            return "waiting for global_path"
        if self._age_seconds(now, self.last_odom_time) > self.odom_timeout:
            return "stale odom"
        if (
            self.path_timeout > 0.0
            and self._age_seconds(now, self.last_path_time) > self.path_timeout
        ):
            return "stale global_path"
        if self.require_fresh_scan and self._active_scan(now) is None:
            if self.scan is None or self.last_scan_time is None:
                return "waiting for scan"
            return "stale scan"
        return ""

    @staticmethod
    def _age_seconds(now: Any, then: Any) -> float:
        return (now - then).nanoseconds * 1e-9

    def _current_pose(self) -> Optional[Pose2D]:
        if self.odom is None:
            return None
        pose = self.odom.pose.pose
        return Pose2D(
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
        )

    def _current_velocity(self) -> Velocity2D:
        if self.odom is None:
            return Velocity2D(0.0, 0.0, 0.0)
        twist = self.odom.twist.twist
        return Velocity2D(twist.linear.x, twist.linear.y, twist.angular.z)

    def _planner_velocity_feedback(self) -> Velocity2D:
        return choose_velocity_feedback(
            self.velocity_feedback_mode,
            self._current_velocity(),
            self.last_command,
        )

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

    def _is_goal_reached(self, pose: Pose2D) -> bool:
        return bool(
            self.path_points
            and self._distance_xy((pose.x, pose.y), self.path_points[-1])
            <= self.goal_tolerance
        )

    def _desired_velocity(self, pose: Pose2D, target: Tuple[float, float]) -> Velocity2D:
        dx = target[0] - pose.x
        dy = target[1] - pose.y
        cos_yaw = math.cos(pose.yaw)
        sin_yaw = math.sin(pose.yaw)
        target_x_body = cos_yaw * dx + sin_yaw * dy
        target_y_body = -sin_yaw * dx + cos_yaw * dy
        heading_error = math.atan2(target_y_body, target_x_body)
        vy = clamp(self.k_lateral * target_y_body, self.min_vy, self.max_vy)
        if not (self.enable_lateral_motion and self.enable_vy):
            vy = 0.0
        return Velocity2D(
            clamp(self.k_linear * target_x_body, self.min_vx, self.max_vx),
            vy,
            clamp(self.k_angular * heading_error, -self.max_wz, self.max_wz),
        )

    def _generate_candidates(
        self,
        start: Pose2D,
        target: Tuple[float, float],
        desired: Velocity2D,
        current: Velocity2D,
    ) -> List[Candidate]:
        candidates: List[Candidate] = []
        for velocity in self._sample_velocity_space(desired, current):
            trajectory = self._rollout(start, velocity)
            score = self._score_trajectory(trajectory, velocity, target, current)
            if math.isfinite(score):
                candidates.append(Candidate(velocity, trajectory, score))
        return candidates

    def _sample_velocity_space(
        self,
        desired: Velocity2D,
        current: Velocity2D,
    ) -> Iterable[Velocity2D]:
        vx_values = self._sample_axis(
            self.min_vx,
            self.max_vx,
            current.vx,
            self.acc_lim_x,
            desired.vx,
            self.vx_samples,
        )
        if self.enable_lateral_motion and self.enable_vy:
            vy_values = self._sample_axis(
                self.min_vy,
                self.max_vy,
                current.vy,
                self.acc_lim_y,
                desired.vy,
                self.vy_samples,
            )
        else:
            vy_values = [0.0]
        wz_values = self._sample_axis(
            -self.max_wz,
            self.max_wz,
            current.wz,
            self.acc_lim_theta,
            desired.wz,
            self.wz_samples,
        )
        for vx in vx_values:
            for vy in vy_values:
                for wz in wz_values:
                    yield Velocity2D(vx, vy, wz)

    def _sample_axis(
        self,
        absolute_min: float,
        absolute_max: float,
        current: float,
        acc_limit: float,
        desired: float,
        count: int,
    ) -> List[float]:
        lower = absolute_min
        upper = absolute_max
        if self.use_dynamic_window:
            current = clamp(current, absolute_min, absolute_max)
            period = 1.0 / max(1.0, self.control_frequency)
            lower = max(absolute_min, current - acc_limit * period)
            upper = min(absolute_max, current + acc_limit * period)

        values = {0.0, clamp(desired, lower, upper), lower, upper}
        if count > 1 and upper > lower:
            step = (upper - lower) / float(count - 1)
            for index in range(count):
                values.add(lower + step * index)
        return sorted(round(value, 4) for value in values)

    def _rollout(self, start: Pose2D, velocity: Velocity2D) -> List[Pose2D]:
        trajectory = [start]
        pose = start
        steps = max(1, int(self.sim_time / max(0.02, self.sim_dt)))
        for _ in range(steps):
            cos_yaw = math.cos(pose.yaw)
            sin_yaw = math.sin(pose.yaw)
            x_dot = cos_yaw * velocity.vx - sin_yaw * velocity.vy
            y_dot = sin_yaw * velocity.vx + cos_yaw * velocity.vy
            pose = Pose2D(
                pose.x + x_dot * self.sim_dt,
                pose.y + y_dot * self.sim_dt,
                normalize_angle(pose.yaw + velocity.wz * self.sim_dt),
            )
            trajectory.append(pose)
        return trajectory

    def _score_trajectory(
        self,
        trajectory: Sequence[Pose2D],
        velocity: Velocity2D,
        target: Tuple[float, float],
        current: Velocity2D,
    ) -> float:
        clearance = self._trajectory_clearance(trajectory)
        if clearance <= 0.0:
            return math.inf
        end = trajectory[-1]
        path_distance = self._distance_to_path(end)
        target_distance = self._distance_xy((end.x, end.y), target)
        target_heading = math.atan2(target[1] - end.y, target[0] - end.x)
        heading_error = abs(normalize_angle(target_heading - end.yaw))
        obstacle_cost = 0.0 if math.isinf(clearance) else 1.0 / max(0.01, clearance)
        speed = math.hypot(velocity.vx, velocity.vy)
        smoothness = (
            abs(velocity.vx - current.vx)
            + abs(velocity.vy - current.vy)
            + 0.5 * abs(velocity.wz - current.wz)
        )
        return (
            self.weight_path_distance * path_distance
            + self.weight_target_distance * target_distance
            + self.weight_heading * heading_error
            + self.weight_obstacle * obstacle_cost
            + self.weight_lateral_velocity * abs(velocity.vy)
            + self.weight_smoothness * smoothness
            - self.weight_speed * speed
        )

    def _distance_to_path(self, pose: Pose2D) -> float:
        if not self.path_points:
            return 0.0
        start_index = max(0, min(self.path_progress_index, len(self.path_points) - 1) - 1)
        return min(
            self._distance_xy((pose.x, pose.y), path_point)
            for path_point in self.path_points[start_index:]
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
        return min_front <= self.front_stop_distance

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

    def _scan_obstacles(self, scan: Optional[LaserScan]) -> List[Tuple[float, float]]:
        if scan is None:
            return []
        return scan_to_obstacle_points(
            ranges=scan.ranges,
            angle_min=scan.angle_min,
            angle_increment=scan.angle_increment,
            range_min=scan.range_min,
            range_max=scan.range_max,
            obstacle_range=self.obstacle_range,
            angle_stride=self.obstacle_angle_stride,
        )

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

    def _publish_cmd(self, velocity: Velocity2D, mode: str, reason: str) -> None:
        msg = Twist()
        msg.linear.x = clamp(velocity.vx, self.min_vx, self.max_vx)
        if self.enable_lateral_motion and self.enable_vy:
            msg.linear.y = clamp(velocity.vy, self.min_vy, self.max_vy)
        msg.angular.z = clamp(velocity.wz, -self.max_wz, self.max_wz)
        self.cmd_pub.publish(msg)
        self.last_command = Velocity2D(msg.linear.x, msg.linear.y, msg.angular.z)
        self._publish_state(mode, reason, stopped=False)
        self.last_status_reason = ""

    def _publish_stop(self, reason: str) -> None:
        if reason != self.last_status_reason:
            self.get_logger().info(f"Publishing zero cmd_vel_raw: {reason}")
            self.last_status_reason = reason
        self.cmd_pub.publish(Twist())
        self.last_command = Velocity2D(0.0, 0.0, 0.0)
        self._publish_state(self._selected_planner_mode(), reason, stopped=True)

    def _publish_state(
        self,
        mode: str,
        reason: str,
        stopped: bool = True,
        active_mode: Optional[str] = None,
    ) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "schema": "robot_planner.planner_state.string.v1",
                "node": self.get_name(),
                "mode": mode,
                "active_mode": active_mode or mode,
                "stopped": stopped,
                "reason": reason,
            },
            sort_keys=True,
        )
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

    def _result_field(self, result: Any, name: str, default: Any) -> Any:
        if isinstance(result, dict):
            return result.get(name, default)
        return getattr(result, name, default)

    def _extract_velocity(self, result: Any) -> Optional[Velocity2D]:
        velocity = self._result_field(result, "best_cmd", None)
        if velocity is None:
            velocity = self._result_field(result, "velocity", None)
        if velocity is None:
            command = self._result_field(result, "command", None)
            velocity = command if command is not None else result
        vx = self._number_field(velocity, "vx", "linear_x", "x")
        vy = self._number_field(velocity, "vy", "linear_y", "y", default=0.0)
        wz = self._number_field(velocity, "wz", "angular_z", "theta")
        if vx is None or vy is None or wz is None:
            return None
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz)):
            return None
        return Velocity2D(vx, vy, wz)

    def _extract_trajectory(self, result: Any, fallback_pose: Pose2D) -> List[Pose2D]:
        raw_trajectory = self._result_field(result, "best_trajectory", None)
        if not raw_trajectory:
            raw_trajectory = self._result_field(result, "trajectory", None)
        if not raw_trajectory:
            raw_trajectory = self._result_field(result, "local_path", None)
        if not raw_trajectory:
            return [fallback_pose]
        trajectory: List[Pose2D] = []
        for state in raw_trajectory:
            x = self._number_field(state, "x", "pose_x")
            y = self._number_field(state, "y", "pose_y")
            yaw = self._number_field(state, "yaw", "theta", default=fallback_pose.yaw)
            if x is None or y is None or yaw is None:
                continue
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw):
                trajectory.append(Pose2D(x, y, yaw))
        return trajectory

    @staticmethod
    def _number_field(obj: Any, *names: str, default: Any = None) -> Optional[float]:
        for name in names:
            if isinstance(obj, dict) and name in obj:
                return float(obj[name])
            if hasattr(obj, name):
                return float(getattr(obj, name))
        return default


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = MecanumOmniPlannerNode()
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
