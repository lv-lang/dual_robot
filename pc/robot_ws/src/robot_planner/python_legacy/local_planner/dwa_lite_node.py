#!/usr/bin/env python3

import importlib
import inspect
import json
import math
from dataclasses import dataclass
from types import SimpleNamespace
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

from local_planner.scan_obstacle_model import downsample_obstacle_points


ROBOT1_GLOBAL_PATH_TOPIC = "/robot1/global_path"
ROBOT1_ODOM_TOPIC = "/robot1/odom"
ROBOT1_SCAN_TOPIC = "/robot1/scan"
ROBOT1_CMD_VEL_TOPIC = "/robot1/cmd_vel"
ROBOT1_LOCAL_PATH_TOPIC = "/robot1/local_path"
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


class DwaLitePlannerNode(Node):
    """ROS2 adapter for the robot1 DWA_lite direct local planner.

    This node owns ROS2 subscriptions and publications. The trajectory sampling,
    scoring, collision checking, and smoothing must live inside DWA_lite core.
    """

    def __init__(self) -> None:
        super().__init__("robot1_dwa_lite_planner")

        self.global_path_topic = self._declare_robot1_topic(
            "global_path_topic", ROBOT1_GLOBAL_PATH_TOPIC
        )
        self.odom_topic = self._declare_robot1_topic("odom_topic", ROBOT1_ODOM_TOPIC)
        self.scan_topic = self._declare_robot1_topic("scan_topic", ROBOT1_SCAN_TOPIC)
        self.cmd_vel_topic = self._declare_fixed_output_topic(
            "cmd_vel_topic", ROBOT1_CMD_VEL_TOPIC
        )
        self.local_path_topic = self._declare_fixed_output_topic(
            "local_path_topic", ROBOT1_LOCAL_PATH_TOPIC
        )
        self.planner_state_topic = self._declare_fixed_output_topic(
            "planner_state_topic", ROBOT1_PLANNER_STATE_TOPIC
        )

        self.control_frequency = self._declare_float("control_frequency", 15.0)
        self.path_timeout = self._declare_float("path_timeout", 0.0)
        self.odom_timeout = self._declare_float("odom_timeout", 0.5)
        self.scan_timeout = self._declare_float("scan_timeout", 0.5)
        self.lookahead_distance = self._declare_float("lookahead_distance", 0.55)
        self.heading_lookahead = self._declare_float(
            "heading_lookahead", self.lookahead_distance
        )
        self.xy_goal_tolerance = self._declare_float("xy_goal_tolerance", 0.12)
        self.goal_tolerance = self._declare_float(
            "goal_tolerance", self.xy_goal_tolerance
        )
        self.rejoin_path_distance = self._declare_float("rejoin_path_distance", 0.16)
        self.max_path_deviation = self._declare_float("max_path_deviation", 0.18)
        self.velocity_feedback_mode = self._declare_str(
            "velocity_feedback_mode", "last_command"
        )

        self.robot_length = self._declare_float("robot_length", 0.24)
        self.robot_width = self._declare_float("robot_width", 0.20)
        self.safety_margin = self._declare_float("safety_margin", 0.06)

        self.min_vx = self._declare_float("min_vx", 0.0)
        self.max_vx = self._declare_float("max_vx", 0.40)
        self.min_vy = self._declare_float("min_vy", -0.30)
        self.max_vy = self._declare_float("max_vy", 0.30)
        self.min_wz = self._declare_float("min_wz", -0.9)
        self.max_wz = self._declare_float("max_wz", 0.9)

        self.acc_lim_x = self._declare_float("acc_lim_x", 0.75)
        self.acc_lim_y = self._declare_float("acc_lim_y", 0.75)
        self.acc_lim_theta = self._declare_float("acc_lim_theta", 1.40)
        self.jerk_lim_x = self._declare_float("jerk_lim_x", 4.00)
        self.jerk_lim_y = self._declare_float("jerk_lim_y", 4.00)
        self.jerk_lim_theta = self._declare_float("jerk_lim_theta", 5.00)
        self.cmd_filter_alpha = self._declare_float("cmd_filter_alpha", 0.85)
        self.vx_deadband = self._declare_float("vx_deadband", 0.01)
        self.vy_deadband = self._declare_float("vy_deadband", 0.01)
        self.wz_deadband = self._declare_float("wz_deadband", 0.02)

        self.vx_samples = self._declare_int("vx_samples", 5)
        self.vy_samples = self._declare_int("vy_samples", 5)
        self.wz_samples = self._declare_int("wz_samples", 5)
        self.sim_time = self._declare_float("sim_time", 1.2)
        self.sim_dt = self._declare_float("dt", 0.1)
        self.use_dynamic_window = self._declare_bool("use_dynamic_window", True)
        self.target_speed = self._declare_float("target_speed", 0.30)

        self.obstacle_min_dist = self._declare_float("obstacle_min_dist", 0.24)
        self.hard_stop_distance = self._declare_float("hard_stop_distance", 0.10)
        self.front_check_distance = self._declare_float("front_check_distance", 0.75)
        self.front_check_width = self._declare_float("front_check_width", 0.34)
        self.front_check_angle = self._declare_float("front_check_angle", 0.70)
        self.side_check_distance = self._declare_float("side_check_distance", 0.75)
        self.obstacle_range = self._declare_float("obstacle_range", 2.5)
        self.max_obstacle_points = self._declare_int("max_obstacle_points", 24)

        self.path_weight = self._declare_float("path_weight", 2.0)
        self.goal_weight = self._declare_float("goal_weight", 3.0)
        self.obstacle_weight = self._declare_float("obstacle_weight", 4.0)
        self.heading_weight = self._declare_float("heading_weight", 0.5)
        self.velocity_weight = self._declare_float("velocity_weight", 0.5)
        self.smoothness_weight = self._declare_float("smoothness_weight", 0.45)
        self.lateral_weight = self._declare_float("lateral_weight", 0.15)

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
        self.last_command_before_smoothing = Velocity2D(0.0, 0.0, 0.0)

        self._core_pose_type = self._optional_core_type("Pose2D")
        self._core_velocity_type = self._optional_core_type("Velocity2D")
        self._core = self._make_core()

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
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.local_path_pub = self.create_publisher(Path, self.local_path_topic, 10)
        self.state_pub = self.create_publisher(String, self.planner_state_topic, 10)

        self.get_logger().info(
            "DWA Lite direct planner is publishing /robot1/cmd_vel directly."
        )

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
        reason = self._not_ready_reason()
        if reason:
            self._publish_stop(reason)
            return

        pose = self._current_pose()
        if pose is None:
            self._publish_stop("waiting for valid odom pose")
            return

        if self._front_obstacle_too_close():
            self._publish_stop("front obstacle inside hard_stop_distance")
            self._publish_local_path([pose])
            return

        if self._is_goal_reached(pose):
            self._publish_stop("goal reached")
            self._publish_local_path([pose])
            return

        if self._core is None:
            self._publish_stop("dwa_lite core unavailable")
            self._publish_local_path([pose])
            return

        try:
            result = self._call_core(pose)
        except Exception as exc:
            self._publish_stop(f"dwa_lite core error: {exc}")
            self._publish_local_path([pose])
            return

        command = self._extract_velocity(result)
        trajectory = self._extract_trajectory(result, pose)
        valid = self._extract_valid(result)
        reason = self._extract_reason(result)
        state = self._extract_state(result, reason, valid)

        if not valid or command is None:
            self._publish_stop(reason or "dwa_lite invalid result", extra=state)
            self._publish_local_path(trajectory)
            return

        command = self._bounded_velocity(command)
        stopped = self._is_zero_velocity(command) or state.get("core_state") == "GOAL_REACHED"
        self.last_command_before_smoothing = command
        self._publish_cmd(command, reason or "ok", stopped=stopped, extra=state)
        self._publish_local_path(trajectory)

    def _not_ready_reason(self) -> str:
        now = self.get_clock().now()
        if self.odom is None or self.last_odom_time is None:
            return "waiting for odom"
        if not self.path_points or self.last_path_time is None:
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
            self.scan_timeout > 0.0
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

    def _front_obstacle_too_close(self) -> bool:
        if self.scan is None:
            return False
        half_angle = 0.5 * self.front_check_angle
        angle = self.scan.angle_min
        min_front = math.inf
        for value in self.scan.ranges:
            if abs(normalize_angle(angle)) <= half_angle and finite_range(value, self.scan):
                min_front = min(min_front, value)
            angle += self.scan.angle_increment
        return min_front <= max(0.0, self.hard_stop_distance)

    def _scan_obstacle_points(self) -> List[Tuple[float, float]]:
        if self.scan is None:
            return []
        points: List[Tuple[float, float]] = []
        angle = self.scan.angle_min
        max_range = min(
            self.obstacle_range,
            self.scan.range_max if self.scan.range_max > 0.0 else self.obstacle_range,
        )
        for value in self.scan.ranges:
            if finite_range(value, self.scan) and value <= max_range:
                points.append((value * math.cos(angle), value * math.sin(angle)))
            angle += self.scan.angle_increment
        return downsample_obstacle_points(points, self.max_obstacle_points)

    def _is_goal_reached(self, pose: Pose2D) -> bool:
        return bool(
            self.path_points
            and self._distance_xy((pose.x, pose.y), self.path_points[-1])
            <= self.xy_goal_tolerance
        )

    def _nearest_forward_path_index(self, pose: Pose2D) -> int:
        start_index = min(self.path_progress_index, len(self.path_points) - 1)
        return min(
            range(start_index, len(self.path_points)),
            key=lambda i: self._distance_xy((pose.x, pose.y), self.path_points[i]),
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

    @staticmethod
    def _distance_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _make_core(self) -> Optional[Any]:
        try:
            module = importlib.import_module("local_planner.dwa_lite_core")
        except Exception as exc:
            self.get_logger().warning(f"DWA_lite core module unavailable: {exc}")
            return None

        params = self._make_core_params()
        params_dict = self._core_params_dict()
        for class_name in (
            "DwaLiteCore",
            "DWALiteCore",
            "DwaLitePlannerCore",
            "DWALitePlannerCore",
            "DwaLitePlanner",
            "DWALitePlanner",
        ):
            core_type = getattr(module, class_name, None)
            if core_type is None:
                continue
            for args, kwargs in (
                ((params,), {}),
                ((params_dict,), {}),
                ((), params_dict),
                ((), {}),
            ):
                try:
                    return core_type(*args, **kwargs)
                except TypeError:
                    continue
        self.get_logger().warning("DWA_lite core class not found")
        return None

    def _make_core_params(self) -> Any:
        params = self._core_params_dict()
        try:
            module = importlib.import_module("local_planner.dwa_lite_params")
        except Exception:
            return SimpleNamespace(**params)

        for class_name in (
            "DwaLiteParams",
            "DWALiteParams",
            "DwaLiteConfig",
            "DWALiteConfig",
        ):
            params_type = getattr(module, class_name, None)
            if params_type is None:
                continue
            try:
                signature = inspect.signature(params_type)
                kwargs = {
                    name: params[name]
                    for name in signature.parameters.keys()
                    if name in params
                }
                return params_type(**kwargs)
            except (TypeError, ValueError):
                try:
                    return params_type(**params)
                except TypeError:
                    continue
        return SimpleNamespace(**params)

    def _core_params_dict(self) -> Dict[str, Any]:
        return {
            "control_frequency": self.control_frequency,
            "control_period": 1.0 / max(1.0, self.control_frequency),
            "heading_lookahead": self.heading_lookahead,
            "xy_goal_tolerance": self.xy_goal_tolerance,
            "goal_tolerance": self.goal_tolerance,
            "rejoin_path_distance": self.rejoin_path_distance,
            "max_path_deviation": self.max_path_deviation,
            "robot_length": self.robot_length,
            "robot_width": self.robot_width,
            "safety_margin": self.safety_margin,
            "min_vx": self.min_vx,
            "max_vx": self.max_vx,
            "min_vy": self.min_vy,
            "max_vy": self.max_vy,
            "min_wz": self.min_wz,
            "max_wz": self.max_wz,
            "acc_lim_x": self.acc_lim_x,
            "acc_lim_y": self.acc_lim_y,
            "acc_lim_theta": self.acc_lim_theta,
            "jerk_lim_x": self.jerk_lim_x,
            "jerk_lim_y": self.jerk_lim_y,
            "jerk_lim_theta": self.jerk_lim_theta,
            "cmd_filter_alpha": self.cmd_filter_alpha,
            "vx_deadband": self.vx_deadband,
            "vy_deadband": self.vy_deadband,
            "wz_deadband": self.wz_deadband,
            "vx_samples": self.vx_samples,
            "vy_samples": self.vy_samples,
            "wz_samples": self.wz_samples,
            "sim_time": self.sim_time,
            "dt": self.sim_dt,
            "use_dynamic_window": self.use_dynamic_window,
            "target_speed": self.target_speed,
            "obstacle_min_dist": self.obstacle_min_dist,
            "hard_stop_distance": self.hard_stop_distance,
            "front_check_distance": self.front_check_distance,
            "front_check_width": self.front_check_width,
            "front_check_angle": self.front_check_angle,
            "side_check_distance": self.side_check_distance,
            "obstacle_range": self.obstacle_range,
            "path_weight": self.path_weight,
            "goal_weight": self.goal_weight,
            "obstacle_weight": self.obstacle_weight,
            "heading_weight": self.heading_weight,
            "velocity_weight": self.velocity_weight,
            "smoothness_weight": self.smoothness_weight,
            "lateral_weight": self.lateral_weight,
        }

    def _optional_core_type(self, name: str) -> Optional[Any]:
        for module_name in (
            "local_planner.planner_utils",
            "local_planner.trajectory_rollout",
            "local_planner.dwa_lite_core",
        ):
            try:
                module = importlib.import_module(module_name)
            except Exception:
                continue
            value = getattr(module, name, None)
            if value is not None:
                return value
        return None

    def _make_core_pose(self, pose: Pose2D) -> Any:
        if self._core_pose_type is None:
            return pose
        for args, kwargs in (
            ((pose.x, pose.y, pose.yaw), {}),
            ((), {"x": pose.x, "y": pose.y, "yaw": pose.yaw}),
        ):
            try:
                return self._core_pose_type(*args, **kwargs)
            except TypeError:
                continue
        return pose

    def _make_core_velocity(self, velocity: Velocity2D) -> Any:
        if self._core_velocity_type is None:
            return velocity
        for args, kwargs in (
            ((velocity.vx, velocity.vy, velocity.wz), {}),
            ((), {"vx": velocity.vx, "vy": velocity.vy, "wz": velocity.wz}),
        ):
            try:
                return self._core_velocity_type(*args, **kwargs)
            except TypeError:
                continue
        return velocity

    def _call_core(self, pose: Pose2D) -> Any:
        target = self._select_lookahead_point(pose)
        if target is None:
            raise RuntimeError("empty global path")

        current_velocity = self._planner_velocity_feedback()
        values = {
            "current_pose": self._make_core_pose(pose),
            "pose": self._make_core_pose(pose),
            "current_velocity": self._make_core_velocity(current_velocity),
            "velocity": self._make_core_velocity(current_velocity),
            "previous_command": self._make_core_velocity(self.last_command),
            "global_path": self.path_points[self.path_progress_index:],
            "path": self.path_points[self.path_progress_index:],
            "path_points": self.path_points[self.path_progress_index:],
            "goal": target,
            "target": target,
            "lookahead_point": target,
            "scan_obstacle_points": self._scan_obstacle_points(),
            "obstacles": self._scan_obstacle_points(),
            "obstacles_robot_frame": self._scan_obstacle_points(),
            "params": self._make_core_params(),
            "dt": 1.0 / max(1.0, self.control_frequency),
        }
        for method_name in ("plan", "compute_command", "compute_velocity", "update"):
            method = getattr(self._core, method_name, None)
            if method is None:
                continue
            try:
                signature = inspect.signature(method)
                if any(
                    parameter.kind == inspect.Parameter.VAR_KEYWORD
                    for parameter in signature.parameters.values()
                ):
                    return method(**values)
                kwargs = {
                    name: values[name]
                    for name in signature.parameters.keys()
                    if name in values
                }
                if kwargs:
                    return method(**kwargs)
            except (TypeError, ValueError):
                pass
            try:
                return method(
                    values["current_pose"],
                    values["current_velocity"],
                    values["global_path"],
                    values["scan_obstacle_points"],
                )
            except TypeError:
                continue
        raise RuntimeError("dwa_lite core has no supported planning API")

    def _extract_velocity(self, result: Any) -> Optional[Velocity2D]:
        value = self._result_value(
            result,
            ("best_cmd", "cmd", "velocity", "command", "twist"),
        )
        if value is None and isinstance(result, (tuple, list)) and result:
            value = result[0]
        if value is None:
            return None
        if isinstance(value, Twist):
            return Velocity2D(value.linear.x, value.linear.y, value.angular.z)
        if isinstance(value, dict):
            vx = value.get("vx", value.get("linear_x", value.get("x", 0.0)))
            vy = value.get("vy", value.get("linear_y", value.get("y", 0.0)))
            wz = value.get("wz", value.get("angular_z", value.get("theta", 0.0)))
            return self._finite_velocity(vx, vy, wz)
        if isinstance(value, (tuple, list)) and len(value) >= 3:
            return self._finite_velocity(value[0], value[1], value[2])
        vx = getattr(value, "vx", getattr(value, "x", None))
        vy = getattr(value, "vy", getattr(value, "y", 0.0))
        wz = getattr(value, "wz", getattr(value, "theta", getattr(value, "yaw_rate", None)))
        return self._finite_velocity(vx, vy, wz)

    @staticmethod
    def _finite_velocity(vx: Any, vy: Any, wz: Any) -> Optional[Velocity2D]:
        try:
            velocity = Velocity2D(float(vx), float(vy), float(wz))
        except (TypeError, ValueError):
            return None
        if not (
            math.isfinite(velocity.vx)
            and math.isfinite(velocity.vy)
            and math.isfinite(velocity.wz)
        ):
            return None
        return velocity

    def _bounded_velocity(self, velocity: Velocity2D) -> Velocity2D:
        vx = clamp(velocity.vx, self.min_vx, self.max_vx)
        vy = clamp(velocity.vy, self.min_vy, self.max_vy)
        wz = clamp(velocity.wz, self.min_wz, self.max_wz)
        if abs(vx) < self.vx_deadband:
            vx = 0.0
        if abs(vy) < self.vy_deadband:
            vy = 0.0
        if abs(wz) < self.wz_deadband:
            wz = 0.0
        return Velocity2D(vx, vy, wz)

    def _extract_trajectory(self, result: Any, fallback_pose: Pose2D) -> List[Pose2D]:
        value = self._result_value(
            result,
            ("best_trajectory", "local_trajectory", "trajectory", "path"),
        )
        if value is None and isinstance(result, (tuple, list)) and len(result) >= 2:
            value = result[1]
        if value is None:
            return [fallback_pose]
        trajectory: List[Pose2D] = []
        for pose in value:
            converted = self._pose_from_any(pose)
            if converted is not None:
                trajectory.append(converted)
        return trajectory or [fallback_pose]

    def _pose_from_any(self, value: Any) -> Optional[Pose2D]:
        if isinstance(value, Pose2D):
            return value
        if isinstance(value, PoseStamped):
            yaw = yaw_from_quaternion(value.pose.orientation)
            return Pose2D(value.pose.position.x, value.pose.position.y, yaw)
        if isinstance(value, dict):
            yaw = value.get("yaw", value.get("theta", 0.0))
            return self._finite_pose(value.get("x"), value.get("y"), yaw)
        if isinstance(value, (tuple, list)) and len(value) >= 2:
            yaw = value[2] if len(value) >= 3 else 0.0
            return self._finite_pose(value[0], value[1], yaw)
        return self._finite_pose(
            getattr(value, "x", None),
            getattr(value, "y", None),
            getattr(value, "yaw", getattr(value, "theta", 0.0)),
        )

    @staticmethod
    def _finite_pose(x: Any, y: Any, yaw: Any) -> Optional[Pose2D]:
        try:
            pose = Pose2D(float(x), float(y), float(yaw))
        except (TypeError, ValueError):
            return None
        if not (
            math.isfinite(pose.x)
            and math.isfinite(pose.y)
            and math.isfinite(pose.yaw)
        ):
            return None
        return pose

    def _extract_valid(self, result: Any) -> bool:
        value = self._result_value(result, ("valid", "is_valid", "success"))
        if value is None:
            return True
        return bool(value)

    def _extract_reason(self, result: Any) -> str:
        value = self._result_value(result, ("reason", "message", "status"))
        if value is None:
            state = self._result_value(result, ("planner_state", "state"))
            value = getattr(state, "reason", None) if state is not None else None
        return "" if value is None else str(value)

    def _extract_state(self, result: Any, reason: str, valid: bool) -> Dict[str, Any]:
        state = self._result_value(result, ("planner_state", "state"))
        if isinstance(state, dict):
            payload = dict(state)
        elif isinstance(state, str):
            payload = {"core_state": state}
        elif getattr(state, "value", None) is not None:
            payload = {"core_state": state.value}
        elif state is None:
            payload = {}
        else:
            payload = {
                name: self._json_safe(getattr(state, name))
                for name in dir(state)
                if not name.startswith("_")
                and not callable(getattr(state, name))
                and name not in ("reason",)
            }
        if "core_state" not in payload:
            core_state = payload.get("state", "")
            payload["core_state"] = self._json_safe(core_state) or (
                "TRACK_PATH" if valid else "BLOCKED_STOP"
            )
        debug = self._result_value(result, ("debug",))
        if isinstance(debug, dict):
            payload["debug"] = {
                str(key): self._json_safe(value) for key, value in debug.items()
            }
        if reason:
            payload["core_reason"] = reason
        return payload

    @staticmethod
    def _json_safe(value: Any) -> Any:
        if value is None:
            return None
        if isinstance(value, (bool, int, float, str)):
            return value
        enum_value = getattr(value, "value", None)
        if enum_value is not None:
            return enum_value
        return str(value)

    @staticmethod
    def _result_value(result: Any, names: Sequence[str]) -> Any:
        if result is None:
            return None
        if isinstance(result, dict):
            for name in names:
                if name in result:
                    return result[name]
            return None
        for name in names:
            if hasattr(result, name):
                return getattr(result, name)
        return None

    def _publish_cmd(
        self,
        velocity: Velocity2D,
        reason: str,
        stopped: bool = False,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        msg = Twist()
        msg.linear.x = velocity.vx
        msg.linear.y = velocity.vy
        msg.angular.z = velocity.wz
        self.cmd_pub.publish(msg)
        self.last_command = velocity
        self.last_status_reason = ""
        self._publish_state(reason, stopped=stopped, extra=extra)

    def _publish_stop(
        self,
        reason: str,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        if reason != self.last_status_reason:
            self.get_logger().info(f"Publishing zero /robot1/cmd_vel: {reason}")
            self.last_status_reason = reason
        self.cmd_pub.publish(Twist())
        self.last_command = Velocity2D(0.0, 0.0, 0.0)
        self._publish_state(reason, stopped=True, extra=extra)

    def _publish_state(
        self,
        reason: str,
        stopped: bool,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        payload: Dict[str, Any] = {
            "schema": "robot_planner.dwa_lite_state.string.v1",
            "node": self.get_name(),
            "active_planner": "dwa_lite",
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

    @staticmethod
    def _is_zero_velocity(velocity: Velocity2D) -> bool:
        return (
            abs(velocity.vx) < 1e-6
            and abs(velocity.vy) < 1e-6
            and abs(velocity.wz) < 1e-6
        )


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = DwaLitePlannerNode()
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
