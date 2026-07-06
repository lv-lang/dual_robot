from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


ACTIVE_POINT_TYPES = {"pickup", "delivery", "inspection"}


@dataclass(frozen=True)
class VisionTarget:
    point_id: str
    point_type: str
    x: float
    y: float
    task_id: str = ""
    step_id: str = ""

    @property
    def key(self) -> str:
        return f"{self.task_id}:{self.step_id}:{self.point_type}:{self.point_id}"


@dataclass(frozen=True)
class VisionActivationSnapshot:
    active: bool
    target: Optional[VisionTarget]
    pose_x: Optional[float]
    pose_y: Optional[float]
    distance: Optional[float]


def infer_robot_id_from_path(path: str) -> str:
    lowered_parts = [part.lower() for part in Path(path).parts]
    lowered = "/".join(lowered_parts)
    if "ackermann_ws" in lowered or "ackermann" in lowered_parts:
        return "ackermann"
    if "mecanum_ws" in lowered or "mecanum" in lowered_parts:
        return "mecanum"
    return "robot_a"


def default_mission_feedback_topic(robot_id: str) -> str:
    if robot_id in {"mecanum", "ackermann"}:
        return f"/{robot_id}/execute_mission/_action/feedback"
    return "/execute_mission/_action/feedback"


def default_amcl_pose_topic(robot_id: str) -> str:
    if robot_id in {"mecanum", "ackermann"}:
        return f"/{robot_id}/amcl_pose"
    return "/amcl_pose"


def point_type_from_id(point_id: str) -> Optional[str]:
    normalized = point_id.strip().upper()
    if normalized.startswith("PICKUP_") or normalized in {"A", "B"}:
        return "pickup"
    if normalized.startswith("DELIVERY_") or normalized in {"C", "D"}:
        return "delivery"
    if normalized.startswith("RVIZ_PICKUP_"):
        return "pickup"
    if normalized.startswith("RVIZ_DELIVERY_"):
        return "delivery"
    if normalized.startswith("RVIZ_INSPECTION_"):
        return "inspection"
    if len(normalized) >= 2 and normalized[0] == "P" and normalized[1:].isdigit():
        return "inspection"
    return None


class VisionActivationGate:
    def __init__(self, enter_radius: float = 0.5, exit_radius: float = 0.7):
        self.enter_radius = max(float(enter_radius), 0.0)
        self.exit_radius = max(float(exit_radius), self.enter_radius)
        self._target: Optional[VisionTarget] = None
        self._pose_x: Optional[float] = None
        self._pose_y: Optional[float] = None
        self._active = False
        self._lock = threading.Lock()

    def clear_target(self) -> None:
        with self._lock:
            self._target = None
            self._active = False

    def update_target(
        self,
        point_id: str,
        x: float,
        y: float,
        *,
        point_type: Optional[str] = None,
        task_id: str = "",
        step_id: str = "",
    ) -> bool:
        resolved_type = point_type or point_type_from_id(point_id)
        if resolved_type not in ACTIVE_POINT_TYPES:
            self.clear_target()
            return False
        if not math.isfinite(float(x)) or not math.isfinite(float(y)):
            self.clear_target()
            return False

        target = VisionTarget(
            point_id=point_id.strip(),
            point_type=resolved_type,
            x=float(x),
            y=float(y),
            task_id=task_id.strip(),
            step_id=step_id.strip(),
        )
        with self._lock:
            if self._target is None or self._target.key != target.key:
                self._active = False
            self._target = target
            self._refresh_locked()
        return True

    def update_pose(self, x: float, y: float) -> None:
        if not math.isfinite(float(x)) or not math.isfinite(float(y)):
            return
        with self._lock:
            self._pose_x = float(x)
            self._pose_y = float(y)
            self._refresh_locked()

    def snapshot(self) -> VisionActivationSnapshot:
        with self._lock:
            distance = self._distance_locked()
            return VisionActivationSnapshot(
                active=self._active,
                target=self._target,
                pose_x=self._pose_x,
                pose_y=self._pose_y,
                distance=distance,
            )

    def _refresh_locked(self) -> None:
        distance = self._distance_locked()
        if distance is None:
            self._active = False
            return
        if self._active:
            if distance > self.exit_radius:
                self._active = False
        elif distance <= self.enter_radius:
            self._active = True

    def _distance_locked(self) -> Optional[float]:
        if self._target is None or self._pose_x is None or self._pose_y is None:
            return None
        return math.hypot(self._pose_x - self._target.x, self._pose_y - self._target.y)
