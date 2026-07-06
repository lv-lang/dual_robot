from dataclasses import dataclass
from datetime import datetime, timezone


TASK_TYPE_TO_ID = {
    "DELIVERY": 1,
    "INSPECTION": 2,
    "RECHECK": 3,
}

TASK_TYPE_BY_ID = {value: key for key, value in TASK_TYPE_TO_ID.items()}

TASK_STATE_BY_ID = {
    1: "CREATED",
    2: "PENDING",
    3: "ASSIGNED",
    4: "RUNNING",
    5: "WAITING_CONFIRMATION",
    6: "PAUSED",
    7: "RESUMING",
    8: "SUCCEEDED",
    9: "FAILED",
    10: "CANCELED",
    11: "WAITING_RESOURCE",
}

ROBOT_STATE_BY_ID = {
    1: "IDLE",
    2: "ASSIGNED",
    3: "EXECUTING",
    4: "WAITING_CONFIRMATION",
    5: "RETURNING_HOME",
    6: "PAUSED",
    7: "ESTOP",
    8: "ERROR",
    9: "WAITING_RESOURCE",
}

POINT_KIND_BY_ID = {
    1: "WAITING_AREA",
    2: "PICKUP",
    3: "DELIVERY",
    4: "INSPECTION",
}

RESOURCE_TYPE_BY_ID = {
    1: "WAITING_AREA",
    2: "PICKUP",
    3: "DELIVERY",
    4: "INSPECTION",
    5: "RECHECK",
}

RESOURCE_STATUS_BY_ID = {
    0: "FREE",
    1: "LOCKED",
    2: "SHARED_ABNORMAL",
}

CONFIRMATION_TO_ID = {
    "OK": 1,
    "ABNORMAL": 2,
    "REJECT": 3,
}

MISSION_STEP_TYPE_BY_ID = {
    1: "NAVIGATE",
    2: "WAIT_CONFIRMATION",
    3: "RETURN_HOME",
}

SYSTEM_STATE_BY_ID = {
    1: "WAITING_ROBOTS",
    2: "STANDBY",
    3: "READY",
    4: "ESTOPPED",
    5: "INTERLOCKED",
}


def utc_now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def enum_name(value, mapping, default="UNKNOWN"):
    try:
        return mapping.get(int(value), default)
    except (TypeError, ValueError):
        return default


@dataclass(frozen=True)
class TaskPointInfo:
    point_id: str
    kind: str
    label: str = ""
    temporary: bool = False
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    has_pose: bool = False

    def to_dict(self):
        return {
            "point_id": self.point_id,
            "kind": self.kind,
            "label": self.label or self.point_id,
            "temporary": self.temporary,
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "has_pose": self.has_pose,
        }
