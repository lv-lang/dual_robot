from robot_web.dispatch_adapter import DispatchSnapshot
from robot_web.models import (
    MISSION_STEP_TYPE_BY_ID,
    RESOURCE_STATUS_BY_ID,
    RESOURCE_TYPE_BY_ID,
    ROBOT_STATE_BY_ID,
    SYSTEM_STATE_BY_ID,
    TASK_STATE_BY_ID,
    TASK_TYPE_BY_ID,
    enum_name,
    utc_now_iso,
)


def _stamp_to_dict(stamp):
    if stamp is None:
        return None
    return {
        "sec": int(getattr(stamp, "sec", 0)),
        "nanosec": int(getattr(stamp, "nanosec", 0)),
    }


def _pose_to_dict(pose_stamped):
    header = getattr(pose_stamped, "header", None)
    pose = getattr(pose_stamped, "pose", None)
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    return {
        "frame_id": getattr(header, "frame_id", "") if header else "",
        "stamp": _stamp_to_dict(getattr(header, "stamp", None)) if header else None,
        "position": {
            "x": float(getattr(position, "x", 0.0)),
            "y": float(getattr(position, "y", 0.0)),
            "z": float(getattr(position, "z", 0.0)),
        },
        "orientation": {
            "x": float(getattr(orientation, "x", 0.0)),
            "y": float(getattr(orientation, "y", 0.0)),
            "z": float(getattr(orientation, "z", 0.0)),
            "w": float(getattr(orientation, "w", 1.0)),
        },
    }


def mission_step_to_dict(step):
    return {
        "sequence": int(getattr(step, "sequence", 0)),
        "step_type": enum_name(getattr(step, "step_type", 0), MISSION_STEP_TYPE_BY_ID),
        "step_id": str(getattr(step, "step_id", "")),
        "point_id": str(getattr(step, "point_id", "")),
        "target_pose": _pose_to_dict(getattr(step, "target_pose", None)),
        "requires_confirmation": bool(getattr(step, "requires_confirmation", False)),
        "resource_id": str(getattr(step, "resource_id", "")),
        "label": str(getattr(step, "label", "")),
    }


def task_display_name(task_type, task_id, display_sequence=None):
    prefix = {
        "DELIVERY": "配送任务",
        "INSPECTION": "巡检任务",
        "RECHECK": "复查任务",
    }.get(task_type, "任务")
    if display_sequence is not None:
        return f"{prefix}_{display_sequence}"
    suffix = str(task_id or "").strip()
    if suffix.startswith("task_"):
        suffix = suffix[len("task_"):]
    return f"{prefix}_{suffix}" if suffix else prefix


def task_to_dict(task, display_sequence=None):
    state = getattr(task, "state", None)
    steps = [mission_step_to_dict(step) for step in getattr(task, "steps", [])]
    task_id = str(getattr(task, "task_id", ""))
    task_type = enum_name(getattr(task, "task_type", 0), TASK_TYPE_BY_ID)
    state_name = enum_name(getattr(state, "state", 0), TASK_STATE_BY_ID)
    current_step_index = int(getattr(task, "current_step_index", 0))
    current_step = steps[current_step_index] if 0 <= current_step_index < len(steps) else {}
    target_points = [step["point_id"] for step in steps if step.get("point_id")]
    label = str(getattr(task, "message", "")) or f"{task_type} {' -> '.join(target_points)}".strip()
    return {
        "task_id": task_id,
        "task_type": task_type,
        "state": state_name,
        "status": state_name,
        "state_reason": str(getattr(state, "reason", "")),
        "assigned_robot_id": str(getattr(task, "assigned_robot_id", "")),
        "robot_id": str(getattr(task, "assigned_robot_id", "")),
        "preferred_robot_id": str(getattr(task, "preferred_robot_id", "")) or "auto",
        "created_by": str(getattr(task, "created_by", "")),
        "current_step_index": current_step_index,
        "current_step_label": current_step.get("label") or current_step.get("point_id", ""),
        "steps": steps,
        "target_points": target_points,
        "label": label or task_id,
        "display_name": task_display_name(task_type, task_id, display_sequence),
        "display_sequence": display_sequence,
        "parent_task_id": str(getattr(task, "parent_task_id", "")),
        "excluded_robot_id": str(getattr(task, "excluded_robot_id", "")),
        "locked_resource_ids": list(getattr(task, "locked_resource_ids", [])),
        "business_result": int(getattr(task, "business_result", 0)),
        "message": str(getattr(task, "message", "")),
    }


def robot_state_to_dict(robot):
    robot_id = str(getattr(robot, "robot_id", ""))
    chassis_type = "mecanum" if robot_id == "mecanum" else "ackermann" if robot_id == "ackermann" else ""
    state_name = enum_name(getattr(robot, "state", 0), ROBOT_STATE_BY_ID)
    return {
        "robot_id": robot_id,
        "display_name": robot_id,
        "robot_namespace": str(getattr(robot, "robot_namespace", "")),
        "state": state_name,
        "status": state_name,
        "current_task_id": str(getattr(robot, "current_task_id", "")),
        "current_step_id": str(getattr(robot, "current_step_id", "")),
        "current_point_id": str(getattr(robot, "current_point_id", "")),
        "current_task_label": str(getattr(robot, "current_point_id", "")),
        "pose": _pose_to_dict(getattr(robot, "pose", None)),
        "message": str(getattr(robot, "message", "")),
        "chassis_type": chassis_type,
    }


def resource_lock_to_dict(lock):
    point_id = str(getattr(lock, "point_id", ""))
    resource_id = str(getattr(lock, "resource_id", ""))
    resource_type = enum_name(getattr(lock, "resource_type", 0), RESOURCE_TYPE_BY_ID)
    return {
        "resource_id": resource_id,
        "resource_type": resource_type,
        "status": enum_name(getattr(lock, "status", 0), RESOURCE_STATUS_BY_ID),
        "point_id": point_id,
        "point_label": point_id or resource_id,
        "locked_by_task_id": str(getattr(lock, "locked_by_task_id", "")),
        "holder_task_id": str(getattr(lock, "locked_by_task_id", "")),
        "locked_by_robot_id": str(getattr(lock, "locked_by_robot_id", "")),
        "robot_id": str(getattr(lock, "locked_by_robot_id", "")),
        "lock_type": resource_type,
        "shared_task_ids": list(getattr(lock, "shared_task_ids", [])),
        "reason": str(getattr(lock, "reason", "")),
    }


def system_state_to_dict(system_state):
    if system_state is None:
        return {
            "state": "UNKNOWN",
            "task_creation_allowed": False,
            "requires_operator_action": False,
            "healthy_for_enable": False,
            "map_version": "",
            "message": "",
            "warnings": [],
        }
    return {
        "state": enum_name(getattr(system_state, "state", 0), SYSTEM_STATE_BY_ID),
        "task_creation_allowed": bool(getattr(system_state, "task_creation_allowed", False)),
        "requires_operator_action": bool(getattr(system_state, "requires_operator_action", False)),
        "healthy_for_enable": bool(getattr(system_state, "healthy_for_enable", False)),
        "map_version": str(getattr(system_state, "map_version", "")),
        "message": str(getattr(system_state, "message", "")),
        "warnings": list(getattr(system_state, "warnings", [])),
    }


def waiting_confirmations_from_tasks(tasks):
    waiting = []
    for task in tasks:
        if task["state"] != "WAITING_CONFIRMATION":
            continue
        step_index = task["current_step_index"]
        steps = task["steps"]
        step = steps[step_index] if 0 <= step_index < len(steps) else {}
        waiting.append({
            "task_id": task["task_id"],
            "display_name": task["display_name"],
            "task_type": task["task_type"],
            "preferred_robot_id": task.get("preferred_robot_id", "auto"),
            "assigned_robot_id": task["assigned_robot_id"],
            "robot_id": task["assigned_robot_id"],
            "step_index": step_index,
            "step_id": step.get("step_id", ""),
            "point_id": step.get("point_id", ""),
            "point_label": step.get("label") or step.get("point_id", ""),
            "label": step.get("label") or step.get("point_id", ""),
        })
    return waiting


def aggregate_state(snapshot, dispatch_online, dispatch_message=""):
    if not isinstance(snapshot, DispatchSnapshot):
        snapshot = DispatchSnapshot(message=dispatch_message)

    display_counters = {}
    tasks = []
    for task in snapshot.tasks:
        task_type = enum_name(getattr(task, "task_type", 0), TASK_TYPE_BY_ID)
        display_counters[task_type] = display_counters.get(task_type, 0) + 1
        tasks.append(task_to_dict(task, display_counters[task_type]))
    robots = [robot_state_to_dict(robot) for robot in snapshot.robot_states]
    locks = [resource_lock_to_dict(lock) for lock in snapshot.resource_locks]
    waiting = waiting_confirmations_from_tasks(tasks)
    system_state = system_state_to_dict(snapshot.system_state)

    disabled_reasons = []
    if not dispatch_online:
        disabled_reasons.append("dispatch_offline")
    if any(robot["state"] == "ESTOP" for robot in robots):
        disabled_reasons.append("global_estop_active")
    if dispatch_online and not system_state["task_creation_allowed"]:
        disabled_reasons.append(f"system_{system_state['state'].lower()}")

    status = {
        "backend_online": True,
        "dispatch_online": bool(dispatch_online),
        "dispatch_degraded": not bool(dispatch_online),
        "system_state": system_state,
        "disabled_reasons": disabled_reasons,
        "updated_at": utc_now_iso(),
    }

    return {
        "status": status,
        "backend_status": {
            "online": True,
            "mode": "online",
        },
        "dispatch_status": {
            "online": bool(dispatch_online),
            "mode": "online" if dispatch_online else "degraded",
            "message": dispatch_message or snapshot.message,
        },
        "system_state": system_state,
        "robots": robots,
        "tasks": tasks,
        "resource_locks": locks,
        "waiting_confirmations": waiting,
        "disabled_reasons": disabled_reasons,
    }
