from copy import deepcopy

from robot_web.config import load_builtin_templates
from robot_web.dispatch_adapter import DispatchSnapshot
from robot_web.exceptions import GatewayError
from robot_web.models import CONFIRMATION_TO_ID, utc_now_iso
from robot_web.points import catalog_to_json
from robot_web.state import aggregate_state, task_display_name, task_to_dict
from robot_web.system_control import SYSTEM_EVENT_NAMES
from robot_web.validation import validate_template_payload


DEMO_INSPECTION_POINT_IDS = {"P1", "P2", "P3"}
DEMO_INSPECTION_POINT_KEYS = {"5", "6", "7", "8", "9", "10", "11", "18"}
DEMO_EVENT_KEY_HINT = "1-19, space, or clear"


DEMO_EVENT_DEFINITIONS = {
    "1": {
        "event_type": "demo_pickup_arrived",
        "message": "mecanum 已确认到达取货点 A",
        "level": "INFO",
        "detail": {
            "robot_id": "mecanum",
            "point_id": "A",
            "point_label": "取货点 A",
            "demo_key": "1",
        },
    },
    "2": {
        "event_type": "demo_delivery_arrived",
        "message": "mecanum 已确认到达配送点 D",
        "level": "INFO",
        "detail": {
            "robot_id": "mecanum",
            "point_id": "D",
            "point_label": "配送点 D",
            "demo_key": "2",
        },
    },
    "3": {
        "event_type": "demo_ackermann_pickup_arrived",
        "message": "ackermann 已确认到达取货点 B",
        "level": "INFO",
        "detail": {
            "robot_id": "ackermann",
            "point_id": "B",
            "point_label": "取货点 B",
            "demo_key": "3",
        },
    },
    "4": {
        "event_type": "demo_ackermann_delivery_arrived",
        "message": "ackermann 已确认到达配送点 C",
        "level": "INFO",
        "detail": {
            "robot_id": "ackermann",
            "point_id": "C",
            "point_label": "配送点 C",
            "demo_key": "4",
        },
    },
    "5": {
        "event_type": "demo_recheck_smoke",
        "message": "{point_id} 发现烟雾异常，已分配 mecanum 前往复检",
        "level": "WARNING",
        "detail": {
            "robot_id": "ackermann",
            "assigned_robot_id": "mecanum",
            "point_id": "{point_id}",
            "point_label": "巡检点 {point_id}",
            "abnormal_type": "smoke",
            "warning_active": True,
            "warning_severity": "warning",
            "warning_title": "WARNING",
            "warning_message": "{point_id} 烟雾异常，mecanum 正在复检",
            "demo_key": "5",
        },
    },
    "6": {
        "event_type": "demo_recheck_stack_risk",
        "message": "{point_id} 发现货物堆叠异常，已分配 mecanum 前往复检",
        "level": "WARNING",
        "detail": {
            "robot_id": "ackermann",
            "assigned_robot_id": "mecanum",
            "point_id": "{point_id}",
            "point_label": "巡检点 {point_id}",
            "abnormal_type": "stack_risk",
            "warning_active": True,
            "warning_severity": "warning",
            "warning_title": "WARNING",
            "warning_message": "{point_id} 货物堆叠异常，mecanum 正在复检",
            "demo_key": "6",
        },
    },
    "7": {
        "event_type": "demo_recheck_confirmed",
        "message": "{point_id} 复检确认异常，已通知对应区域安全负责人前去处理",
        "level": "WARNING",
        "detail": {
            "robot_id": "mecanum",
            "point_id": "{point_id}",
            "point_label": "巡检点 {point_id}",
            "warning_active": False,
            "warning_severity": "warning",
            "demo_key": "7",
        },
    },
    "8": {
        "event_type": "demo_fire_alert",
        "message": "发现当前巡检点 {point_id} 有火情",
        "level": "ERROR",
        "detail": {
            "robot_id": "ackermann",
            "point_id": "{point_id}",
            "point_label": "巡检点 {point_id}",
            "abnormal_type": "fire",
            "warning_active": True,
            "warning_severity": "danger",
            "warning_title": "WARNING",
            "warning_message": "{point_id} 火灾警告",
            "demo_key": "8",
        },
    },
    "9": {
        "event_type": "demo_fire_alert_cleared",
        "message": "已关闭 {point_id} 火灾警告",
        "level": "INFO",
        "detail": {
            "point_id": "{point_id}",
            "point_label": "巡检点 {point_id}",
            "warning_active": False,
            "warning_severity": "danger",
            "demo_key": "9",
        },
    },
    "10": {
        "event_type": "demo_inspection_started",
        "message": "ackermann 已确认到达巡检点 {point_id}，并开始巡检",
        "level": "INFO",
        "detail": {
            "robot_id": "ackermann",
            "point_id": "{point_id}",
            "point_label": "巡检点 {point_id}",
            "demo_key": "10",
        },
    },
    "11": {
        "event_type": "demo_inspection_normal",
        "message": "{point_id} 巡检正常",
        "level": "INFO",
        "detail": {
            "robot_id": "ackermann",
            "point_id": "{point_id}",
            "point_label": "巡检点 {point_id}",
            "warning_active": False,
            "warning_severity": "warning",
            "demo_key": "11",
        },
    },
    "12": {
        "event_type": "demo_ackermann_returning_home",
        "message": "ackermann 当前任务已结束，正在返回等待区",
        "level": "INFO",
        "detail": {
            "robot_id": "ackermann",
            "demo_key": "12",
        },
    },
    "13": {
        "event_type": "demo_mecanum_returning_home",
        "message": "mecanum 当前任务已结束，正在返回等待区",
        "level": "INFO",
        "detail": {
            "robot_id": "mecanum",
            "demo_key": "13",
        },
    },
    "14": {
        "event_type": "demo_mecanum_heading_pickup",
        "message": "mecanum 正在前往取货点 A",
        "level": "INFO",
        "detail": {
            "robot_id": "mecanum",
            "point_id": "A",
            "point_label": "取货点 A",
            "demo_key": "14",
        },
    },
    "15": {
        "event_type": "demo_ackermann_heading_pickup",
        "message": "ackermann 正在前往取货点 B",
        "level": "INFO",
        "detail": {
            "robot_id": "ackermann",
            "point_id": "B",
            "point_label": "取货点 B",
            "demo_key": "15",
        },
    },
    "16": {
        "event_type": "demo_mecanum_heading_delivery_d",
        "message": "mecanum 正在前往配送点 D",
        "level": "INFO",
        "detail": {
            "robot_id": "mecanum",
            "point_id": "D",
            "point_label": "配送点 D",
            "demo_key": "16",
        },
    },
    "17": {
        "event_type": "demo_ackermann_heading_delivery_c",
        "message": "ackermann 正在前往配送点 C",
        "level": "INFO",
        "detail": {
            "robot_id": "ackermann",
            "point_id": "C",
            "point_label": "配送点 C",
            "demo_key": "17",
        },
    },
    "18": {
        "event_type": "demo_ackermann_heading_inspection",
        "message": "ackermann 正在前往巡检点 {point_id}",
        "level": "INFO",
        "detail": {
            "robot_id": "ackermann",
            "point_id": "{point_id}",
            "point_label": "巡检点 {point_id}",
            "demo_key": "18",
        },
    },
    "19": {
        "event_type": "demo_mecanum_heading_inspection_p3",
        "message": "mecanum 正在前往巡检点 P3",
        "level": "INFO",
        "detail": {
            "robot_id": "mecanum",
            "point_id": "P3",
            "point_label": "巡检点 P3",
            "demo_key": "19",
        },
    },
}


class RobotWebGateway:
    def __init__(
        self,
        store,
        dispatch_client,
        builtin_templates=None,
        fallback_points=None,
        system_control=None,
    ):
        self.store = store
        self.dispatch = dispatch_client
        self.builtin_templates = builtin_templates or []
        self.fallback_points = fallback_points or {}
        self.system_control = system_control
        self._external_running_logged = False
        if self.system_control is not None and hasattr(self.system_control, "set_event_sink"):
            self.system_control.set_event_sink(self._log_system_event)

    @classmethod
    def from_paths(cls, store, dispatch_client, builtin_templates_file, fallback_points, system_control=None):
        return cls(
            store=store,
            dispatch_client=dispatch_client,
            builtin_templates=load_builtin_templates(builtin_templates_file),
            fallback_points=fallback_points,
            system_control=system_control,
        )

    def task_points(self):
        result = self.dispatch.get_task_points()
        if result.accepted and result.data:
            return result.data, "dispatch"
        if self.fallback_points:
            return self.fallback_points, "fallback"
        raise GatewayError(
            503,
            "dispatch_offline",
            "robot_dispatch is offline and no task point fallback is available",
        )

    def health(self):
        dispatch_online = self.dispatch.is_online(timeout_sec=0.1)
        return {
            "backend_online": True,
            "dispatch_online": dispatch_online,
            "dispatch_degraded": not dispatch_online,
            "mode": "online" if dispatch_online else "degraded",
            "reason": "" if dispatch_online else "dispatch_offline",
            "updated_at": utc_now_iso(),
            "disabled_reasons": [] if dispatch_online else ["dispatch_offline"],
        }

    def state(self):
        result = self.dispatch.get_state()
        if result.accepted:
            return aggregate_state(result.data, True, result.message)
        return aggregate_state(
            DispatchSnapshot(message=result.message),
            False,
            result.message,
        )

    def list_task_points(self):
        points, source = self.task_points()
        return {
            "source": source,
            "points": catalog_to_json(points),
        }

    @staticmethod
    def _template_to_public(template):
        public = dict(template)
        public.setdefault("display_name", public.get("name", ""))
        public.setdefault("name", public.get("display_name", ""))
        public.setdefault("target_point_ids", list(public.get("target_points", [])))
        public.setdefault("target_points", list(public.get("target_point_ids", [])))
        public["builtin"] = bool(public.get("builtin", public.get("readonly", False)))
        public["readonly"] = bool(public.get("readonly", public["builtin"]))
        return public

    @staticmethod
    def _point_to_business_point(point):
        kind = str(point.kind).upper()
        point_type = {
            "WAITING_AREA": "waiting",
            "PICKUP": "pickup",
            "DELIVERY": "delivery",
            "INSPECTION": "inspection",
        }.get(kind, kind.lower() or "unknown")
        return {
            "point_id": point.point_id,
            "label": point.label or point.point_id,
            "point_type": point_type,
            "kind": kind,
            "temporary": bool(point.temporary),
        }

    def list_templates(self):
        templates = list(self.builtin_templates) + self.store.list_user_templates()
        return sorted(
            [self._template_to_public(template) for template in templates],
            key=lambda item: (int(item.get("sort_order", 0)), item.get("display_name", "")),
        )

    @staticmethod
    def _annotate_template_availability(template, points):
        annotated = dict(template)
        target_point_ids = list(annotated.get("target_point_ids", annotated.get("target_points", [])))
        missing_point_ids = [point_id for point_id in target_point_ids if point_id not in points]
        annotated["available"] = not missing_point_ids
        annotated["missing_point_ids"] = missing_point_ids
        annotated["unavailable_reason"] = "missing_task_points" if missing_point_ids else ""
        return annotated

    def list_template_catalog(self):
        points, _ = self.task_points()
        return {
            "templates": [
                self._annotate_template_availability(template, points)
                for template in self.list_templates()
            ],
            "business_points": [
                self._point_to_business_point(point)
                for point in sorted(points.values(), key=lambda item: item.point_id)
            ],
        }

    def _find_template(self, template_id):
        for template in self.list_templates():
            if template["template_id"] == template_id:
                return template
        return None

    def _require_template(self, template_id):
        template = self._find_template(template_id)
        if template is None:
            raise GatewayError(404, "template_not_found", "template not found")
        return template

    def _reject_builtin(self, template_id):
        template = self._find_template(template_id)
        if template is not None and template.get("builtin"):
            raise GatewayError(
                409,
                "builtin_template_read_only",
                "built-in templates are read-only",
                {"template_id": template_id},
            )

    def create_template(self, payload):
        points, _ = self.task_points()
        normalized = validate_template_payload(payload, points)
        return self._template_to_public(self.store.create_user_template(normalized))

    def update_template(self, template_id, payload):
        self._reject_builtin(template_id)
        existing = self.store.get_user_template(template_id)
        if existing is None:
            raise GatewayError(404, "template_not_found", "template not found")
        merged = dict(existing)
        merged.update(payload)
        points, _ = self.task_points()
        normalized = validate_template_payload(merged, points)
        return self._template_to_public(self.store.update_user_template(template_id, normalized))

    def delete_template(self, template_id):
        self._reject_builtin(template_id)
        return self.store.delete_user_template(template_id)

    def reorder_templates(self, payload):
        if isinstance(payload, dict) and isinstance(payload.get("template_ids"), list):
            orders = []
            for index, template_id in enumerate(payload["template_ids"]):
                template = self._find_template(str(template_id))
                if template is None or template.get("builtin"):
                    continue
                orders.append({
                    "template_id": str(template_id),
                    "sort_order": (index + 1) * 10,
                })
        else:
            orders = payload.get("orders", payload if isinstance(payload, list) else None)
        if orders is None:
            raise GatewayError(400, "invalid_reorder", "orders must be provided")
        for item in orders:
            template_id = str(item.get("template_id", ""))
            self._reject_builtin(template_id)
        if orders:
            self.store.reorder_user_templates(orders)
        return self.list_template_catalog()

    def _log(self, event_type, message, level="INFO", task_id=None, template_id=None, detail=None, timestamp=None):
        return self.store.append_log(
            event_type=event_type,
            message=message,
            level=level,
            task_id=task_id,
            template_id=template_id,
            detail=detail or {},
            timestamp=timestamp,
        )

    def _log_system_event(self, event_type, message, level="INFO", detail=None):
        return self._log(event_type, message, level=level, detail=detail or {})

    @staticmethod
    def _status_for_dispatch_failure(result):
        if result.reason == "dispatch_offline":
            return 503
        if result.reason == "service_timeout":
            return 504
        return 409

    def trigger_template(self, template_id, payload=None):
        payload = payload or {}
        requester = str(payload.get("requester") or "pwa")
        template = self._require_template(template_id)
        points, _ = self.task_points()
        normalized = validate_template_payload(template, points)
        preferred_robot_id = normalized["robot_preference"] or "auto"
        note = f"robot_web template_id={template_id}"
        result = self.dispatch.create_task(
            normalized["task_type"],
            normalized["target_point_ids"],
            requester=requester,
            note=note,
            preferred_robot_id=preferred_robot_id,
        )
        if not result.accepted:
            log = self._log(
                "template_trigger_failed",
                result.message or "template trigger failed",
                level="ERROR",
                template_id=template_id,
                detail={"reason": result.reason, "template": normalized},
            )
            raise GatewayError(
                self._status_for_dispatch_failure(result),
                result.reason or "dispatch_rejected",
                result.message or "template trigger failed",
                {"log": log},
            )

        task = getattr(result.data, "task", None)
        task_id = str(getattr(result.data, "task_id", "")) or str(getattr(task, "task_id", ""))
        assigned_robot_id = str(getattr(result.data, "assigned_robot_id", "")) or str(getattr(task, "assigned_robot_id", ""))
        preferred_robot_id = str(getattr(result.data, "preferred_robot_id", "")) or preferred_robot_id
        state_task = None
        try:
            state_task = next(
                (candidate for candidate in self.state()["tasks"] if candidate["task_id"] == task_id),
                None,
            )
        except GatewayError:
            state_task = None
        display_name = (
            state_task.get("display_name")
            if state_task
            else task_display_name(normalized["task_type"], task_id)
        )
        log = self._log(
            "template_triggered",
            f"template {template_id} created task {task_id}",
            task_id=task_id,
            template_id=template_id,
            detail={
                "task_type": normalized["task_type"],
                "target_point_ids": normalized["target_point_ids"],
                "preferred_robot_id": preferred_robot_id,
                "assigned_robot_id": assigned_robot_id,
                "display_name": display_name,
            },
        )
        response = {
            "accepted": True,
            "task_id": task_id,
            "display_name": display_name,
            "preferred_robot_id": preferred_robot_id,
            "assigned_robot_id": assigned_robot_id,
            "message": result.message,
            "log": log,
        }
        if task is not None:
            response["task"] = state_task or task_to_dict(task)
            response["display_name"] = response["task"]["display_name"]
            response["assigned_robot_id"] = response["task"].get("assigned_robot_id", assigned_robot_id)
            if hasattr(task, "preferred_robot_id"):
                response["preferred_robot_id"] = response["task"].get("preferred_robot_id", preferred_robot_id)
        return response

    def _current_waiting_confirmation(self, task_id):
        state = self.state()
        if not state["dispatch_status"]["online"]:
            raise GatewayError(
                503,
                "dispatch_offline",
                state["dispatch_status"].get("message") or "robot_dispatch is offline",
            )
        for confirmation in state["waiting_confirmations"]:
            if confirmation["task_id"] == task_id:
                return confirmation
        raise GatewayError(
            409,
            "confirmation_not_waiting",
            "task is not waiting for confirmation",
            {"task_id": task_id},
        )

    def confirm_task(self, task_id, payload):
        result_name = str(payload.get("result", "")).upper()
        if result_name not in CONFIRMATION_TO_ID:
            raise GatewayError(
                400,
                "invalid_confirmation_result",
                "result must be OK, ABNORMAL, or REJECT",
            )
        current = self._current_waiting_confirmation(task_id)
        mismatches = {}
        for field in ("step_index", "step_id", "point_id"):
            if field in payload and payload[field] != current[field]:
                mismatches[field] = {"expected": current[field], "actual": payload[field]}
        if mismatches:
            raise GatewayError(
                409,
                "stale_confirmation",
                "confirmation request does not match the current waiting step",
                mismatches,
            )

        requester = str(payload.get("requester") or "pwa")
        note = str(payload.get("note") or "robot_web confirmation")
        dispatch_result = self.dispatch.confirm_task(
            task_id=task_id,
            result=result_name,
            requester=requester,
            step_index=current["step_index"],
            step_id=current["step_id"],
            point_id=current["point_id"],
            note=note,
        )
        if not dispatch_result.accepted:
            log = self._log(
                "confirmation_failed",
                dispatch_result.message or "confirmation failed",
                level="ERROR",
                task_id=task_id,
                detail={"reason": dispatch_result.reason, "result": result_name},
            )
            raise GatewayError(
                self._status_for_dispatch_failure(dispatch_result),
                dispatch_result.reason or "dispatch_rejected",
                dispatch_result.message or "confirmation failed",
                {"log": log},
            )
        derived_task_id = str(getattr(dispatch_result.data, "derived_task_id", ""))
        log = self._log(
            "confirmation_submitted",
            f"task {task_id} confirmed as {result_name}",
            task_id=task_id,
            detail={
                "result": result_name,
                "derived_task_id": derived_task_id,
                "display_name": current.get("display_name", ""),
            },
        )
        return {
            "accepted": True,
            "task_id": task_id,
            "result": result_name,
            "derived_task_id": derived_task_id,
            "message": dispatch_result.message,
            "log": log,
        }

    def _task_control(self, action, task_id, payload):
        requester = str(payload.get("requester") or "pwa")
        reason = str(payload.get("reason") or f"robot_web {action}")
        method = getattr(self.dispatch, f"{action}_task")
        task_display = ""
        try:
            current_task = next(
                (candidate for candidate in self.state()["tasks"] if candidate["task_id"] == task_id),
                None,
            )
            task_display = current_task.get("display_name", "") if current_task else ""
        except GatewayError:
            task_display = ""
        result = method(task_id, requester=requester, reason=reason)
        event_type = f"task_{action}"
        if not result.accepted:
            log = self._log(
                f"{event_type}_failed",
                result.message or f"{action} failed",
                level="ERROR",
                task_id=task_id,
                detail={"reason": result.reason},
            )
            raise GatewayError(
                self._status_for_dispatch_failure(result),
                result.reason or "dispatch_rejected",
                result.message or f"{action} failed",
                {"log": log},
            )
        log = self._log(
            event_type,
            f"task {task_id} {action} accepted",
            task_id=task_id,
            detail={"reason": reason, "display_name": task_display},
        )
        return {
            "accepted": True,
            "task_id": task_id,
            "action": action,
            "message": result.message,
            "log": log,
        }

    def pause_task(self, task_id, payload=None):
        return self._task_control("pause", task_id, payload or {})

    def resume_task(self, task_id, payload=None):
        return self._task_control("resume", task_id, payload or {})

    def cancel_task(self, task_id, payload=None):
        return self._task_control("cancel", task_id, payload or {})

    def emergency_stop(self, payload=None):
        payload = payload or {}
        requester = str(payload.get("requester") or "pwa")
        reason = str(payload.get("reason") or "robot_web global emergency stop")
        result = self.dispatch.emergency_stop(requester=requester, reason=reason)
        if not result.accepted:
            log = self._log(
                "global_estop_failed",
                result.message or "global emergency stop failed",
                level="ERROR",
                detail={"reason": result.reason},
            )
            raise GatewayError(
                self._status_for_dispatch_failure(result),
                result.reason or "dispatch_rejected",
                result.message or "global emergency stop failed",
                {"log": log},
            )
        log = self._log(
            "global_estop",
            "global emergency stop accepted",
            detail={"reason": reason},
        )
        return {
            "accepted": True,
            "message": result.message,
            "log": log,
        }

    def list_logs(self, limit=100):
        return self.store.list_logs(limit=limit, exclude_prefix="system_")

    def clear_demo_logs(self):
        deleted = self.store.clear_logs(exclude_prefix="system_")
        return {
            "accepted": True,
            "key": "space",
            "message": "事件日志已清空",
            "deleted": deleted,
            "logs": self.list_logs(limit=100),
        }

    @staticmethod
    def _demo_inspection_point_id(payload=None):
        payload = payload or {}
        point_id = str(payload.get("point_id") or payload.get("inspection_point_id") or "P3").upper()
        if point_id not in DEMO_INSPECTION_POINT_IDS:
            raise GatewayError(
                400,
                "invalid_demo_inspection_point",
                "demo inspection point must be P1, P2, or P3",
                {"point_id": point_id},
            )
        return point_id

    @staticmethod
    def _render_demo_event(key, event, payload=None):
        rendered = deepcopy(event)
        if key not in DEMO_INSPECTION_POINT_KEYS:
            return rendered
        point_id = RobotWebGateway._demo_inspection_point_id(payload)
        rendered["message"] = rendered["message"].format(point_id=point_id)
        rendered["detail"] = {
            name: value.format(point_id=point_id) if isinstance(value, str) else value
            for name, value in rendered["detail"].items()
        }
        return rendered

    def trigger_demo_event(self, key, payload=None):
        key = str(key)
        if key in {"space", "clear"}:
            return self.clear_demo_logs()
        event = DEMO_EVENT_DEFINITIONS.get(str(key))
        if event is None:
            raise GatewayError(
                404,
                "demo_event_not_found",
                f"demo event key must be {DEMO_EVENT_KEY_HINT}",
                {"key": str(key)},
            )
        event = self._render_demo_event(key, event, payload)
        log = self._log(
            event["event_type"],
            event["message"],
            level=event["level"],
            detail=event["detail"],
        )
        return {
            "accepted": True,
            "key": str(key),
            "message": event["message"],
            "log": log,
        }

    def _require_system_control(self):
        if self.system_control is None:
            raise GatewayError(
                503,
                "system_control_unavailable",
                "system control backend is not configured",
            )
        return self.system_control

    @staticmethod
    def _reject_system_command_payload(payload):
        payload = payload or {}
        forbidden = {
            "command",
            "cmd",
            "args",
            "argv",
            "launch_file",
            "launch_args",
            "profile",
            "profile_id",
        }
        provided = sorted(forbidden.intersection(payload.keys()))
        if provided:
            raise GatewayError(
                400,
                "system_command_not_allowed",
                "system control uses a fixed backend white-listed launch command",
                {"forbidden_fields": provided},
            )

    def system_status(self):
        status = self._require_system_control().status()
        if status.get("external_running"):
            if not self._external_running_logged:
                self._log_system_event(
                    "system_external_running",
                    "检测到外部运行中的调度系统，App 进入只读状态",
                    detail={"status": status.get("status")},
                )
                self._external_running_logged = True
        else:
            self._external_running_logged = False
        return status

    def system_start(self, payload=None):
        self._reject_system_command_payload(payload)
        system = self._require_system_control()
        try:
            status = system.start()
        except GatewayError as exc:
            event_type = (
                "system_external_running"
                if exc.reason == "system_external_running"
                else "system_start_failed"
            )
            log = self._log_system_event(
                event_type,
                exc.message,
                level="ERROR",
                detail={"reason": exc.reason},
            )
            exc.detail["log"] = log
            raise
        log = self._log_system_event(
            "system_start",
            "调度系统启动中",
            detail={"profile_id": status["profile"]["id"], "pid": status["pid"], "pgid": status["pgid"]},
        )
        return {
            "accepted": True,
            "message": "调度系统启动中",
            "status": status,
            "log": log,
        }

    def system_stop(self, payload=None):
        self._reject_system_command_payload(payload)
        system = self._require_system_control()
        try:
            status = system.stop()
        except GatewayError as exc:
            event_type = (
                "system_external_running"
                if exc.reason == "system_external_running"
                else "system_stop_failed"
            )
            log = self._log_system_event(
                event_type,
                exc.message,
                level="ERROR",
                detail={"reason": exc.reason},
            )
            exc.detail["log"] = log
            raise
        log = self._log_system_event("system_stop", "调度系统已停止")
        return {
            "accepted": True,
            "message": "调度系统已停止",
            "status": status,
            "log": log,
        }

    def system_restart(self, payload=None):
        self._reject_system_command_payload(payload)
        system = self._require_system_control()
        try:
            status = system.restart()
        except GatewayError as exc:
            event_type = (
                "system_external_running"
                if exc.reason == "system_external_running"
                else "system_restart_failed"
            )
            log = self._log_system_event(
                event_type,
                exc.message,
                level="ERROR",
                detail={"reason": exc.reason},
            )
            exc.detail["log"] = log
            raise
        log = self._log_system_event(
            "system_restart",
            "调度系统重启中",
            detail={"profile_id": status["profile"]["id"], "pid": status["pid"], "pgid": status["pgid"]},
        )
        return {
            "accepted": True,
            "message": "调度系统重启中",
            "status": status,
            "log": log,
        }

    @staticmethod
    def _operation_log_to_public(log):
        return {
            "log_id": str(log.get("log_id") or log.get("id") or ""),
            "timestamp": log.get("timestamp") or log.get("created_at") or "",
            "level": log.get("level", "info"),
            "event": log.get("event") or log.get("event_type") or "",
            "message": log.get("message", ""),
            "detail": log.get("detail", {}),
        }

    def system_logs(self, limit=120):
        system = self._require_system_control()
        operation_logs = [
            self._operation_log_to_public(log)
            for log in self.store.list_logs(limit=limit, event_prefix="system_")
            if log.get("event_type") in SYSTEM_EVENT_NAMES
        ]
        return {
            "launch_logs": system.logs(limit=limit),
            "operation_logs": operation_logs,
        }
