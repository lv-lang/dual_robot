from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeoutError
from types import SimpleNamespace

import pytest

from robot_web.dispatch_adapter import DispatchResult, DispatchSnapshot
from robot_web.exceptions import GatewayError
from robot_web.gateway import RobotWebGateway
from robot_web.models import TaskPointInfo
from robot_web.storage import RobotWebStore


def point_catalog():
    return {
        "PICKUP_A": TaskPointInfo("PICKUP_A", "PICKUP", "A 取货点"),
        "DELIVERY_C": TaskPointInfo("DELIVERY_C", "DELIVERY", "C 配送点"),
        "P1": TaskPointInfo("P1", "INSPECTION", "P1 巡检点"),
        "P2": TaskPointInfo("P2", "INSPECTION", "P2 巡检点"),
    }


def builtin_templates():
    return [
        {
            "template_id": "builtin_delivery_demo",
            "display_name": "配送演示",
            "task_type": "DELIVERY",
            "target_point_ids": ["PICKUP_A", "DELIVERY_C"],
            "robot_preference": "auto",
            "builtin": True,
            "sort_order": 10,
            "created_at": "now",
            "updated_at": "now",
        },
        {
            "template_id": "builtin_inspection_demo",
            "display_name": "巡检演示",
            "task_type": "INSPECTION",
            "target_point_ids": ["P1", "P2"],
            "robot_preference": "auto",
            "builtin": True,
            "sort_order": 20,
            "created_at": "now",
            "updated_at": "now",
        },
    ]


class FakeDispatch:
    def __init__(self, online=True, snapshot=None, points=None):
        self.online = online
        self.snapshot = snapshot or DispatchSnapshot(message="ok")
        self.points = points or point_catalog()
        self.calls = []

    def is_online(self, timeout_sec=0.1):
        return self.online

    def get_state(self):
        if not self.online:
            return DispatchResult(False, "dispatch down", reason="dispatch_offline")
        return DispatchResult(True, self.snapshot.message, data=self.snapshot)

    def get_task_points(self):
        if not self.online:
            return DispatchResult(False, "dispatch down", reason="dispatch_offline")
        return DispatchResult(True, "ok", data=self.points)

    def create_task(self, task_type, point_ids, requester, note="", preferred_robot_id="auto"):
        self.calls.append(("create_task", task_type, list(point_ids), requester, note, preferred_robot_id))
        if not self.online:
            return DispatchResult(False, "dispatch down", reason="dispatch_offline")
        return DispatchResult(
            True,
            "created",
            data=SimpleNamespace(task_id="task_1", assigned_robot_id="ackermann"),
        )

    def confirm_task(self, task_id, result, requester, step_index=0, step_id="", point_id="", note=""):
        self.calls.append(("confirm_task", task_id, result, requester, step_index, step_id, point_id, note))
        return DispatchResult(
            True,
            "confirmed",
            data=SimpleNamespace(derived_task_id="task_2" if result == "ABNORMAL" else ""),
        )

    def pause_task(self, task_id, requester, reason=""):
        self.calls.append(("pause_task", task_id, requester, reason))
        return DispatchResult(True, "paused", data=SimpleNamespace(task_id=task_id))

    def resume_task(self, task_id, requester, reason=""):
        self.calls.append(("resume_task", task_id, requester, reason))
        return DispatchResult(True, "resumed", data=SimpleNamespace(task_id=task_id))

    def cancel_task(self, task_id, requester, reason=""):
        self.calls.append(("cancel_task", task_id, requester, reason))
        return DispatchResult(True, "canceled", data=SimpleNamespace(task_id=task_id))

    def emergency_stop(self, requester, reason=""):
        self.calls.append(("emergency_stop", requester, reason))
        return DispatchResult(True, "emergency stop active")


class FakeSystemControl:
    def __init__(self):
        self.current_status = self._status("stopped", managed=False)
        self.calls = []

    def set_event_sink(self, event_sink):
        self.event_sink = event_sink

    @staticmethod
    def _status(status, managed):
        return {
            "status": status,
            "summary": "调度系统启动中" if managed else "调度系统未启动",
            "managed": managed,
            "external_running": False,
            "can_start": not managed,
            "can_stop": managed,
            "can_restart": managed,
            "profile": {
                "id": "real_robot_control_plane",
                "name": "Real Robot Control Plane",
                "command": "ros2 launch robot_bringup real_robot_control_plane.launch.py launch_rviz:=true",
            },
            "pid": 620001 if managed else None,
            "pgid": 620001 if managed else None,
            "started_at": "2026-07-06T10:00:00+08:00" if managed else "",
            "updated_at": "2026-07-06T10:00:00+08:00",
            "health": [],
        }

    def status(self):
        return self.current_status

    def start(self):
        self.calls.append("start")
        self.current_status = self._status("starting", managed=True)
        return self.current_status

    def stop(self):
        self.calls.append("stop")
        self.current_status = self._status("stopped", managed=False)
        return self.current_status

    def restart(self):
        self.calls.append("restart")
        self.current_status = self._status("starting", managed=True)
        return self.current_status

    def logs(self, limit=120):
        return []


def make_gateway(tmp_path, dispatch, system_control=None):
    return RobotWebGateway(
        store=RobotWebStore(tmp_path / "robot_web.sqlite3"),
        dispatch_client=dispatch,
        builtin_templates=builtin_templates(),
        fallback_points=point_catalog(),
        system_control=system_control,
    )


def waiting_snapshot():
    step = SimpleNamespace(
        sequence=0,
        step_type=1,
        step_id="task_1_P1",
        point_id="P1",
        target_pose=None,
        requires_confirmation=True,
        resource_id="P1",
        label="P1 巡检点",
    )
    task_state = SimpleNamespace(state=5, reason="")
    task = SimpleNamespace(
        task_id="task_1",
        task_type=2,
        state=task_state,
        assigned_robot_id="ackermann",
        preferred_robot_id="ackermann",
        created_by="robot_web",
        current_step_index=0,
        steps=[step],
        parent_task_id="",
        excluded_robot_id="",
        locked_resource_ids=["P1"],
        business_result=0,
        message="",
    )
    robot = SimpleNamespace(
        robot_id="ackermann",
        robot_namespace="/ackermann",
        state=4,
        current_task_id="task_1",
        current_step_id="task_1_P1",
        current_point_id="P1",
        pose=None,
        message="",
    )
    return DispatchSnapshot(tasks=[task], robot_states=[robot], resource_locks=[], message="ok")


def task_snapshot(task_id, task_type, state=4, assigned_robot_id="mecanum"):
    task_state = SimpleNamespace(state=state, reason="")
    step = SimpleNamespace(
        sequence=0,
        step_type=1,
        step_id=f"{task_id}_step",
        point_id="P1" if task_type != 1 else "PICKUP_A",
        target_pose=None,
        requires_confirmation=True,
        resource_id="P1" if task_type != 1 else "PICKUP_A",
        label="P1 巡检点" if task_type != 1 else "A 取货点",
    )
    return SimpleNamespace(
        task_id=task_id,
        task_type=task_type,
        state=task_state,
        assigned_robot_id=assigned_robot_id,
        preferred_robot_id="auto",
        created_by="robot_web",
        current_step_index=0,
        steps=[step],
        parent_task_id="",
        excluded_robot_id="",
        locked_resource_ids=[],
        business_result=0,
        message="",
    )


def test_health_and_state_degraded_when_dispatch_offline(tmp_path):
    gateway = make_gateway(tmp_path, FakeDispatch(online=False))

    health = gateway.health()
    assert health["backend_online"] is True
    assert health["dispatch_online"] is False
    assert health["mode"] == "degraded"

    state = gateway.state()
    assert state["dispatch_status"]["mode"] == "degraded"
    assert state["disabled_reasons"] == ["dispatch_offline"]
    assert state["tasks"] == []


def test_template_crud_validates_read_only_and_persists(tmp_path):
    dispatch = FakeDispatch()
    gateway = make_gateway(tmp_path, dispatch)

    created = gateway.create_template({
        "name": "Inspect P1",
        "task_type": "INSPECTION",
        "target_points": ["P1"],
        "robot_preference": "ackermann",
        "sort_order": 30,
    })
    assert created["builtin"] is False
    assert created["name"] == "Inspect P1"
    assert created["target_point_ids"] == ["P1"]
    assert created["target_points"] == ["P1"]

    reloaded = make_gateway(tmp_path, dispatch)
    assert reloaded._require_template(created["template_id"])["display_name"] == "Inspect P1"

    with pytest.raises(GatewayError) as read_only:
        gateway.delete_template("builtin_delivery_demo")
    assert read_only.value.reason == "builtin_template_read_only"

    with pytest.raises(GatewayError) as bad_point:
        gateway.create_template({
            "display_name": "Unknown",
            "task_type": "INSPECTION",
            "target_point_ids": ["MISSING"],
        })
    assert bad_point.value.reason == "unknown_task_points"

    with pytest.raises(GatewayError) as recheck:
        gateway.create_template({
            "display_name": "Forbidden",
            "task_type": "RECHECK",
            "target_point_ids": ["P1"],
        })
    assert recheck.value.reason == "invalid_task_type"


def test_template_catalog_marks_templates_with_missing_points_unavailable(tmp_path):
    rviz_points = {
        **point_catalog(),
        "RVIZ_PICKUP_1": TaskPointInfo("RVIZ_PICKUP_1", "PICKUP", "RViz 临时取货点", temporary=True),
        "RVIZ_DELIVERY_1": TaskPointInfo("RVIZ_DELIVERY_1", "DELIVERY", "RViz 临时放货点", temporary=True),
    }
    gateway = make_gateway(tmp_path, FakeDispatch(points=rviz_points))
    created = gateway.create_template({
        "display_name": "RViz 临时配送",
        "task_type": "DELIVERY",
        "target_point_ids": ["RVIZ_PICKUP_1", "RVIZ_DELIVERY_1"],
    })

    reloaded = make_gateway(tmp_path, FakeDispatch(points=point_catalog()))
    catalog = reloaded.list_template_catalog()

    stale_template = next(item for item in catalog["templates"] if item["template_id"] == created["template_id"])
    assert stale_template["available"] is False
    assert stale_template["unavailable_reason"] == "missing_task_points"
    assert stale_template["missing_point_ids"] == ["RVIZ_PICKUP_1", "RVIZ_DELIVERY_1"]

    valid_template = next(item for item in catalog["templates"] if item["template_id"] == "builtin_delivery_demo")
    assert valid_template["available"] is True
    assert valid_template["missing_point_ids"] == []


def test_trigger_template_maps_to_create_task_and_logs(tmp_path):
    dispatch = FakeDispatch()
    gateway = make_gateway(tmp_path, dispatch)

    response = gateway.trigger_template("builtin_delivery_demo", {"requester": "tablet"})

    assert response["accepted"] is True
    assert response["task_id"] == "task_1"
    assert response["display_name"] == "配送任务_1"
    assert response["preferred_robot_id"] == "auto"
    assert response["assigned_robot_id"] == "ackermann"
    assert dispatch.calls[0][0:4] == (
        "create_task",
        "DELIVERY",
        ["PICKUP_A", "DELIVERY_C"],
        "tablet",
    )
    assert dispatch.calls[0][4] == "robot_web template_id=builtin_delivery_demo"
    assert "robot_preference=" not in dispatch.calls[0][4]
    assert dispatch.calls[0][5] == "auto"
    logs = gateway.list_logs()
    assert logs[0]["event_type"] == "template_triggered"
    assert logs[0]["task_id"] == "task_1"
    assert logs[0]["task_display_name"] == "配送任务_1"
    assert logs[0]["detail"]["display_name"] == "配送任务_1"
    assert logs[0]["detail"]["preferred_robot_id"] == "auto"
    assert logs[0]["detail"]["assigned_robot_id"] == "ackermann"


def test_trigger_template_passes_robot_preference_as_formal_field(tmp_path):
    dispatch = FakeDispatch()
    gateway = make_gateway(tmp_path, dispatch)

    template = gateway.create_template({
        "display_name": "Robot2 delivery",
        "task_type": "DELIVERY",
        "target_point_ids": ["PICKUP_A", "DELIVERY_C"],
        "robot_preference": "ackermann",
    })

    response = gateway.trigger_template(template["template_id"], {"requester": "tablet"})

    assert response["task_id"] == "task_1"
    assert response["preferred_robot_id"] == "ackermann"
    assert dispatch.calls[-1] == (
        "create_task",
        "DELIVERY",
        ["PICKUP_A", "DELIVERY_C"],
        "tablet",
        f"robot_web template_id={template['template_id']}",
        "ackermann",
    )


def test_trigger_template_returns_type_scoped_display_name(tmp_path):
    inspection = task_snapshot("task_1", 2, assigned_robot_id="ackermann")
    delivery = task_snapshot("task_2", 1, assigned_robot_id="mecanum")

    class SequencedDispatch(FakeDispatch):
        def __init__(self):
            super().__init__(snapshot=DispatchSnapshot(tasks=[inspection], message="ok"))

        def create_task(self, task_type, point_ids, requester, note="", preferred_robot_id="auto"):
            self.calls.append(("create_task", task_type, list(point_ids), requester, note, preferred_robot_id))
            self.snapshot = DispatchSnapshot(tasks=[inspection, delivery], message="ok")
            return DispatchResult(
                True,
                "created",
                data=SimpleNamespace(task_id="task_2", task=delivery, assigned_robot_id="mecanum"),
            )

    gateway = make_gateway(tmp_path, SequencedDispatch())

    response = gateway.trigger_template("builtin_delivery_demo", {"requester": "tablet"})

    assert response["task_id"] == "task_2"
    assert response["display_name"] == "配送任务_1"
    assert response["task"]["display_name"] == "配送任务_1"


def test_state_adds_business_display_names_to_tasks_and_waiting_confirmations(tmp_path):
    dispatch = FakeDispatch(snapshot=waiting_snapshot())
    gateway = make_gateway(tmp_path, dispatch)

    state = gateway.state()

    assert state["tasks"][0]["task_id"] == "task_1"
    assert state["tasks"][0]["display_name"] == "巡检任务_1"
    assert state["tasks"][0]["preferred_robot_id"] == "ackermann"
    assert state["waiting_confirmations"][0]["task_id"] == "task_1"
    assert state["waiting_confirmations"][0]["display_name"] == "巡检任务_1"
    assert state["waiting_confirmations"][0]["preferred_robot_id"] == "ackermann"


def test_state_numbers_task_display_names_independently_by_task_type(tmp_path):
    snapshot = DispatchSnapshot(
        tasks=[
            task_snapshot("task_1", 2, assigned_robot_id="ackermann"),
            task_snapshot("task_2", 1, state=5, assigned_robot_id="mecanum"),
            task_snapshot("task_3", 3, assigned_robot_id="ackermann"),
            task_snapshot("task_4", 1, assigned_robot_id="mecanum"),
        ],
        message="ok",
    )
    gateway = make_gateway(tmp_path, FakeDispatch(snapshot=snapshot))

    state = gateway.state()

    assert [task["display_name"] for task in state["tasks"]] == [
        "巡检任务_1",
        "配送任务_1",
        "复查任务_1",
        "配送任务_2",
    ]
    assert state["tasks"][1]["task_id"] == "task_2"
    assert state["tasks"][1]["display_sequence"] == 1
    assert state["waiting_confirmations"][0]["task_id"] == "task_2"
    assert state["waiting_confirmations"][0]["display_name"] == "配送任务_1"


def test_confirm_rejects_stale_request_and_maps_abnormal(tmp_path):
    dispatch = FakeDispatch(snapshot=waiting_snapshot())
    gateway = make_gateway(tmp_path, dispatch)

    with pytest.raises(GatewayError) as stale:
        gateway.confirm_task("task_1", {
            "result": "OK",
            "step_index": 0,
            "step_id": "task_1_P1",
            "point_id": "P2",
        })
    assert stale.value.reason == "stale_confirmation"

    response = gateway.confirm_task("task_1", {
        "result": "ABNORMAL",
        "step_index": 0,
        "step_id": "task_1_P1",
        "point_id": "P1",
        "requester": "tablet",
    })

    assert response["accepted"] is True
    assert response["derived_task_id"] == "task_2"
    assert dispatch.calls[-1][0:7] == (
        "confirm_task",
        "task_1",
        "ABNORMAL",
        "tablet",
        0,
        "task_1_P1",
        "P1",
    )


def test_task_controls_and_global_estop_log_public_actions(tmp_path):
    dispatch = FakeDispatch()
    gateway = make_gateway(tmp_path, dispatch)

    assert gateway.pause_task("task_1")["accepted"] is True
    assert gateway.resume_task("task_1")["accepted"] is True
    assert gateway.cancel_task("task_1")["accepted"] is True
    assert gateway.emergency_stop({"reason": "operator"})["accepted"] is True

    actions = [call[0] for call in dispatch.calls]
    assert actions == ["pause_task", "resume_task", "cancel_task", "emergency_stop"]
    assert [log["event_type"] for log in gateway.list_logs(limit=4)] == [
        "global_estop",
        "task_cancel",
        "task_resume",
        "task_pause",
    ]


def test_demo_event_keys_write_competition_logs(tmp_path):
    gateway = make_gateway(tmp_path, FakeDispatch())

    pickup = gateway.trigger_demo_event("1")
    delivery = gateway.trigger_demo_event("2")
    ackermann_pickup = gateway.trigger_demo_event("3")
    ackermann_delivery = gateway.trigger_demo_event("4")
    mecanum_heading_pickup = gateway.trigger_demo_event("14")
    ackermann_heading_pickup = gateway.trigger_demo_event("15")
    mecanum_heading_delivery_d = gateway.trigger_demo_event("16")
    ackermann_heading_delivery_c = gateway.trigger_demo_event("17")
    ackermann_heading_inspection = gateway.trigger_demo_event("18", {"point_id": "P2"})
    mecanum_heading_inspection_p3 = gateway.trigger_demo_event("19")
    fire = gateway.trigger_demo_event("8", {"point_id": "P2"})
    cleared = gateway.trigger_demo_event("9", {"point_id": "P2"})
    inspection_started = gateway.trigger_demo_event("10", {"point_id": "P1"})
    inspection_normal = gateway.trigger_demo_event("11", {"point_id": "P1"})
    ackermann_returning = gateway.trigger_demo_event("12")
    mecanum_returning = gateway.trigger_demo_event("13")

    assert pickup["log"]["event_type"] == "demo_pickup_arrived"
    assert pickup["log"]["message"] == "mecanum 已确认到达取货点 A"
    assert pickup["log"]["detail"]["robot_id"] == "mecanum"
    assert delivery["log"]["message"] == "mecanum 已确认到达配送点 D"
    assert delivery["log"]["detail"]["point_id"] == "D"
    assert ackermann_pickup["log"]["event_type"] == "demo_ackermann_pickup_arrived"
    assert ackermann_pickup["log"]["message"] == "ackermann 已确认到达取货点 B"
    assert ackermann_pickup["log"]["detail"]["point_id"] == "B"
    assert ackermann_delivery["log"]["event_type"] == "demo_ackermann_delivery_arrived"
    assert ackermann_delivery["log"]["message"] == "ackermann 已确认到达配送点 C"
    assert ackermann_delivery["log"]["detail"]["point_id"] == "C"
    assert mecanum_heading_pickup["log"]["message"] == "mecanum 正在前往取货点 A"
    assert ackermann_heading_pickup["log"]["message"] == "ackermann 正在前往取货点 B"
    assert mecanum_heading_delivery_d["log"]["event_type"] == "demo_mecanum_heading_delivery_d"
    assert mecanum_heading_delivery_d["log"]["message"] == "mecanum 正在前往配送点 D"
    assert mecanum_heading_delivery_d["log"]["detail"]["robot_id"] == "mecanum"
    assert ackermann_heading_delivery_c["log"]["message"] == "ackermann 正在前往配送点 C"
    assert ackermann_heading_inspection["log"]["message"] == "ackermann 正在前往巡检点 P2"
    assert ackermann_heading_inspection["log"]["detail"]["point_id"] == "P2"
    assert mecanum_heading_inspection_p3["log"]["event_type"] == "demo_mecanum_heading_inspection_p3"
    assert mecanum_heading_inspection_p3["log"]["message"] == "mecanum 正在前往巡检点 P3"
    assert mecanum_heading_inspection_p3["log"]["detail"]["point_id"] == "P3"
    assert fire["log"]["event_type"] == "demo_fire_alert"
    assert fire["log"]["level"] == "error"
    assert fire["log"]["detail"]["point_id"] == "P2"
    assert fire["log"]["detail"]["warning_active"] is True
    assert fire["log"]["detail"]["warning_message"] == "P2 火灾警告"
    assert cleared["log"]["event_type"] == "demo_fire_alert_cleared"
    assert cleared["log"]["detail"]["warning_active"] is False
    assert inspection_started["log"]["message"] == "ackermann 已确认到达巡检点 P1，并开始巡检"
    assert inspection_normal["log"]["message"] == "P1 巡检正常"
    assert inspection_normal["log"]["detail"]["warning_active"] is False
    assert inspection_normal["log"]["detail"]["warning_severity"] == "warning"
    assert ackermann_returning["log"]["message"] == "ackermann 当前任务已结束，正在返回等待区"
    assert mecanum_returning["log"]["message"] == "mecanum 当前任务已结束，正在返回等待区"
    assert [log["event_type"] for log in gateway.list_logs(limit=4)] == [
        "demo_mecanum_returning_home",
        "demo_ackermann_returning_home",
        "demo_inspection_normal",
        "demo_inspection_started",
    ]


def test_demo_start_no_longer_overrides_state_without_real_dispatch(tmp_path):
    gateway = make_gateway(tmp_path, FakeDispatch(online=False))

    assert gateway.health()["dispatch_online"] is False

    with pytest.raises(GatewayError) as missing:
        gateway.trigger_demo_event("start")

    assert missing.value.reason == "demo_event_not_found"
    assert gateway.health()["dispatch_online"] is False


def test_demo_recheck_warning_and_clear_logs(tmp_path):
    gateway = make_gateway(tmp_path, FakeDispatch())

    smoke = gateway.trigger_demo_event("5", {"point_id": "P1"})
    confirmed = gateway.trigger_demo_event("7", {"point_id": "P1"})

    assert smoke["log"]["event_type"] == "demo_recheck_smoke"
    assert smoke["log"]["message"] == "P1 发现烟雾异常，已分配 mecanum 前往复检"
    assert smoke["log"]["detail"]["warning_active"] is True
    assert smoke["log"]["detail"]["warning_severity"] == "warning"
    assert smoke["log"]["detail"]["warning_message"] == "P1 烟雾异常，mecanum 正在复检"
    assert confirmed["log"]["event_type"] == "demo_recheck_confirmed"
    assert confirmed["log"]["detail"]["warning_active"] is False
    assert confirmed["log"]["detail"]["warning_severity"] == "warning"

    gateway._log_system_event("system_start", "调度系统启动中")
    cleared = gateway.trigger_demo_event("space")

    assert cleared["deleted"] == 2
    assert [log["event_type"] for log in gateway.store.list_logs(limit=10)] == ["system_start"]


def test_demo_event_rejects_unknown_key(tmp_path):
    gateway = make_gateway(tmp_path, FakeDispatch())

    with pytest.raises(GatewayError) as unknown:
        gateway.trigger_demo_event("0")

    assert unknown.value.reason == "demo_event_not_found"


def test_demo_event_rejects_invalid_inspection_point(tmp_path):
    gateway = make_gateway(tmp_path, FakeDispatch())

    with pytest.raises(GatewayError) as invalid_point:
        gateway.trigger_demo_event("8", {"point_id": "P4"})

    assert invalid_point.value.reason == "invalid_demo_inspection_point"


def test_demo_event_logs_can_be_written_from_thread_after_direct_write(tmp_path):
    gateway = make_gateway(tmp_path, FakeDispatch())

    gateway.trigger_demo_event("8")

    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(gateway.trigger_demo_event, "7")
        try:
            cleared = future.result(timeout=2.0)
        except FutureTimeoutError:
            pytest.fail("demo event log write deadlocked from a worker thread")

    assert cleared["log"]["event_type"] == "demo_recheck_confirmed"


def test_system_control_start_restart_stop_uses_real_backend_without_virtual_state(tmp_path):
    system_control = FakeSystemControl()
    gateway = make_gateway(
        tmp_path,
        FakeDispatch(online=False),
        system_control=system_control,
    )

    assert gateway.health()["dispatch_online"] is False

    started = gateway.system_start({})
    assert started["status"]["status"] == "starting"
    assert started["message"] == "调度系统启动中"
    assert "state" not in started
    assert system_control.calls == ["start"]
    assert gateway.health()["dispatch_online"] is False
    assert gateway.state()["dispatch_status"]["online"] is False

    restarted = gateway.system_restart({})
    assert restarted["status"]["status"] == "starting"
    assert restarted["message"] == "调度系统重启中"
    assert "state" not in restarted
    assert system_control.calls == ["start", "restart"]
    assert gateway.health()["dispatch_online"] is False

    stopped = gateway.system_stop({})
    assert stopped["status"]["status"] == "stopped"
    assert "state" not in stopped
    assert system_control.calls == ["start", "restart", "stop"]
    assert gateway.health()["dispatch_online"] is False
    assert gateway.state()["dispatch_status"]["online"] is False
    assert [log["event"] for log in gateway.system_logs()["operation_logs"][:3]] == [
        "system_stop",
        "system_restart",
        "system_start",
    ]
