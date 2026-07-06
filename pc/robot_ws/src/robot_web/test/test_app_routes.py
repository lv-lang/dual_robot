from types import SimpleNamespace

import pytest

from robot_web.dispatch_adapter import DispatchResult, DispatchSnapshot
from robot_web.gateway import RobotWebGateway
from robot_web.models import TaskPointInfo
from robot_web.storage import RobotWebStore


class RouteFakeDispatch:
    def __init__(self):
        self.points = {
            "PICKUP_A": TaskPointInfo("PICKUP_A", "PICKUP", "A 取货点"),
            "DELIVERY_C": TaskPointInfo("DELIVERY_C", "DELIVERY", "C 配送点"),
        }

    def is_online(self, timeout_sec=0.1):
        return True

    def get_state(self):
        return DispatchResult(True, "ok", data=DispatchSnapshot(message="ok"))

    def get_task_points(self):
        return DispatchResult(True, "ok", data=self.points)

    def create_task(self, task_type, point_ids, requester, note="", preferred_robot_id="auto"):
        return DispatchResult(True, "created", data=SimpleNamespace(task_id="task_1"))

    def confirm_task(self, task_id, result, requester, step_index=0, step_id="", point_id="", note=""):
        return DispatchResult(True, "confirmed", data=SimpleNamespace(derived_task_id=""))

    def pause_task(self, task_id, requester, reason=""):
        return DispatchResult(True, "paused")

    def resume_task(self, task_id, requester, reason=""):
        return DispatchResult(True, "resumed")

    def cancel_task(self, task_id, requester, reason=""):
        return DispatchResult(True, "canceled")

    def emergency_stop(self, requester, reason=""):
        return DispatchResult(True, "emergency stop active")


def system_status(status="stopped", managed=False, external=False):
    return {
        "status": status,
        "summary": "调度系统未启动" if status == "stopped" else "调度系统启动中",
        "managed": managed,
        "external_running": external,
        "can_start": status == "stopped" and not external,
        "can_stop": managed and not external,
        "can_restart": managed and not external,
        "profile": {
            "id": "real_robot_control_plane",
            "name": "Real Robot Control Plane",
            "command": "ros2 launch robot_bringup real_robot_control_plane.launch.py launch_rviz:=true",
        },
        "pid": 1234 if managed else None,
        "pgid": 1234 if managed else None,
        "started_at": "2026-05-23T00:00:00+00:00" if managed else "",
        "updated_at": "2026-05-23T00:00:01+00:00",
        "health": [{
            "id": "process.map_server",
            "label": "PC map_server",
            "category": "process",
            "status": "missing",
            "required": True,
            "detail": "no matching process",
        }],
    }


class RouteFakeSystemControl:
    def __init__(self):
        self.state = system_status()
        self.event_sink = None

    def set_event_sink(self, event_sink):
        self.event_sink = event_sink

    def status(self):
        return self.state

    def start(self):
        self.state = system_status("starting", managed=True)
        return self.state

    def stop(self):
        self.state = system_status()
        return self.state

    def restart(self):
        self.state = system_status("starting", managed=True)
        return self.state

    def logs(self, limit=120):
        return [{
            "line_no": 1,
            "stream": "launch",
            "message": "fake launch ready",
            "timestamp": "",
        }]


def test_http_routes_expose_health_templates_trigger_and_root(tmp_path):
    pytest.importorskip("fastapi")
    pytest.importorskip("httpx")
    from fastapi.testclient import TestClient
    from robot_web.app import create_app

    gateway = RobotWebGateway(
        store=RobotWebStore(tmp_path / "robot_web.sqlite3"),
        dispatch_client=RouteFakeDispatch(),
        builtin_templates=[{
            "template_id": "builtin_delivery_demo",
            "display_name": "配送演示",
            "task_type": "DELIVERY",
            "target_point_ids": ["PICKUP_A", "DELIVERY_C"],
            "robot_preference": "auto",
            "builtin": True,
            "sort_order": 10,
            "created_at": "now",
            "updated_at": "now",
        }],
        fallback_points={},
        system_control=RouteFakeSystemControl(),
    )
    client = TestClient(create_app(gateway, frontend_dist=None))

    health = client.get("/api/health")
    assert health.status_code == 200
    assert health.json()["dispatch_online"] is True
    assert health.json()["dispatch_degraded"] is False
    assert health.json()["updated_at"]

    templates = client.get("/api/templates")
    assert templates.status_code == 200
    assert templates.json()["templates"][0]["builtin"] is True
    assert templates.json()["templates"][0]["name"] == "配送演示"
    assert templates.json()["templates"][0]["target_points"] == ["PICKUP_A", "DELIVERY_C"]
    assert templates.json()["templates"][0]["readonly"] is True
    assert templates.json()["business_points"][0]["point_type"] in {"delivery", "pickup"}

    trigger = client.post("/api/templates/builtin_delivery_demo/trigger", json={})
    assert trigger.status_code == 200
    assert trigger.json()["task_id"] == "task_1"
    assert trigger.json()["display_name"] == "配送任务_1"
    assert trigger.json()["preferred_robot_id"] == "auto"

    root = client.get("/")
    assert root.status_code == 200
    assert root.json()["frontend_dist_present"] is False


def test_camera_route_reads_configured_streams(tmp_path):
    pytest.importorskip("fastapi")
    pytest.importorskip("httpx")
    from fastapi.testclient import TestClient
    from robot_web.app import create_app

    cameras_file = tmp_path / "cameras.yaml"
    cameras_file.write_text(
        """
cameras:
  - robot_id: mecanum
    label: mecanum车检测画面
    stream_url: "http://172.20.10.12:8088/stream.mjpg"
    kind: mjpeg
  - robot_id: ackermann
    label: ackermann车检测画面
    stream_url: "http://172.20.10.13:8088/stream.mjpg"
    kind: mjpeg
""".strip(),
        encoding="utf-8",
    )
    gateway = RobotWebGateway(
        store=RobotWebStore(tmp_path / "robot_web.sqlite3"),
        dispatch_client=RouteFakeDispatch(),
        builtin_templates=[],
        fallback_points={},
        system_control=RouteFakeSystemControl(),
    )
    client = TestClient(create_app(gateway, frontend_dist=None, cameras_file=str(cameras_file)))

    response = client.get("/api/cameras")

    assert response.status_code == 200
    assert response.json() == {
        "cameras": [
            {
                "robot_id": "mecanum",
                "label": "mecanum车检测画面",
                "stream_url": "http://172.20.10.12:8088/stream.mjpg",
                "kind": "mjpeg",
            },
            {
                "robot_id": "ackermann",
                "label": "ackermann车检测画面",
                "stream_url": "http://172.20.10.13:8088/stream.mjpg",
                "kind": "mjpeg",
            },
        ]
    }


def test_system_control_routes_follow_contract_and_log_operations(tmp_path):
    pytest.importorskip("fastapi")
    pytest.importorskip("httpx")
    from fastapi.testclient import TestClient
    from robot_web.app import create_app

    gateway = RobotWebGateway(
        store=RobotWebStore(tmp_path / "robot_web.sqlite3"),
        dispatch_client=RouteFakeDispatch(),
        builtin_templates=[],
        fallback_points={},
        system_control=RouteFakeSystemControl(),
    )
    client = TestClient(create_app(gateway, frontend_dist=None))

    status = client.get("/api/system/status")
    assert status.status_code == 200
    assert status.json()["status"] == "stopped"
    assert status.json()["profile"]["command"] == (
        "ros2 launch robot_bringup real_robot_control_plane.launch.py launch_rviz:=true"
    )

    start = client.post("/api/system/start", json={})
    assert start.status_code == 200
    assert start.json()["accepted"] is True
    assert start.json()["status"]["managed"] is True
    assert start.json()["log"]["event"] == "system_start"

    logs = client.get("/api/system/logs?limit=120")
    assert logs.status_code == 200
    assert logs.json()["launch_logs"][0]["message"] == "fake launch ready"
    assert logs.json()["operation_logs"][0]["event"] == "system_start"


def test_demo_event_route_writes_competition_logs(tmp_path):
    pytest.importorskip("fastapi")
    httpx = pytest.importorskip("httpx")
    from robot_web.app import create_app

    gateway = RobotWebGateway(
        store=RobotWebStore(tmp_path / "robot_web.sqlite3"),
        dispatch_client=RouteFakeDispatch(),
        builtin_templates=[],
        fallback_points={},
        system_control=RouteFakeSystemControl(),
    )
    app = create_app(gateway, frontend_dist=None)
    transport = httpx.ASGITransport(app=app)

    async def run_requests():
        async with httpx.AsyncClient(transport=transport, base_url="http://testserver") as client:
            fire_response = await client.post("/api/demo-events/8", json={"point_id": "P1"})
            cleared_response = await client.post("/api/demo-events/9", json={"point_id": "P1"})
            clear_logs_response = await client.post("/api/demo-events/space")
            start_response = await client.post("/api/demo-events/start")
            return fire_response, cleared_response, clear_logs_response, start_response

    import asyncio
    fire, cleared, clear_logs, start = asyncio.run(run_requests())

    assert fire.status_code == 200
    assert fire.json()["log"]["event"] == "demo_fire_alert"
    assert fire.json()["log"]["message"] == "发现当前巡检点 P1 有火情"
    assert fire.json()["log"]["detail"]["point_id"] == "P1"
    assert fire.json()["log"]["detail"]["warning_active"] is True
    assert cleared.status_code == 200
    assert cleared.json()["log"]["event"] == "demo_fire_alert_cleared"
    assert cleared.json()["log"]["detail"]["point_id"] == "P1"
    assert cleared.json()["log"]["detail"]["warning_active"] is False
    assert clear_logs.status_code == 200
    assert clear_logs.json()["logs"] == []
    assert start.status_code == 404
    assert start.json()["reason"] == "demo_event_not_found"


def test_system_start_rejects_command_payload(tmp_path):
    pytest.importorskip("fastapi")
    pytest.importorskip("httpx")
    from fastapi.testclient import TestClient
    from robot_web.app import create_app

    gateway = RobotWebGateway(
        store=RobotWebStore(tmp_path / "robot_web.sqlite3"),
        dispatch_client=RouteFakeDispatch(),
        builtin_templates=[],
        fallback_points={},
        system_control=RouteFakeSystemControl(),
    )
    client = TestClient(create_app(gateway, frontend_dist=None))

    response = client.post("/api/system/start", json={"command": "echo unsafe"})

    assert response.status_code == 400
    assert response.json()["reason"] == "system_command_not_allowed"
