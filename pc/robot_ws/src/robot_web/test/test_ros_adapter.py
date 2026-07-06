import sys
from types import ModuleType

from robot_web.dispatch_adapter import DispatchResult
from robot_web.ros_adapter import RosDispatchClient


class FakeCreateTask:
    class Request:
        def __init__(self):
            self.requester = ""
            self.task_type = 0
            self.note = ""
            self.preferred_robot_id = ""
            self.steps = []


class FakeMissionStep:
    STEP_NAVIGATE = 1

    def __init__(self):
        self.sequence = 0
        self.step_type = 0
        self.step_id = ""
        self.point_id = ""
        self.requires_confirmation = False
        self.resource_id = ""
        self.label = ""


def test_ros_create_task_sets_preferred_robot_field(monkeypatch):
    robot_interfaces = ModuleType("robot_interfaces")
    robot_interfaces_msg = ModuleType("robot_interfaces.msg")
    robot_interfaces_msg.MissionStep = FakeMissionStep
    monkeypatch.setitem(sys.modules, "robot_interfaces", robot_interfaces)
    monkeypatch.setitem(sys.modules, "robot_interfaces.msg", robot_interfaces_msg)

    client = RosDispatchClient.__new__(RosDispatchClient)
    client._services = {"create": FakeCreateTask}
    captured = {}

    def fake_call(key, request):
        captured["key"] = key
        captured["request"] = request
        return DispatchResult(True, "created")

    client._call = fake_call

    result = client.create_task(
        "DELIVERY",
        ["PICKUP_A", "DELIVERY_C"],
        requester="tablet",
        note="robot_web template_id=custom_delivery",
        preferred_robot_id="ackermann",
    )

    request = captured["request"]
    assert result.accepted is True
    assert captured["key"] == "create"
    assert request.requester == "tablet"
    assert request.task_type == 1
    assert request.note == "robot_web template_id=custom_delivery"
    assert request.preferred_robot_id == "ackermann"
    assert [step.point_id for step in request.steps] == ["PICKUP_A", "DELIVERY_C"]
