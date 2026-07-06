import numpy as np

import yolo_web.usb_camera_web_server as server
from yolo_web.usb_camera_web_server import Detection, SharedUsbCamera
from yolo_web.vision_activation_gate import VisionActivationGate, point_type_from_id


class DummyCapture:
    def __init__(self):
        self.opened = True
        self.frame = np.zeros((48, 64, 3), dtype=np.uint8)

    def set(self, *_args):
        return True

    def isOpened(self):
        return self.opened

    def read(self):
        return True, self.frame.copy()

    def release(self):
        self.opened = False


class CountingDetector:
    def __init__(self):
        self.calls = 0

    def detect(self, frame):
        self.calls += 1
        return [Detection(8, 8, 42, 38, 0.92, 0, "box", "box_camera")]


def test_point_type_from_id_only_activates_task_points():
    assert point_type_from_id("PICKUP_A") == "pickup"
    assert point_type_from_id("DELIVERY_C") == "delivery"
    assert point_type_from_id("P1") == "inspection"
    assert point_type_from_id("W1") is None
    assert point_type_from_id("G1") is None


def test_activation_gate_uses_enter_exit_hysteresis():
    gate = VisionActivationGate(enter_radius=0.5, exit_radius=0.7)
    assert gate.update_target("P1", 0.0, 0.0, task_id="task-1", step_id="step-1")

    gate.update_pose(0.51, 0.0)
    assert gate.snapshot().active is False

    gate.update_pose(0.5, 0.0)
    assert gate.snapshot().active is True

    gate.update_pose(0.69, 0.0)
    assert gate.snapshot().active is True

    gate.update_pose(0.71, 0.0)
    assert gate.snapshot().active is False


def test_activation_gate_ignores_waiting_area_targets():
    gate = VisionActivationGate()
    assert gate.update_target("W1", 0.0, 0.0) is False
    gate.update_pose(0.0, 0.0)
    snapshot = gate.snapshot()
    assert snapshot.active is False
    assert snapshot.target is None


def test_camera_skips_detector_when_activation_gate_is_closed(monkeypatch):
    detector = CountingDetector()
    gate = VisionActivationGate()
    gate.update_target("PICKUP_A", 0.0, 0.0, task_id="task-1", step_id="step-1")
    gate.update_pose(1.0, 0.0)
    monkeypatch.setattr(server.cv2, "VideoCapture", lambda _device: DummyCapture())

    camera = SharedUsbCamera(
        "/dev/null",
        64,
        48,
        15,
        80,
        [detector],
        vision_activation_gate=gate,
    )
    camera.open()

    jpeg = camera.read_jpeg()

    assert jpeg.startswith(b"\xff\xd8")
    assert detector.calls == 0


def test_camera_runs_detector_when_activation_gate_is_open(monkeypatch):
    detector = CountingDetector()
    gate = VisionActivationGate()
    gate.update_target("PICKUP_A", 0.0, 0.0, task_id="task-1", step_id="step-1")
    gate.update_pose(0.4, 0.0)
    monkeypatch.setattr(server.cv2, "VideoCapture", lambda _device: DummyCapture())

    camera = SharedUsbCamera(
        "/dev/null",
        64,
        48,
        15,
        80,
        [detector],
        vision_activation_gate=gate,
    )
    camera.open()

    jpeg = camera.read_jpeg()

    assert jpeg.startswith(b"\xff\xd8")
    assert detector.calls == 1
