from yolo_web.detect_box import BoxOrderConfig, BoxOrderResult
import yolo_web.usb_camera_web_server as server
from yolo_web.usb_camera_web_server import Detection, SharedUsbCamera

import numpy as np


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


class DummyDetector:
    def detect(self, frame):
        assert frame.shape == (48, 64, 3)
        return [Detection(8, 8, 42, 38, 0.92, 0, "box", "box_camera")]


def test_usb_camera_runs_box_order_analysis_without_drawing_video_status(monkeypatch):
    calls = {}

    def fake_analyze(frame, detections, config):
        calls["frame_sum"] = int(frame.sum())
        calls["detections"] = detections
        calls["config"] = config
        return BoxOrderResult(True, "测试通过", 1, [])

    def fake_draw(frame, result):
        raise AssertionError("box-order overlay should not be drawn on the video frame")

    monkeypatch.setattr(server.cv2, "VideoCapture", lambda _device: DummyCapture())
    monkeypatch.setattr(server, "analyze_box_order", fake_analyze)
    monkeypatch.setattr(server, "draw_box_order_result", fake_draw)
    assert not hasattr(server, "draw_center_status_text")

    config = BoxOrderConfig()
    camera = SharedUsbCamera(
        "/dev/null",
        64,
        48,
        15,
        80,
        [DummyDetector()],
        enable_box_order_check=True,
        box_order_config=config,
    )

    camera.open()
    camera._video_status_text = "已到达巡检点 P1"
    jpeg = camera.read_jpeg()

    assert jpeg.startswith(b"\xff\xd8")
    assert calls["frame_sum"] == 0
    assert calls["detections"][0].label == "box"
    assert calls["config"] is config


def test_box_order_log_skips_zero_boxes_and_formats_status():
    info_messages = []
    warn_messages = []
    camera = SharedUsbCamera(
        "/dev/null",
        64,
        48,
        15,
        80,
        [],
        enable_box_order_check=True,
        box_order_config=BoxOrderConfig(log_interval_sec=0),
        box_order_log_callback=info_messages.append,
        box_order_warn_callback=warn_messages.append,
    )

    camera._log_box_order_result(BoxOrderResult(True, "未检测到箱子", 0, []))
    assert info_messages == []
    assert warn_messages == []

    camera._log_box_order_result(BoxOrderResult(True, "箱子中心点组成规则行列", 3, [], 1, 3))
    assert info_messages == ["检测到3个箱子 箱子摆放状态：正常"]
    assert warn_messages == []
    assert "行" not in info_messages[0]
    assert "列" not in info_messages[0]

    camera._log_box_order_result(BoxOrderResult(False, "第2个箱子斜放", 3, []))
    assert warn_messages == ["检测到3个箱子 箱子摆放状态：异常"]


def test_box_order_warn_is_not_blocked_by_recent_info_log():
    info_messages = []
    warn_messages = []
    camera = SharedUsbCamera(
        "/dev/null",
        64,
        48,
        15,
        80,
        [],
        enable_box_order_check=True,
        box_order_config=BoxOrderConfig(log_interval_sec=10),
        box_order_log_callback=info_messages.append,
        box_order_warn_callback=warn_messages.append,
    )

    camera._log_box_order_result(BoxOrderResult(True, "箱子中心点组成规则行列", 2, [], 1, 2))
    camera._log_box_order_result(BoxOrderResult(False, "两个箱子中心点未水平或竖直对齐", 2, []))

    assert info_messages == ["检测到2个箱子 箱子摆放状态：正常"]
    assert warn_messages == ["检测到2个箱子 箱子摆放状态：异常"]


def test_box_order_log_only_emits_when_count_or_status_changes():
    info_messages = []
    warn_messages = []
    camera = SharedUsbCamera(
        "/dev/null",
        64,
        48,
        15,
        80,
        [],
        enable_box_order_check=True,
        box_order_config=BoxOrderConfig(log_interval_sec=10),
        box_order_log_callback=info_messages.append,
        box_order_warn_callback=warn_messages.append,
    )

    camera._log_box_order_result(BoxOrderResult(False, "异常", 2, []))
    camera._log_box_order_result(BoxOrderResult(False, "仍然异常", 2, []))
    camera._log_box_order_result(BoxOrderResult(True, "摆正", 1, []))
    camera._log_box_order_result(BoxOrderResult(True, "仍然摆正", 1, []))

    assert warn_messages == ["检测到2个箱子 箱子摆放状态：异常"]
    assert info_messages == ["检测到1个箱子 箱子摆放状态：正常"]


def test_box_order_zero_boxes_resets_log_state():
    info_messages = []
    warn_messages = []
    camera = SharedUsbCamera(
        "/dev/null",
        64,
        48,
        15,
        80,
        [],
        enable_box_order_check=True,
        box_order_config=BoxOrderConfig(log_interval_sec=10),
        box_order_log_callback=info_messages.append,
        box_order_warn_callback=warn_messages.append,
    )

    camera._log_box_order_result(BoxOrderResult(False, "异常", 2, []))
    camera._log_box_order_result(BoxOrderResult(True, "未检测到箱子", 0, []))
    camera._log_box_order_result(BoxOrderResult(False, "再次异常", 2, []))

    assert warn_messages == [
        "检测到2个箱子 箱子摆放状态：异常",
        "检测到2个箱子 箱子摆放状态：异常",
    ]
    assert info_messages == []


def test_box_order_info_and_warn_callbacks_are_called_from_separate_branches():
    calls = []
    camera = SharedUsbCamera(
        "/dev/null",
        64,
        48,
        15,
        80,
        [],
        enable_box_order_check=True,
        box_order_config=BoxOrderConfig(log_interval_sec=0),
        box_order_log_callback=lambda message: calls.append(("info", message)),
        box_order_warn_callback=lambda message: calls.append(("warn", message)),
    )

    assert camera._log_box_order_message("正常消息", warn=False) is True
    assert camera._log_box_order_message("异常消息", warn=True) is True
    assert calls == [("info", "正常消息"), ("warn", "异常消息")]


def test_box_order_state_is_not_updated_if_log_callback_fails():
    info_messages = []
    attempts = []

    def failing_warn(_message):
        attempts.append("warn")
        raise RuntimeError("warn failed")

    camera = SharedUsbCamera(
        "/dev/null",
        64,
        48,
        15,
        80,
        [],
        enable_box_order_check=True,
        box_order_config=BoxOrderConfig(log_interval_sec=0),
        box_order_log_callback=info_messages.append,
        box_order_warn_callback=failing_warn,
    )

    for _ in range(2):
        try:
            camera._log_box_order_result(BoxOrderResult(False, "异常", 2, []))
        except RuntimeError:
            pass

    assert attempts == ["warn", "warn"]
    assert info_messages == []


def test_hazard_log_warns_for_fire_and_smoke_once_per_active_state():
    warn_messages = []
    camera = SharedUsbCamera(
        "/dev/null",
        64,
        48,
        15,
        80,
        [],
        enable_box_order_check=True,
        box_order_config=BoxOrderConfig(log_interval_sec=10),
        box_order_warn_callback=warn_messages.append,
    )

    fire = Detection(1, 1, 10, 10, 0.9, 0, "fire", "fire_smoke")
    smoke = Detection(12, 12, 20, 20, 0.8, 1, "smoke", "fire_smoke")

    camera._log_hazard_detections([fire])
    camera._log_hazard_detections([fire])
    camera._log_hazard_detections([fire, smoke])
    camera._log_hazard_detections([fire, smoke])

    assert warn_messages == ["检测到疑似火焰", "检测到疑似烟雾"]


def test_hazard_log_resets_after_hazard_disappears():
    warn_messages = []
    camera = SharedUsbCamera(
        "/dev/null",
        64,
        48,
        15,
        80,
        [],
        enable_box_order_check=True,
        box_order_config=BoxOrderConfig(log_interval_sec=10),
        box_order_warn_callback=warn_messages.append,
    )

    fire = Detection(1, 1, 10, 10, 0.9, 0, "fire", "fire_smoke")

    camera._log_hazard_detections([fire])
    camera._log_hazard_detections([])
    camera._log_hazard_detections([fire])

    assert warn_messages == ["检测到疑似火焰", "检测到疑似火焰"]


class FakeVector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class FakeTwist:
    def __init__(self, linear_x=0.0, angular_z=0.0):
        self.linear = FakeVector(x=linear_x)
        self.angular = FakeVector(z=angular_z)


def make_event_camera(published, stop_hold_sec=1.0, raw_event_interval_sec=0):
    return SharedUsbCamera(
        "/dev/null",
        64,
        48,
        15,
        80,
        [],
        box_order_config=BoxOrderConfig(log_interval_sec=0),
        inspection_event_callback=published.append,
        robot_id="robot_a",
        raw_event_interval_sec=raw_event_interval_sec,
        stop_hold_sec=stop_hold_sec,
        stop_epsilon=0.001,
    )


def set_motion(camera, monkeypatch, linear_x=0.0, angular_z=0.0, now=0.0):
    monkeypatch.setattr(server.time, "monotonic", lambda: now)
    camera.update_motion_state(FakeTwist(linear_x=linear_x, angular_z=angular_z))


def report_arrival(camera, monkeypatch, detections, start=0.0, stop_hold_sec=1.0):
    set_motion(camera, monkeypatch, now=start)
    camera._publish_raw_detection_event(detections, None)
    set_motion(camera, monkeypatch, now=start + stop_hold_sec + 0.1)
    return camera._publish_raw_detection_event(detections, None)


def enable_inspection(camera, monkeypatch, detections, move_start=1.2, stop_start=1.4, ready=2.5):
    set_motion(camera, monkeypatch, linear_x=0.2, now=move_start)
    camera._publish_raw_detection_event(detections, None)
    set_motion(camera, monkeypatch, now=stop_start)
    camera._publish_raw_detection_event(detections, None)
    set_motion(camera, monkeypatch, now=ready)


def test_raw_event_suppresses_app_output_before_point_detection():
    published = []
    camera = make_event_camera(published, stop_hold_sec=0)
    detections = [
        Detection(1, 1, 10, 10, 0.9, 0, "box", "box_camera"),
        Detection(2, 2, 20, 20, 0.88, 0, "fire", "fire_smoke"),
        Detection(3, 3, 30, 30, 0.87, 1, "smoke", "fire_smoke"),
        Detection(4, 4, 40, 40, 0.86, 12, "shelf", "box_camera"),
    ]

    assert camera._publish_raw_detection_event(detections, None) is False
    assert published == []


def test_point_context_maps_letters_to_pickup_delivery_and_numbers_to_patrol():
    camera = make_event_camera([], stop_hold_sec=0)

    assert camera._point_context_from_label("letter_A", 0.9).point_type == "pickup"
    assert camera._point_context_from_label("letter_B", 0.9).point_type == "pickup"
    assert camera._point_context_from_label("letter_C", 0.9).point_type == "delivery"
    assert camera._point_context_from_label("letter_D", 0.9).point_type == "delivery"

    point = camera._point_context_from_label("number_1", 0.9)
    assert point.point_type == "patrol"
    assert point.point_id == "P1"
    assert camera._point_context_from_label("number_4", 0.9).point_id == "P4"


def test_patrol_video_status_stays_empty(monkeypatch):
    published = []
    camera = make_event_camera(published, stop_hold_sec=1.0)
    point = [Detection(1, 1, 10, 10, 0.9, 2, "number_1", "box_camera")]
    fire = [Detection(2, 2, 20, 20, 0.88, 0, "fire", "fire_smoke")]

    report_arrival(camera, monkeypatch, point)
    assert camera._video_status_text == ""

    set_motion(camera, monkeypatch, linear_x=0.2, now=1.2)
    assert camera._video_status_text == ""

    enable_inspection(camera, monkeypatch, fire)
    assert camera._publish_raw_detection_event(fire, None) is True
    assert camera._video_status_text == ""


def test_patrol_point_arrival_waits_for_first_stable_stop(monkeypatch):
    import json

    published = []
    camera = make_event_camera(published, stop_hold_sec=1.0)
    detections = [Detection(1, 1, 10, 10, 0.9, 2, "number_1", "box_camera")]

    set_motion(camera, monkeypatch, linear_x=0.4, now=0.0)
    assert camera._publish_raw_detection_event(detections, None) is False
    assert published == []

    set_motion(camera, monkeypatch, linear_x=0.0, now=1.0)
    assert camera._publish_raw_detection_event(detections, None) is False
    assert published == []

    assert camera._publish_raw_detection_event(detections, None) is False
    assert published == []

    set_motion(camera, monkeypatch, linear_x=0.0, now=2.1)
    assert camera._publish_raw_detection_event(detections, None) is True

    event = json.loads(published[-1])
    assert event["event_type"] == "arrival"
    assert event["event_code"] == "PATROL_POINT_P1"
    assert event["message"] == ""


def test_patrol_inspection_waits_for_move_and_second_stable_stop(monkeypatch):
    import json

    published = []
    camera = make_event_camera(published, stop_hold_sec=1.0)
    letter = [Detection(1, 1, 10, 10, 0.9, 3, "number_2", "box_camera")]
    fire = [Detection(2, 2, 20, 20, 0.88, 0, "fire", "fire_smoke")]

    report_arrival(camera, monkeypatch, letter)
    assert len(published) == 1

    set_motion(camera, monkeypatch, linear_x=0.2, now=1.2)
    assert camera._publish_raw_detection_event(fire, None) is False
    assert len(published) == 1

    set_motion(camera, monkeypatch, linear_x=0.0, now=1.4)
    assert camera._publish_raw_detection_event(fire, None) is False
    assert len(published) == 1

    set_motion(camera, monkeypatch, linear_x=0.0, now=2.5)
    assert camera._publish_raw_detection_event(fire, None) is True

    event = json.loads(published[-1])
    assert event["patrol_point"] == "P2"
    assert event["event_type"] == "fire"
    assert event["message"] == "检测到疑似火焰"


def test_point_context_does_not_switch_while_moving_to_target(monkeypatch):
    import json

    published = []
    camera = make_event_camera(published, stop_hold_sec=1.0)
    letter = [Detection(1, 1, 10, 10, 0.9, 2, "number_1", "box_camera")]
    stray_number = [Detection(1, 1, 10, 10, 0.95, 7, "letter_A", "box_camera")]
    fire = [Detection(2, 2, 20, 20, 0.88, 0, "fire", "fire_smoke")]

    report_arrival(camera, monkeypatch, letter)
    set_motion(camera, monkeypatch, linear_x=0.2, now=1.2)
    assert camera._publish_raw_detection_event(stray_number, None) is False
    set_motion(camera, monkeypatch, now=1.4)
    assert camera._publish_raw_detection_event(fire, None) is False
    set_motion(camera, monkeypatch, now=2.5)
    assert camera._publish_raw_detection_event(fire, None) is True

    event = json.loads(published[-1])
    assert event["patrol_point"] == "P1"
    assert event["event_type"] == "fire"
    assert event["event_code"] == "FIRE_DETECTED"


def test_pickup_point_reports_shelf_after_second_stable_stop(monkeypatch):
    import json

    published = []
    camera = make_event_camera(published, stop_hold_sec=1.0)
    point = [Detection(1, 1, 10, 10, 0.9, 7, "letter_A", "box_camera")]
    shelf = [Detection(2, 2, 20, 20, 0.91, 1, "shelf", "box_camera")]

    report_arrival(camera, monkeypatch, point)
    arrival = json.loads(published[-1])
    assert arrival["event_type"] == "arrival"
    assert arrival["event_code"] == "PICKUP_POINT_A"
    assert arrival["message"] == ""
    assert arrival["task_point_type"] == "pickup"

    set_motion(camera, monkeypatch, linear_x=0.2, now=1.2)
    assert camera._publish_raw_detection_event(shelf, None) is False
    set_motion(camera, monkeypatch, now=1.4)
    assert camera._publish_raw_detection_event(shelf, None) is False
    set_motion(camera, monkeypatch, now=2.5)
    assert camera._publish_raw_detection_event(shelf, None) is True

    event = json.loads(published[-1])
    assert event["event_type"] == "shelf"
    assert event["event_code"] == "SHELF_PICKUP_READY"
    assert event["message"] == "已检测到货架，开始检货"


def test_delivery_point_reports_shelf_after_second_stable_stop(monkeypatch):
    import json

    published = []
    camera = make_event_camera(published, stop_hold_sec=1.0)
    point = [Detection(1, 1, 10, 10, 0.9, 7, "letter_C", "box_camera")]
    shelf = [Detection(2, 2, 20, 20, 0.91, 1, "shelf", "box_camera")]

    report_arrival(camera, monkeypatch, point)
    arrival = json.loads(published[-1])
    assert arrival["event_code"] == "DELIVERY_POINT_C"
    assert arrival["message"] == ""
    assert arrival["task_point_type"] == "delivery"

    set_motion(camera, monkeypatch, linear_x=0.2, now=1.2)
    assert camera._publish_raw_detection_event(shelf, None) is False
    set_motion(camera, monkeypatch, now=1.4)
    assert camera._publish_raw_detection_event(shelf, None) is False
    set_motion(camera, monkeypatch, now=2.5)
    assert camera._publish_raw_detection_event(shelf, None) is True

    event = json.loads(published[-1])
    assert event["event_code"] == "SHELF_DELIVERY_READY"
    assert event["message"] == "已检测到货架，开始发货"


def test_fire_hydrant_does_not_publish_app_event(monkeypatch):
    published = []
    camera = make_event_camera(published, stop_hold_sec=0)
    point = [Detection(1, 1, 10, 10, 0.9, 2, "number_4", "box_camera")]
    hydrant = [Detection(2, 2, 20, 20, 0.91, 12, "Fire-hydrant", "box_camera")]

    report_arrival(camera, monkeypatch, point, stop_hold_sec=0)
    set_motion(camera, monkeypatch, linear_x=0.2, now=0.1)
    camera._publish_raw_detection_event(hydrant, None)
    set_motion(camera, monkeypatch, now=0.2)

    assert camera._publish_raw_detection_event(hydrant, None) is False
    assert len(published) == 1


def test_raw_event_reports_patrol_point_arrival_on_letter_detection(monkeypatch):
    published = []
    camera = make_event_camera(published, stop_hold_sec=1.0)
    detections = [Detection(1, 1, 10, 10, 0.9, 2, "number_1", "box_camera")]

    assert report_arrival(camera, monkeypatch, detections) is True

    import json
    event = json.loads(published[-1])
    assert event["robot_id"] == "robot_a"
    assert event["patrol_point"] == "P1"
    assert event["location"] == "巡检点 P1"
    assert event["level"] == "info"
    assert event["status"] == "arrived"
    assert event["event_type"] == "arrival"
    assert event["event_code"] == "PATROL_POINT_P1"
    assert event["message"] == ""


def test_raw_event_reports_hazard_after_arrival(monkeypatch):
    published = []
    camera = make_event_camera(published, stop_hold_sec=1.0)
    point = [Detection(1, 1, 10, 10, 0.9, 2, "number_2", "box_camera")]
    fire = [Detection(2, 2, 20, 20, 0.88, 0, "fire", "fire_smoke")]

    report_arrival(camera, monkeypatch, point)
    enable_inspection(camera, monkeypatch, fire)
    camera._publish_raw_detection_event(fire, None)

    import json
    event = json.loads(published[-1])
    assert event["patrol_point"] == "P2"
    assert event["location"] == "巡检点 P2"
    assert event["level"] == "warn"
    assert event["status"] == "abnormal"
    assert event["event_type"] == "fire"
    assert event["event_code"] == "FIRE_DETECTED"
    assert event["message"] == "检测到疑似火焰"


def test_raw_event_keeps_letter_arrival_and_allows_later_inspection(monkeypatch):
    published = []
    camera = make_event_camera(published, stop_hold_sec=1.0)
    letter = [Detection(1, 1, 10, 10, 0.9, 2, "number_3", "box_camera")]
    box = [Detection(1, 1, 10, 10, 0.9, 0, "box", "box_camera")]

    report_arrival(camera, monkeypatch, letter)
    enable_inspection(camera, monkeypatch, box)
    camera._publish_raw_detection_event(box, None)

    import json
    arrival_event = json.loads(published[0])
    inspection_event = json.loads(published[1])
    assert arrival_event["event_type"] == "arrival"
    assert inspection_event["event_code"] == "NORMAL"
    assert inspection_event["status"] == "normal"


def test_patrol_inspection_keeps_publishing_raw_events_until_vehicle_moves(monkeypatch):
    import json

    published = []
    camera = make_event_camera(published, stop_hold_sec=1.0, raw_event_interval_sec=1.0)
    letter = [Detection(1, 1, 10, 10, 0.9, 2, "number_3", "box_camera")]
    box = [Detection(1, 1, 10, 10, 0.9, 0, "box", "box_camera")]

    report_arrival(camera, monkeypatch, letter)
    enable_inspection(camera, monkeypatch, box)

    for now in (2.5, 3.6, 4.7):
        monkeypatch.setattr(server.time, "monotonic", lambda now=now: now)
        assert camera._publish_raw_detection_event(box, None) is True

    events = [json.loads(message) for message in published]
    inspection_events = [event for event in events if event["event_type"] == "inspection"]
    assert len(inspection_events) == 3
    assert all(event["patrol_point"] == "P3" for event in inspection_events)
    assert camera._current_point is not None
    assert camera._point_state == server.POINT_INSPECTION_ENABLED


def test_raw_event_rate_limits_to_one_second(monkeypatch):
    published = []
    camera = make_event_camera(published, stop_hold_sec=0, raw_event_interval_sec=1.0)
    detections = [Detection(1, 1, 10, 10, 0.9, 2, "number_3", "box_camera")]

    set_motion(camera, monkeypatch, now=0.0)
    camera._publish_raw_detection_event(detections, None)
    camera._publish_raw_detection_event(detections, None)

    assert len(published) == 1
