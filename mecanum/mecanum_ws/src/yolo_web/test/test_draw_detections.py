import numpy as np
import cv2
import inspect

from yolo_web.usb_camera_web_server import Detection, UsbCameraWebNode, build_index_html, draw_detections
from yolo_web.detect_box import BoxOrderResult, draw_box_order_result


def test_draw_detections_changes_image_pixels():
    image = np.zeros((80, 120, 3), dtype=np.uint8)
    det = Detection(10, 12, 70, 60, 0.91, 0, 'box', 'box_camera')
    drawn = draw_detections(image, [det])
    assert drawn.shape == image.shape
    assert not np.array_equal(drawn, image)


def test_draw_detections_does_not_render_total_debug_count(monkeypatch):
    texts = []

    def record_text(*args, **kwargs):
        texts.append(args[1])
        return None

    monkeypatch.setattr(cv2, "putText", record_text)
    image = np.zeros((80, 120, 3), dtype=np.uint8)
    det = Detection(10, 12, 70, 60, 0.91, 0, 'box', 'box_camera')

    draw_detections(image, [det])

    assert all("detections:" not in text for text in texts)


def test_draw_box_order_result_does_not_render_summary_banner_text(monkeypatch):
    texts = []

    def record_text(*args, **kwargs):
        texts.append(args[1])
        return None

    monkeypatch.setattr(cv2, "putText", record_text)
    image = np.zeros((80, 120, 3), dtype=np.uint8)

    draw_box_order_result(image, BoxOrderResult(True, "ok", 2, []))

    assert all("Boxes tidy" not in text for text in texts)
    assert all("Boxes abnormal" not in text for text in texts)
    assert all("rows=" not in text for text in texts)
    assert all("cols=" not in text for text in texts)


def test_index_html_hides_temporary_event_overlays():
    html = build_index_html().decode("utf-8")

    assert "arrivalOverlay" not in html
    assert "inspectionResultOverlay" not in html
    assert "temporaryAppReceiver" not in html


def test_yolo_web_node_does_not_upload_inspection_events():
    source = inspect.getsource(UsbCameraWebNode.__init__)

    assert "/inspection/event" not in source
    assert "create_publisher" not in source
    assert "inspection_event_callback=self._publish_inspection_event" not in source
