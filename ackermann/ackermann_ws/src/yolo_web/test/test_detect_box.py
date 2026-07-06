from types import SimpleNamespace

import cv2
import numpy as np

from yolo_web.detect_box import BoxOrderConfig, analyze_box_order


def _det(x1, y1, x2, y2, label='box', score=0.9, model_name='box_camera'):
    return SimpleNamespace(x1=x1, y1=y1, x2=x2, y2=y2, label=label, score=score, class_id=0, model_name=model_name)


def _draw_detection_boxes(frame, detections):
    for det in detections:
        cv2.rectangle(frame, (int(det.x1) + 4, int(det.y1) + 4), (int(det.x2) - 4, int(det.y2) - 4), (255, 255, 255), -1)
    return frame


def _frame_with_rotated_box(angle_deg):
    frame = np.zeros((180, 220, 3), dtype=np.uint8)
    rect = ((110, 90), (100, 50), angle_deg)
    points = cv2.boxPoints(rect).astype(np.int32)
    cv2.drawContours(frame, [points], 0, (255, 255, 255), -1)
    return frame


def test_marks_single_tilted_box_from_roi_contour():
    frame = _frame_with_rotated_box(24)
    result = analyze_box_order(frame, [_det(45, 35, 175, 145)], BoxOrderConfig(angle_thresh_deg=10.0, min_contour_area_ratio=0.02))
    assert result.box_count == 1
    assert result.is_tidy is False
    assert result.boxes[0].is_tilted is True
    assert '斜放' in result.reason


def test_accepts_incomplete_regular_grid():
    frame = np.zeros((260, 320, 3), dtype=np.uint8)
    detections = [_det(20, 20, 80, 80), _det(120, 20, 180, 80), _det(220, 20, 280, 80), _det(20, 140, 80, 200), _det(120, 140, 180, 200)]
    frame = _draw_detection_boxes(frame, detections)
    result = analyze_box_order(frame, detections, BoxOrderConfig(row_align_thresh_px=25.0, col_align_thresh_px=25.0, spacing_tolerance_ratio=0.35, min_contour_area_ratio=0.02))
    assert result.box_count == 5
    assert result.is_tidy is True
    assert '规则' in result.reason


def test_rejects_misaligned_centers():
    frame = np.zeros((260, 320, 3), dtype=np.uint8)
    detections = [_det(20, 20, 80, 80), _det(122, 33, 182, 93), _det(225, 20, 285, 80), _det(20, 140, 80, 200)]
    frame = _draw_detection_boxes(frame, detections)
    result = analyze_box_order(frame, detections, BoxOrderConfig(row_align_thresh_px=8.0, col_align_thresh_px=8.0, spacing_tolerance_ratio=0.25, min_contour_area_ratio=0.02))
    assert result.box_count == 4
    assert result.is_tidy is False
    assert '行列' in result.reason or '不规则' in result.reason


def test_ignores_fire_smoke_model_detections():
    frame = np.zeros((120, 120, 3), dtype=np.uint8)
    result = analyze_box_order(frame, [_det(10, 10, 80, 80, label='fire', model_name='fire_smoke')], BoxOrderConfig())
    assert result.box_count == 0
    assert result.is_tidy is True


def test_estimates_angle_from_fragmented_box_edges():
    frame = np.zeros((200, 240, 3), dtype=np.uint8)
    rect = ((120, 100), (120, 70), 18)
    points = cv2.boxPoints(rect).astype(np.int32)
    for start, end in zip(points, np.roll(points, -1, axis=0)):
        for t0 in np.linspace(0.08, 0.78, 4):
            t1 = t0 + 0.08
            p0 = (start + (end - start) * t0).astype(np.int32)
            p1 = (start + (end - start) * t1).astype(np.int32)
            cv2.line(frame, tuple(p0), tuple(p1), (255, 255, 255), 2)

    result = analyze_box_order(
        frame,
        [_det(45, 45, 195, 155)],
        BoxOrderConfig(angle_thresh_deg=10.0, min_contour_area_ratio=0.03),
    )

    assert result.box_count == 1
    assert result.boxes[0].angle_status == 'ok'
    assert result.boxes[0].is_tilted is True
    assert '斜放' in result.reason
