from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional

import cv2
import numpy as np

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


@dataclass
class BoxOrderConfig:
    """Parameters for single-box angle and multi-box grid checks."""

    box_class_names: list[str] = field(default_factory=lambda: ["box"])
    box_model_names: list[str] = field(default_factory=lambda: ["box_camera"])
    min_confidence: float = 0.0
    angle_thresh_deg: float = 12.0
    row_align_thresh_px: float = 42.0
    col_align_thresh_px: float = 42.0
    spacing_tolerance_ratio: float = 0.40
    min_contour_area_ratio: float = 0.03
    min_edge_points: int = 40
    min_edge_span_ratio: float = 0.25
    canny_low: int = 30
    canny_high: int = 120
    blur_kernel: int = 5
    log_interval_sec: float = 1.5


@dataclass(frozen=True)
class BoxItemResult:
    x1: float
    y1: float
    x2: float
    y2: float
    cx: float
    cy: float
    confidence: float
    angle_deg: Optional[float]
    angle_deviation_deg: Optional[float]
    is_tilted: bool
    angle_status: str
    reason: str = ""


@dataclass(frozen=True)
class BoxOrderResult:
    is_tidy: bool
    reason: str
    box_count: int
    boxes: list[BoxItemResult]
    row_count: int = 0
    col_count: int = 0


def load_box_order_config(path: str | None) -> BoxOrderConfig:
    cfg = BoxOrderConfig()
    if not path:
        return cfg
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) if yaml is not None else None
    except OSError:
        return cfg
    if not isinstance(data, dict):
        return cfg
    values = cfg.__dict__.copy()
    for key in values:
        if key in data:
            values[key] = data[key]
    for key in ("box_class_names", "box_model_names"):
        if isinstance(values.get(key), str):
            values[key] = [values[key]]
    return BoxOrderConfig(**values)


def is_box_detection(detection: Any, config: BoxOrderConfig) -> bool:
    label = str(getattr(detection, "label", ""))
    model_name = str(getattr(detection, "model_name", ""))
    confidence = float(getattr(detection, "score", getattr(detection, "confidence", 0.0)))
    if model_name and model_name not in set(config.box_model_names):
        return False
    return label in set(config.box_class_names) and confidence >= config.min_confidence


def analyze_box_order(frame: np.ndarray, detections: list[Any], config: BoxOrderConfig) -> BoxOrderResult:
    box_detections = [det for det in detections if is_box_detection(det, config)]
    boxes = [_analyze_single_box(frame, det, config) for det in box_detections]
    if not boxes:
        return BoxOrderResult(True, "未检测到箱子", 0, [])

    unknown = [(idx + 1, box) for idx, box in enumerate(boxes) if box.angle_status == "unknown"]
    if unknown:
        index, box = unknown[0]
        detail = f": {box.reason}" if box.reason else ""
        return BoxOrderResult(False, f"第{index}个箱子角度无法判断{detail}", len(boxes), boxes)

    tilted = [idx + 1 for idx, box in enumerate(boxes) if box.is_tilted]
    if tilted:
        return BoxOrderResult(False, f"第{tilted[0]}个箱子斜放", len(boxes), boxes)

    if len(boxes) == 1:
        return BoxOrderResult(True, "单个箱子未发现斜放", 1, boxes)

    if len(boxes) == 2:
        row_ok = abs(boxes[0].cy - boxes[1].cy) <= config.row_align_thresh_px
        col_ok = abs(boxes[0].cx - boxes[1].cx) <= config.col_align_thresh_px
        if row_ok or col_ok:
            direction = "水平" if row_ok else "竖直"
            return BoxOrderResult(True, f"两个箱子{direction}对齐", 2, boxes, 1 if row_ok else 2, 2 if row_ok else 1)
        return BoxOrderResult(False, "两个箱子中心点未水平或竖直对齐", 2, boxes)

    centers = np.array([[box.cx, box.cy] for box in boxes], dtype=np.float32)
    row_centers, row_ids = _cluster_axis(centers[:, 1], config.row_align_thresh_px)
    col_centers, col_ids = _cluster_axis(centers[:, 0], config.col_align_thresh_px)

    if _has_duplicate_cells(row_ids, col_ids):
        return BoxOrderResult(False, "箱子中心点落入重复行列单元，排列不规则", len(boxes), boxes, len(row_centers), len(col_centers))
    if not _spacing_is_regular(col_centers, config.spacing_tolerance_ratio):
        return BoxOrderResult(False, "相邻列间距不规则", len(boxes), boxes, len(row_centers), len(col_centers))
    if not _spacing_is_regular(row_centers, config.spacing_tolerance_ratio):
        return BoxOrderResult(False, "相邻行间距不规则", len(boxes), boxes, len(row_centers), len(col_centers))
    return BoxOrderResult(True, "箱子中心点组成规则行列", len(boxes), boxes, len(row_centers), len(col_centers))


def draw_box_order_result(frame: np.ndarray, result: BoxOrderResult) -> np.ndarray:
    drawn = frame.copy()
    for index, box in enumerate(result.boxes, start=1):
        x1, y1, x2, y2 = map(lambda value: int(round(value)), (box.x1, box.y1, box.x2, box.y2))
        if box.is_tilted:
            color = (0, 0, 255)
            state = "TILTED"
        elif box.angle_status == "unknown":
            color = (0, 165, 255)
            state = "UNKNOWN"
        else:
            color = (0, 220, 0)
            state = "OK"
        cv2.rectangle(drawn, (x1, y1), (x2, y2), color, 3)
        angle_text = "angle=?" if box.angle_deviation_deg is None else f"dev={box.angle_deviation_deg:.1f}"
        cv2.putText(drawn, f"box#{index} {state} {angle_text}", (x1, min(y2 + 18, drawn.shape[0] - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)
    return drawn


def _analyze_single_box(frame: np.ndarray, detection: Any, config: BoxOrderConfig) -> BoxItemResult:
    x1 = float(getattr(detection, "x1"))
    y1 = float(getattr(detection, "y1"))
    x2 = float(getattr(detection, "x2"))
    y2 = float(getattr(detection, "y2"))
    confidence = float(getattr(detection, "score", getattr(detection, "confidence", 0.0)))
    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0
    angle_deg, reason = _estimate_box_angle_from_roi(frame, x1, y1, x2, y2, config)
    if angle_deg is None:
        return BoxItemResult(x1, y1, x2, y2, cx, cy, confidence, None, None, False, "unknown", reason)
    normalized = angle_deg % 90.0
    deviation = min(normalized, 90.0 - normalized)
    is_tilted = deviation > config.angle_thresh_deg
    return BoxItemResult(x1, y1, x2, y2, cx, cy, confidence, angle_deg, deviation, is_tilted, "ok", reason)


def _estimate_box_angle_from_roi(frame: np.ndarray, x1: float, y1: float, x2: float, y2: float, config: BoxOrderConfig) -> tuple[Optional[float], str]:
    height, width = frame.shape[:2]
    left = max(int(np.floor(x1)), 0)
    top = max(int(np.floor(y1)), 0)
    right = min(int(np.ceil(x2)), width)
    bottom = min(int(np.ceil(y2)), height)
    if right <= left + 3 or bottom <= top + 3:
        return None, "ROI无效"
    roi = frame[top:bottom, left:right]
    if roi.size == 0:
        return None, "ROI为空"
    try:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        kernel = max(1, int(config.blur_kernel))
        if kernel % 2 == 0:
            kernel += 1
        if kernel > 1:
            gray = cv2.GaussianBlur(gray, (kernel, kernel), 0)
        edges = cv2.Canny(gray, int(config.canny_low), int(config.canny_high))
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    except cv2.error:
        return None, "轮廓提取失败"
    if not contours:
        return None, "未找到轮廓"
    roi_area = float(roi.shape[0] * roi.shape[1])
    min_area = roi_area * float(config.min_contour_area_ratio)
    valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= min_area]
    if valid_contours:
        angle = _long_edge_angle_from_points(max(valid_contours, key=cv2.contourArea))
        if angle is None:
            return None, "旋转矩形尺寸过小"
        return angle, ""

    angle = _long_edge_angle_from_fragmented_contours(contours, roi.shape[:2], config)
    if angle is None:
        return None, "轮廓面积过小"
    return angle, ""


def _long_edge_angle_from_fragmented_contours(contours: list[np.ndarray], roi_shape: tuple[int, int], config: BoxOrderConfig) -> Optional[float]:
    if not contours:
        return None
    points = np.concatenate([contour.reshape(-1, 2) for contour in contours if len(contour) > 0], axis=0)
    if len(points) < int(config.min_edge_points):
        return None

    roi_h, roi_w = roi_shape
    span_w = float(np.max(points[:, 0]) - np.min(points[:, 0]))
    span_h = float(np.max(points[:, 1]) - np.min(points[:, 1]))
    min_span_ratio = float(config.min_edge_span_ratio)
    if span_w < roi_w * min_span_ratio or span_h < roi_h * min_span_ratio:
        return None
    return _long_edge_angle_from_points(points.astype(np.float32))


def _long_edge_angle_from_points(points: np.ndarray) -> Optional[float]:
    try:
        rect = cv2.minAreaRect(points)
    except cv2.error:
        return None
    rect_w, rect_h = rect[1]
    if rect_w <= 1 or rect_h <= 1:
        return None
    angle = float(rect[2])
    long_edge_angle = angle if rect_w >= rect_h else angle + 90.0
    return long_edge_angle % 180.0


def _cluster_axis(values: np.ndarray, threshold: float) -> tuple[list[float], list[int]]:
    order = np.argsort(values)
    centers: list[float] = []
    clusters: list[list[int]] = []
    for idx in order:
        value = float(values[idx])
        if not centers or abs(value - centers[-1]) > threshold:
            centers.append(value)
            clusters.append([int(idx)])
        else:
            clusters[-1].append(int(idx))
            centers[-1] = float(np.mean(values[clusters[-1]]))
    ids = [0] * len(values)
    for cluster_id, indices in enumerate(clusters):
        for idx in indices:
            ids[idx] = cluster_id
    return centers, ids


def _has_duplicate_cells(row_ids: list[int], col_ids: list[int]) -> bool:
    cells = list(zip(row_ids, col_ids))
    return len(set(cells)) != len(cells)


def _spacing_is_regular(centers: list[float], tolerance_ratio: float) -> bool:
    if len(centers) <= 2:
        return True
    sorted_centers = np.array(sorted(centers), dtype=np.float32)
    spacings = np.diff(sorted_centers)
    median = float(np.median(spacings))
    if median <= 1e-6:
        return False
    return bool(np.all(np.abs(spacings - median) <= median * tolerance_ratio))
