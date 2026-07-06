from __future__ import annotations

import json
import threading
import time
from datetime import datetime
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Callable, Optional

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from scipy.special import softmax

try:
    from robot_interfaces.action import ExecuteMission
except ImportError:  # pragma: no cover - robot_interfaces is present on the RDK workspaces.
    ExecuteMission = None

from yolo_web.detect_box import (
    BoxOrderConfig,
    BoxOrderResult,
    analyze_box_order,
    draw_box_order_result,
    load_box_order_config,
)
from yolo_web.vision_activation_gate import (
    VisionActivationGate,
    VisionTarget,
    default_amcl_pose_topic,
    default_mission_feedback_topic,
    infer_robot_id_from_path,
)


BOUNDARY = b"frame"


def package_asset_path(*parts: str) -> str:
    """Return an installed yolo_web asset path, falling back to the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory

        package_root = Path(get_package_share_directory("yolo_web"))
    except Exception:
        package_root = Path(__file__).resolve().parents[1]
    return str(package_root.joinpath(*parts))


DEFAULT_MODEL_PATH = package_asset_path("models", "box_camera_best_bayese_640x640_nv12.bin")
DEFAULT_LABEL_FILE = package_asset_path("models", "classes.txt")
DEFAULT_FIRE_SMOKE_MODEL_PATH = package_asset_path("models", "fire_smoke_best_bayese_640x640_nv12.bin")
DEFAULT_FIRE_SMOKE_LABEL_FILE = package_asset_path("models", "fire_smoke.list")
DEFAULT_BOX_ORDER_CONFIG = package_asset_path("config", "box_order.yaml")
DEFAULT_ROBOT_ID = infer_robot_id_from_path(package_asset_path())


@dataclass(frozen=True)
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    class_id: int
    label: str = ""
    model_name: str = ""


@dataclass(frozen=True)
class PointContext:
    point_id: str
    point_type: str
    label: str
    score: float

    @property
    def key(self) -> str:
        return f"{self.point_type}:{self.point_id}"


POINT_IDLE = "idle"
POINT_WAITING_ARRIVAL_STOP = "waiting_arrival_stop"
POINT_ARRIVAL_REPORTED_WAITING_MOVE = "arrival_reported_waiting_move"
POINT_MOVING_TO_TARGET = "moving_to_target"
POINT_WAITING_INSPECTION_STOP = "waiting_inspection_stop"
POINT_INSPECTION_ENABLED = "inspection_enabled"

PICKUP_LETTERS = {"A", "B"}
DELIVERY_LETTERS = {"C", "D"}
PATROL_NUMBERS = {"1", "2", "3", "4"}

def parameter_as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def build_mjpeg_frame(jpeg_bytes: bytes) -> bytes:
    return (
        b"--" + BOUNDARY + b"\r\n"
        b"Content-Type: image/jpeg\r\n"
        b"Content-Length: " + str(len(jpeg_bytes)).encode("ascii") + b"\r\n"
        b"\r\n" + jpeg_bytes + b"\r\n"
    )


def load_labels(label_file: str, classes_num: int) -> list[str]:
    labels: list[str] = []
    if label_file:
        try:
            with open(label_file, "r", encoding="utf-8") as handle:
                labels = [line.strip() for line in handle if line.strip()]
        except OSError:
            labels = []
    if len(labels) < classes_num:
        labels.extend(f"class_{idx}" for idx in range(len(labels), classes_num))
    return labels


def letterbox_image(image: np.ndarray, input_w: int, input_h: int) -> np.ndarray:
    img_h, img_w = image.shape[:2]
    scale = min(input_h / img_h, input_w / img_w)
    new_w, new_h = int(img_w * scale), int(img_h * scale)
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
    pad_w = input_w - new_w
    pad_h = input_h - new_h
    left, right = pad_w // 2, pad_w - pad_w // 2
    top, bottom = pad_h // 2, pad_h - pad_h // 2
    return cv2.copyMakeBorder(
        resized,
        top,
        bottom,
        left,
        right,
        borderType=cv2.BORDER_CONSTANT,
        value=(127, 127, 127),
    )


def bgr_to_nv12_tensor(image: np.ndarray) -> np.ndarray:
    height, width = image.shape[:2]
    area = height * width
    yuv420p = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
    y = yuv420p[:area].reshape((height, width))
    u = yuv420p[area: area + area // 4].reshape((height // 2, width // 2))
    v = yuv420p[area + area // 4:].reshape((height // 2, width // 2))
    uv = np.stack((u, v), axis=-1)
    nv12 = np.concatenate((y.reshape(-1), uv.reshape(-1)), axis=0)
    return nv12.reshape((1, height * 3 // 2, width, 1))


def scale_coords_back(boxes: np.ndarray, img_w: int, img_h: int, input_w: int, input_h: int) -> np.ndarray:
    if boxes.size == 0:
        return boxes
    scale = min(input_w / img_w, input_h / img_h)
    pad_w = (input_w - img_w * scale) / 2
    pad_h = (input_h - img_h * scale) / 2
    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad_w) / scale
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad_h) / scale
    boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, img_w)
    boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, img_h)
    return boxes


def nms_classwise(boxes: np.ndarray, scores: np.ndarray, class_ids: np.ndarray, iou_threshold: float) -> list[int]:
    keep: list[int] = []
    if boxes.size == 0:
        return keep
    for class_id in np.unique(class_ids):
        indices = np.where(class_ids == class_id)[0]
        x1, y1, x2, y2 = boxes[indices].T
        areas = np.maximum(x2 - x1, 0) * np.maximum(y2 - y1, 0)
        order = scores[indices].argsort()[::-1]
        while order.size > 0:
            current = order[0]
            keep.append(indices[current])
            xx1 = np.maximum(x1[current], x1[order[1:]])
            yy1 = np.maximum(y1[current], y1[order[1:]])
            xx2 = np.minimum(x2[current], x2[order[1:]])
            yy2 = np.minimum(y2[current], y2[order[1:]])
            inter = np.maximum(xx2 - xx1, 0) * np.maximum(yy2 - yy1, 0)
            union = areas[current] + areas[order[1:]] - inter + 1e-9
            order = order[1:][inter / union < iou_threshold]
    return keep


def gen_anchor(grid_size: int) -> np.ndarray:
    x = np.tile(np.linspace(0.5, grid_size - 0.5, grid_size), reps=grid_size)
    y = np.repeat(np.linspace(0.5, grid_size - 0.5, grid_size), grid_size)
    return np.stack([x, y], axis=1)


def draw_detections(image: np.ndarray, detections: list[Detection]) -> np.ndarray:
    drawn = image.copy()
    for detection in detections:
        x1 = int(round(detection.x1))
        y1 = int(round(detection.y1))
        x2 = int(round(detection.x2))
        y2 = int(round(detection.y2))
        color = (
            int((37 * detection.class_id + 80) % 255),
            int((17 * detection.class_id + 180) % 255),
            int((97 * detection.class_id + 40) % 255),
        )
        prefix = f"{detection.model_name}:" if detection.model_name else ""
        label = detection.label or f"class_{detection.class_id}"
        text = f"{prefix}{label} {detection.score:.2f}"
        cv2.rectangle(drawn, (x1, y1), (x2, y2), color, 2)
        text_size, baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        text_w, text_h = text_size
        box_y1 = max(y1 - text_h - baseline - 4, 0)
        cv2.rectangle(drawn, (x1, box_y1), (x1 + text_w + 6, box_y1 + text_h + baseline + 4), color, -1)
        cv2.putText(drawn, text, (x1 + 3, box_y1 + text_h + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return drawn


def build_index_html() -> bytes:
    html = """<!doctype html>
<html>
<head><meta charset="utf-8"><title>YOLO Web</title>
</head>
<body style="margin:0;background:#111;height:100vh;overflow:hidden;">
<img src="/stream.mjpg" style="width:100vw;height:100vh;object-fit:contain;display:block;" />
</body>
</html>
"""
    return html.encode("utf-8")


class YoloV8Detector:
    def __init__(self, model_path: str, label_file: str, score_threshold: float, nms_threshold: float, display_name: str):
        import hbm_runtime

        self.model = hbm_runtime.HB_HBMRuntime(model_path)
        self.display_name = display_name
        self.model_name = self.model.model_names[0]
        self.input_names = self.model.input_names[self.model_name]
        self.output_names = self.model.output_names[self.model_name]
        self.input_shapes = self.model.input_shapes[self.model_name]
        self.input_h = self.input_shapes[self.input_names[0]][2]
        self.input_w = self.input_shapes[self.input_names[0]][3]
        self.score_threshold = score_threshold
        self.conf_threshold_raw = -np.log(1 / score_threshold - 1)
        self.nms_threshold = nms_threshold
        self.reg = 16
        self.strides = [8, 16, 32]
        self.anchor_sizes = [80, 40, 20]
        self.weights_static = np.arange(self.reg, dtype=np.float32)[np.newaxis, np.newaxis, :]
        self.classes_num = int(self.model.output_shapes[self.model_name][self.output_names[0]][-1])
        self.labels = load_labels(label_file, self.classes_num)
        self.lock = threading.Lock()

    def pre_process(self, frame: np.ndarray) -> dict:
        resized = letterbox_image(frame, self.input_w, self.input_h)
        nv12 = bgr_to_nv12_tensor(resized)
        return {self.model_name: {self.input_names[0]: nv12}}

    def detect(self, frame: np.ndarray) -> list[Detection]:
        img_h, img_w = frame.shape[:2]
        with self.lock:
            outputs = self.model.run(self.pre_process(frame))[self.model_name]
        boxes, class_ids, scores = self.post_process(outputs, img_w, img_h)
        detections: list[Detection] = []
        for box, class_id, score in zip(boxes, class_ids, scores):
            class_idx = int(class_id)
            detections.append(Detection(
                float(box[0]), float(box[1]), float(box[2]), float(box[3]), float(score), class_idx,
                self.labels[class_idx] if class_idx < len(self.labels) else f"class_{class_idx}",
                self.display_name,
            ))
        return detections

    def post_process(self, outputs: dict, img_w: int, img_h: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        decoded_boxes = []
        decoded_scores = []
        decoded_ids = []
        for i, (stride, anchor_size) in enumerate(zip(self.strides, self.anchor_sizes)):
            cls_output = outputs[self.output_names[2 * i]].reshape(-1, outputs[self.output_names[2 * i]].shape[-1])
            box_output = outputs[self.output_names[2 * i + 1]].reshape(-1, outputs[self.output_names[2 * i + 1]].shape[-1])
            max_scores = np.max(cls_output, axis=1)
            valid_indices = np.flatnonzero(max_scores >= self.conf_threshold_raw)
            if valid_indices.size == 0:
                decoded_boxes.append(np.empty((0, 4), dtype=np.float32))
                decoded_scores.append(np.empty((0,), dtype=np.float32))
                decoded_ids.append(np.empty((0,), dtype=np.int32))
                continue
            class_ids = np.argmax(cls_output[valid_indices], axis=1).astype(np.int32)
            scores = (1 / (1 + np.exp(-max_scores[valid_indices]))).astype(np.float32)
            selected_boxes = box_output[valid_indices]
            ltrb = np.sum(softmax(selected_boxes.reshape(-1, 4, self.reg), axis=2) * self.weights_static, axis=2)
            anchor = gen_anchor(anchor_size)[valid_indices]
            x1y1 = anchor - ltrb[:, 0:2]
            x2y2 = anchor + ltrb[:, 2:4]
            decoded_boxes.append((np.hstack([x1y1, x2y2]) * stride).astype(np.float32))
            decoded_scores.append(scores)
            decoded_ids.append(class_ids)
        boxes = np.concatenate(decoded_boxes, axis=0)
        scores = np.concatenate(decoded_scores, axis=0)
        class_ids = np.concatenate(decoded_ids, axis=0)
        keep = nms_classwise(boxes, scores, class_ids, self.nms_threshold)
        if not keep:
            return np.empty((0, 4), dtype=np.float32), np.empty((0,), dtype=np.int32), np.empty((0,), dtype=np.float32)
        boxes = scale_coords_back(boxes[keep], img_w, img_h, self.input_w, self.input_h)
        return boxes, class_ids[keep], scores[keep]


class SharedUsbCamera:
    def __init__(
        self,
        device: str,
        width: int,
        height: int,
        fps: int,
        jpeg_quality: int,
        detectors: Optional[list[YoloV8Detector]] = None,
        enable_box_order_check: bool = False,
        box_order_config: Optional[BoxOrderConfig] = None,
        box_order_log_callback: Optional[Callable[[str], None]] = None,
        box_order_warn_callback: Optional[Callable[[str], None]] = None,
        inspection_event_callback: Optional[Callable[[str], None]] = None,
        robot_id: str = "robot_a",
        raw_event_interval_sec: float = 1.0,
        stop_hold_sec: float = 1.0,
        stop_epsilon: float = 1e-3,
        vision_activation_gate: Optional[VisionActivationGate] = None,
    ):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self.detectors = detectors or []
        self.enable_box_order_check = enable_box_order_check
        self.box_order_config = box_order_config or BoxOrderConfig()
        self.box_order_log_callback = box_order_log_callback
        self.box_order_warn_callback = box_order_warn_callback
        self.inspection_event_callback = inspection_event_callback
        self.robot_id = robot_id
        self.raw_event_interval_sec = float(raw_event_interval_sec)
        self.stop_hold_sec = max(float(stop_hold_sec), 0.0)
        self.stop_epsilon = max(float(stop_epsilon), 0.0)
        self.vision_activation_gate = vision_activation_gate
        self._last_raw_event_time = float("-inf")
        self._motion_is_stopped = False
        self._motion_stopped_since: Optional[float] = None
        self._point_state = POINT_IDLE
        self._current_point: Optional[PointContext] = None
        self._reported_point_keys: set[str] = set()
        self._completed_point_keys: set[str] = set()
        self._last_patrol_point: Optional[str] = None
        self._reported_arrival_points: set[str] = set()
        self._last_box_order_info_time = 0.0
        self._last_box_order_warn_time = 0.0
        self._last_box_order_state = None
        self._last_hazard_labels: set[str] = set()
        self._video_status_text = ""
        self._active_task_target_key: Optional[str] = None
        self._lock = threading.Lock()
        self._cap: Optional[cv2.VideoCapture] = None

    def open(self) -> None:
        with self._lock:
            if self._cap is not None and self._cap.isOpened():
                return
            cap = cv2.VideoCapture(self.device)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            if not cap.isOpened():
                cap.release()
                raise RuntimeError(f"failed to open camera device {self.device}")
            self._cap = cap

    def read_jpeg(self) -> Optional[bytes]:
        with self._lock:
            if self._cap is None or not self._cap.isOpened():
                return None
            ok, frame = self._cap.read()
        if not ok or frame is None:
            return None

        if not self._vision_detection_is_active():
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ok:
                return None
            return encoded.tobytes()

        source_frame = frame.copy()
        detections: list[Detection] = []
        for detector in self.detectors:
            detections.extend(detector.detect(frame))
        if self.detectors:
            frame = draw_detections(frame, detections)

        box_order_result: Optional[BoxOrderResult] = None
        if self.enable_box_order_check:
            try:
                box_order_result = analyze_box_order(source_frame, detections, self.box_order_config)
                self._log_box_order_result(box_order_result)
            except Exception as exc:  # Keep the web stream alive even if one frame fails.
                self._log_box_order_message(f"箱子摆放异常判断失败：{exc}")

        self._log_hazard_detections(detections)
        self._publish_raw_detection_event(detections, box_order_result)
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return None
        return encoded.tobytes()

    def _vision_detection_is_active(self) -> bool:
        if self.vision_activation_gate is None:
            return True

        snapshot = self.vision_activation_gate.snapshot()
        if not snapshot.active or snapshot.target is None:
            self._clear_task_context_for_inactive_gate()
            return False

        context = self._point_context_from_activation_target(snapshot.target)
        if context is None:
            self._clear_task_context_for_inactive_gate()
            return False

        if snapshot.target.key != self._active_task_target_key:
            self._active_task_target_key = snapshot.target.key
            self._start_point_context(context)
        return True

    def _clear_task_context_for_inactive_gate(self) -> None:
        self._current_point = None
        self._point_state = POINT_IDLE
        self._last_hazard_labels = set()
        self._last_box_order_state = None
        self._video_status_text = ""
        self._active_task_target_key = None

    def _log_hazard_detections(self, detections: list[Detection]) -> None:
        current_labels = {det.label for det in detections if det.model_name == "fire_smoke" and det.label in {"fire", "smoke"}}
        if not current_labels:
            self._last_hazard_labels = set()
            return
        new_labels = current_labels - self._last_hazard_labels
        for label, message in (("fire", "检测到疑似火焰"), ("smoke", "检测到疑似烟雾")):
            if label in new_labels:
                self._log_box_order_message(message, warn=True, force=True)
        self._last_hazard_labels = current_labels

    def update_motion_state(self, twist, now: Optional[float] = None) -> None:
        if now is None:
            now = time.monotonic()
        values = (
            float(getattr(twist.linear, "x", 0.0)),
            float(getattr(twist.linear, "y", 0.0)),
            float(getattr(twist.linear, "z", 0.0)),
            float(getattr(twist.angular, "x", 0.0)),
            float(getattr(twist.angular, "y", 0.0)),
            float(getattr(twist.angular, "z", 0.0)),
        )
        stopped = all(abs(value) <= self.stop_epsilon for value in values)
        self._set_motion_stopped(stopped, now)

    def _set_motion_stopped(self, stopped: bool, now: float) -> None:
        was_stopped = self._motion_is_stopped
        self._motion_is_stopped = stopped
        if stopped:
            if not was_stopped:
                self._motion_stopped_since = now
            if self._point_state == POINT_MOVING_TO_TARGET:
                self._point_state = POINT_WAITING_INSPECTION_STOP
                self._video_status_text = self._waiting_status_text()
            return

        self._motion_stopped_since = None
        if self._point_state == POINT_ARRIVAL_REPORTED_WAITING_MOVE:
            self._point_state = POINT_MOVING_TO_TARGET
            self._video_status_text = self._waiting_status_text()
        elif self._point_state == POINT_INSPECTION_ENABLED:
            self._finish_current_point()

    def _has_stopped_for_required_time(self, now: float) -> bool:
        if not self._motion_is_stopped or self._motion_stopped_since is None:
            return False
        return now - self._motion_stopped_since >= self.stop_hold_sec

    def _publish_raw_detection_event(self, detections: list[Detection], box_order_result: Optional[BoxOrderResult]) -> bool:
        now = time.monotonic()
        current_point = self._extract_point_context(detections)
        if self._current_point is None and current_point is not None and current_point.key not in self._completed_point_keys:
            self._start_point_context(current_point)

        event = self._build_gated_detection_event(detections, box_order_result, now)
        if event is None:
            return False

        interval = max(self.raw_event_interval_sec, 0.0)
        if interval > 0.0 and now - self._last_raw_event_time < interval:
            return False

        self._last_raw_event_time = now
        self._publish_event_dict(event)
        self._after_event_published(event)
        return True

    def _build_gated_detection_event(
        self,
        detections: list[Detection],
        box_order_result: Optional[BoxOrderResult],
        now: float,
    ) -> Optional[dict]:
        point = self._current_point
        if point is None:
            return None

        if self._point_state == POINT_WAITING_ARRIVAL_STOP:
            if not self._has_stopped_for_required_time(now):
                return None
            if point.key in self._reported_point_keys:
                self._point_state = POINT_ARRIVAL_REPORTED_WAITING_MOVE
                return None
            return self._build_arrival_event(point)

        if self._point_state in {POINT_ARRIVAL_REPORTED_WAITING_MOVE, POINT_MOVING_TO_TARGET}:
            return None

        if self._point_state == POINT_WAITING_INSPECTION_STOP:
            if not self._has_stopped_for_required_time(now):
                return None
            self._point_state = POINT_INSPECTION_ENABLED
            self._video_status_text = self._active_status_text()

        if self._point_state == POINT_INSPECTION_ENABLED:
            if point.point_type == "patrol":
                if self._has_only_ignored_app_detections(detections) and box_order_result is None:
                    return None
                return self._build_raw_detection_event(
                    detections,
                    box_order_result,
                    point.point_id,
                    None,
                    False,
                )
            return self._build_shelf_task_event(point, detections)

        return None

    def _build_arrival_event(self, point: PointContext) -> dict:
        base = self._event_base_for_point(point)
        if point.point_type == "patrol":
            return {**base, "level": "info", "status": "arrived", "event_type": "arrival", "event_code": f"PATROL_POINT_{point.point_id}", "message": ""}
        if point.point_type == "pickup":
            return {**base, "level": "info", "status": "arrived", "event_type": "arrival", "event_code": f"PICKUP_POINT_{point.point_id}", "message": ""}
        return {**base, "level": "info", "status": "arrived", "event_type": "arrival", "event_code": f"DELIVERY_POINT_{point.point_id}", "message": ""}

    def _build_shelf_task_event(self, point: PointContext, detections: list[Detection]) -> Optional[dict]:
        has_shelf = any(det.model_name == "box_camera" and det.label == "shelf" for det in detections)
        if not has_shelf:
            return None
        base = self._event_base_for_point(point)
        if point.point_type == "pickup":
            event = {**base, "level": "info", "status": "ready", "event_type": "shelf", "event_code": "SHELF_PICKUP_READY", "message": "已检测到货架，开始检货"}
        else:
            event = {**base, "level": "info", "status": "ready", "event_type": "shelf", "event_code": "SHELF_DELIVERY_READY", "message": "已检测到货架，开始发货"}
        self._video_status_text = self._event_status_text(event)
        return event

    def _event_base_for_point(self, point: PointContext) -> dict:
        if point.point_type == "patrol":
            location = f"巡检点 {point.point_id}"
        elif point.point_type == "pickup":
            location = f"检货点 {point.point_id}"
        else:
            location = f"发货点 {point.point_id}"
        return {
            "robot_id": self.robot_id,
            "patrol_point": point.point_id,
            "location": location,
            "task_point_type": point.point_type,
            "timestamp": datetime.now().isoformat(),
        }

    def _after_event_published(self, event: dict) -> None:
        point = self._current_point
        if point is None:
            return
        if event.get("event_type") == "arrival":
            self._video_status_text = self._arrival_status_text(point)
            self._reported_point_keys.add(point.key)
            self._reported_arrival_points.add(point.point_id)
            self._point_state = POINT_ARRIVAL_REPORTED_WAITING_MOVE
            return
        if self._point_state == POINT_INSPECTION_ENABLED and point.point_type != "patrol":
            self._finish_current_point(clear_status=False)

    def _has_only_ignored_app_detections(self, detections: list[Detection]) -> bool:
        return bool(detections) and all(det.model_name == "box_camera" and det.label == "Fire-hydrant" for det in detections)

    def _build_raw_detection_event(
        self,
        detections: list[Detection],
        box_order_result: Optional[BoxOrderResult],
        patrol_point: str,
        current_patrol_point: Optional[str],
        is_new_arrival: bool,
    ) -> Optional[dict]:
        location = f"巡检点 {patrol_point}" if patrol_point != "UNKNOWN" else "未知巡检点"
        base = {
            "robot_id": self.robot_id,
            "patrol_point": patrol_point,
            "location": location,
            "timestamp": datetime.now().isoformat(),
        }
        if is_new_arrival and current_patrol_point is not None:
            return {**base, "level": "info", "status": "arrived", "event_type": "arrival", "event_code": f"PATROL_POINT_{current_patrol_point}", "message": ""}
        hazard_labels = {det.label for det in detections if det.model_name == "fire_smoke" and det.label in {"fire", "smoke"}}
        if "fire" in hazard_labels:
            event = {**base, "level": "warn", "status": "abnormal", "event_type": "fire", "event_code": "FIRE_DETECTED", "message": "检测到疑似火焰"}
            self._video_status_text = self._event_status_text(event)
            return event
        if "smoke" in hazard_labels:
            event = {**base, "level": "warn", "status": "abnormal", "event_type": "smoke", "event_code": "SMOKE_DETECTED", "message": "检测到疑似烟雾"}
            self._video_status_text = self._event_status_text(event)
            return event
        if box_order_result is not None and box_order_result.box_count > 0 and not box_order_result.is_tidy:
            event = {**base, "level": "warn", "status": "abnormal", "event_type": "box_order", "event_code": "BOX_ORDER_ABNORMAL", "message": "箱子摆放异常"}
            self._video_status_text = self._event_status_text(event)
            return event
        event = {**base, "level": "info", "status": "normal", "event_type": "inspection", "event_code": "NORMAL", "message": "当前巡检点未发现异常"}
        self._video_status_text = self._event_status_text(event)
        return event

    def _extract_point_context(self, detections: list[Detection]) -> Optional[PointContext]:
        best_point: Optional[PointContext] = None
        best_score = -1.0
        for det in detections:
            if det.model_name and det.model_name != "box_camera":
                continue
            label = str(det.label)
            context = self._point_context_from_label(label, det.score)
            if context is not None and det.score > best_score:
                best_point = context
                best_score = det.score
        return best_point

    def _point_context_from_label(self, label: str, score: float) -> Optional[PointContext]:
        if label.startswith("letter_"):
            point_id = label.split("letter_", 1)[1]
            if point_id in PICKUP_LETTERS:
                return PointContext(point_id, "pickup", label, score)
            if point_id in DELIVERY_LETTERS:
                return PointContext(point_id, "delivery", label, score)
            return None
        if label.startswith("number_"):
            point_id = label.split("number_", 1)[1]
            if point_id in PATROL_NUMBERS:
                return PointContext(f"P{point_id}", "patrol", label, score)
        return None

    def _point_context_from_activation_target(self, target: VisionTarget) -> Optional[PointContext]:
        point_id = target.point_id.strip()
        upper = point_id.upper()
        if target.point_type == "pickup":
            display_id = upper.split("PICKUP_", 1)[1] if upper.startswith("PICKUP_") else point_id
            return PointContext(display_id, "pickup", "task_target", 1.0)
        if target.point_type == "delivery":
            display_id = upper.split("DELIVERY_", 1)[1] if upper.startswith("DELIVERY_") else point_id
            return PointContext(display_id, "delivery", "task_target", 1.0)
        if target.point_type == "inspection":
            return PointContext(point_id, "patrol", "task_target", 1.0)
        return None

    def _start_point_context(self, point: PointContext) -> None:
        self._current_point = point
        self._last_patrol_point = point.point_id
        self._point_state = POINT_WAITING_ARRIVAL_STOP
        self._last_box_order_state = None
        self._last_hazard_labels = set()
        self._video_status_text = self._arrival_status_text(point)

    def _finish_current_point(self, clear_status: bool = True) -> None:
        if self._current_point is not None:
            self._completed_point_keys.add(self._current_point.key)
        self._current_point = None
        self._point_state = POINT_IDLE
        if clear_status:
            self._video_status_text = ""

    def _extract_patrol_point(self, detections: list[Detection]) -> Optional[str]:
        point = self._extract_point_context(detections)
        if point is None or point.point_type != "patrol":
            return None
        return point.point_id

    def _point_title(self, point: Optional[PointContext] = None) -> str:
        point = point or self._current_point
        if point is None:
            return ""
        if point.point_type == "patrol":
            return f"巡检点 {point.point_id}"
        if point.point_type == "pickup":
            return f"检货点 {point.point_id}"
        return f"发货点 {point.point_id}"

    def _arrival_status_text(self, point: Optional[PointContext] = None) -> str:
        return ""

    def _waiting_status_text(self) -> str:
        return ""

    def _active_status_text(self) -> str:
        return ""

    def _event_status_text(self, event: dict) -> str:
        return ""

    def _log_box_order_result(self, result: BoxOrderResult) -> None:
        if result.box_count <= 0:
            self._last_box_order_state = None
            return
        state = (result.box_count, result.is_tidy)
        if state == self._last_box_order_state:
            return
        status = "正常" if result.is_tidy else "异常"
        message = f"检测到{result.box_count}个箱子 箱子摆放状态：{status}"
        self._log_box_order_message(message, warn=not result.is_tidy, force=True)
        self._last_box_order_state = state
        if result.is_tidy:
            pass
        else:
            pass

    def _publish_inspection_event(self, level: str, status: str, event_type: str, event_code: str, message: str) -> bool:
        event = {
            "robot_id": self.robot_id,
            "level": level,
            "status": status,
            "event_type": event_type,
            "event_code": event_code,
            "message": message,
            "timestamp": datetime.now().isoformat(),
        }
        return self._publish_event_dict(event)

    def _publish_event_dict(self, event: dict) -> bool:
        if self.inspection_event_callback is None:
            return False
        self.inspection_event_callback(json.dumps(event, ensure_ascii=False))
        return True

    def _log_box_order_message(self, message: str, warn: bool = False, force: bool = False) -> bool:
        now = time.monotonic()
        interval = max(float(self.box_order_config.log_interval_sec), 0.0)
        last_time_attr = "_last_box_order_warn_time" if warn else "_last_box_order_info_time"
        last_time = getattr(self, last_time_attr, 0.0)
        if not force and interval > 0.0 and now - last_time < interval:
            return False
        if warn:
            if self.box_order_warn_callback is None:
                return False
            self.box_order_warn_callback(message)
        else:
            if self.box_order_log_callback is None:
                return False
            self.box_order_log_callback(message)
        setattr(self, last_time_attr, now)
        return True

    def close(self) -> None:
        with self._lock:
            if self._cap is not None:
                self._cap.release()
                self._cap = None


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class UsbCameraRequestHandler(BaseHTTPRequestHandler):
    camera: SharedUsbCamera = None
    frame_delay: float = 0.1

    def log_message(self, fmt, *args):
        return

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send_index()
        elif self.path == "/stream.mjpg":
            self._send_stream()
        elif self.path == "/snapshot.jpg":
            self._send_snapshot()
        elif self.path == "/health":
            self._send_text("ok\n")
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "not found")

    def do_POST(self):
        if self.path != "/arrival_confirm":
            self.send_error(HTTPStatus.NOT_FOUND, "not found")
            return
        content_length = int(self.headers.get("Content-Length", "0") or "0")
        body = self.rfile.read(content_length).decode("utf-8", errors="ignore") if content_length > 0 else "{}"
        try:
            payload = json.loads(body)
        except json.JSONDecodeError:
            payload = {"raw": body}
        print(f"[yolo_web] arrival_confirm received: {payload}")
        self._send_text("ok\n")

    def _send_index(self):
        body = build_index_html()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_text(self, text: str):
        body = text.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_snapshot(self):
        jpeg = self.camera.read_jpeg()
        if jpeg is None:
            self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "camera frame unavailable")
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpeg)))
        self.end_headers()
        self.wfile.write(jpeg)

    def _send_stream(self):
        self.send_response(HTTPStatus.OK)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        while True:
            jpeg = self.camera.read_jpeg()
            if jpeg is None:
                time.sleep(self.frame_delay)
                continue
            try:
                self.wfile.write(build_mjpeg_frame(jpeg))
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                break
            time.sleep(self.frame_delay)


class UsbCameraWebNode(Node):
    def __init__(self):
        super().__init__("yolo_web")
        default_robot_id = DEFAULT_ROBOT_ID
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8088)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("enable_detection", True)
        self.declare_parameter("model_path", DEFAULT_MODEL_PATH)
        self.declare_parameter("label_file", DEFAULT_LABEL_FILE)
        self.declare_parameter("fire_smoke_model_path", DEFAULT_FIRE_SMOKE_MODEL_PATH)
        self.declare_parameter("fire_smoke_label_file", DEFAULT_FIRE_SMOKE_LABEL_FILE)
        self.declare_parameter("score_threshold", 0.35)
        self.declare_parameter("fire_smoke_score_threshold", 0.35)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("enable_box_order_check", True)
        self.declare_parameter("box_order_config", DEFAULT_BOX_ORDER_CONFIG)
        self.declare_parameter("robot_id", default_robot_id)
        self.declare_parameter("raw_event_interval_sec", 1.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("stop_hold_sec", 1.0)
        self.declare_parameter("stop_epsilon", 1e-3)
        self.declare_parameter("enable_activation_gate", True)
        self.declare_parameter("activation_enter_radius", 0.5)
        self.declare_parameter("activation_exit_radius", 0.7)
        self.declare_parameter("mission_feedback_topic", default_mission_feedback_topic(default_robot_id))
        self.declare_parameter("amcl_pose_topic", default_amcl_pose_topic(default_robot_id))

        device = self.get_parameter("device").value
        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        fps = int(self.get_parameter("fps").value)
        jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        enable_detection = parameter_as_bool(self.get_parameter("enable_detection").value)
        model_path = self.get_parameter("model_path").value
        label_file = self.get_parameter("label_file").value
        fire_smoke_model_path = self.get_parameter("fire_smoke_model_path").value
        fire_smoke_label_file = self.get_parameter("fire_smoke_label_file").value
        score_threshold = float(self.get_parameter("score_threshold").value)
        fire_smoke_score_threshold = float(self.get_parameter("fire_smoke_score_threshold").value)
        nms_threshold = float(self.get_parameter("nms_threshold").value)
        enable_box_order_check = parameter_as_bool(self.get_parameter("enable_box_order_check").value)
        box_order_config_path = self.get_parameter("box_order_config").value
        robot_id = str(self.get_parameter("robot_id").value).strip() or default_robot_id
        raw_event_interval_sec = float(self.get_parameter("raw_event_interval_sec").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        stop_hold_sec = float(self.get_parameter("stop_hold_sec").value)
        stop_epsilon = float(self.get_parameter("stop_epsilon").value)
        enable_activation_gate = parameter_as_bool(self.get_parameter("enable_activation_gate").value)
        activation_enter_radius = float(self.get_parameter("activation_enter_radius").value)
        activation_exit_radius = float(self.get_parameter("activation_exit_radius").value)
        mission_feedback_topic = (
            str(self.get_parameter("mission_feedback_topic").value).strip()
            or default_mission_feedback_topic(robot_id)
        )
        amcl_pose_topic = (
            str(self.get_parameter("amcl_pose_topic").value).strip()
            or default_amcl_pose_topic(robot_id)
        )

        self.activation_gate: Optional[VisionActivationGate] = None
        if enable_activation_gate:
            self.activation_gate = VisionActivationGate(activation_enter_radius, activation_exit_radius)
            if ExecuteMission is None:
                self.get_logger().warn("robot_interfaces action 不可用，视觉激活门无法订阅任务反馈，YOLO 将保持关闭")
            else:
                self.mission_feedback_sub = self.create_subscription(
                    ExecuteMission.Impl.FeedbackMessage,
                    mission_feedback_topic,
                    self._mission_feedback_callback,
                    10,
                )
            self.amcl_pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                amcl_pose_topic,
                self._amcl_pose_callback,
                10,
            )
            self.get_logger().info(
                "视觉激活门已启用："
                f"feedback={mission_feedback_topic} amcl_pose={amcl_pose_topic} "
                f"enter={activation_enter_radius:.2f}m exit={activation_exit_radius:.2f}m"
            )
        self.detectors: list[YoloV8Detector] = []
        if enable_detection:
            box_detector = YoloV8Detector(model_path, label_file, score_threshold, nms_threshold, "box_camera")
            self.detectors.append(box_detector)
            self.get_logger().info(f"YOLOv8 detector loaded: name=box_camera model={model_path} classes={len(box_detector.labels)}")
            fire_detector = YoloV8Detector(fire_smoke_model_path, fire_smoke_label_file, fire_smoke_score_threshold, nms_threshold, "fire_smoke")
            self.detectors.append(fire_detector)
            self.get_logger().info(f"YOLOv8 detector loaded: name=fire_smoke model={fire_smoke_model_path} classes={len(fire_detector.labels)}")

        box_order_config = load_box_order_config(str(box_order_config_path))
        effective_box_order_check = enable_detection and enable_box_order_check
        if enable_box_order_check and not enable_detection:
            self.get_logger().warn("箱子摆放异常判断依赖 YOLO 检测，当前 enable_detection=false，已自动关闭")
        if effective_box_order_check:
            self.get_logger().info(f"箱子摆放异常判断已启用：config={box_order_config_path}")

        self.camera = SharedUsbCamera(
            device,
            width,
            height,
            fps,
            jpeg_quality,
            self.detectors,
            enable_box_order_check=effective_box_order_check,
            box_order_config=box_order_config,
            box_order_log_callback=self.get_logger().info,
            box_order_warn_callback=self.get_logger().warn,
            inspection_event_callback=lambda _event_json: None,
            robot_id=robot_id,
            raw_event_interval_sec=raw_event_interval_sec,
            stop_hold_sec=stop_hold_sec,
            stop_epsilon=stop_epsilon,
            vision_activation_gate=self.activation_gate,
        )
        self.camera.open()
        self.cmd_vel_sub = self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_callback, 10)
        handler = type("ConfiguredUsbCameraRequestHandler", (UsbCameraRequestHandler,), {"camera": self.camera, "frame_delay": 1.0 / max(fps, 1)})
        self.server = ReusableThreadingHTTPServer((host, port), handler)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self.get_logger().info(f"YOLO web display started: http://{host}:{port} device={device} size={width}x{height} fps={fps} detection={enable_detection} cmd_vel_topic={cmd_vel_topic} stop_hold_sec={stop_hold_sec} robot_id={robot_id}")

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.camera.update_motion_state(msg)

    def _mission_feedback_callback(self, msg) -> None:
        if self.activation_gate is None:
            return
        feedback = msg.feedback
        point_id = str(feedback.current_point_id).strip()
        if not point_id:
            self.activation_gate.clear_target()
            return
        pose = feedback.current_goal.pose.position
        self.activation_gate.update_target(
            point_id,
            float(pose.x),
            float(pose.y),
            task_id=str(feedback.task_id),
            step_id=str(feedback.current_step_id),
        )

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        if self.activation_gate is None:
            return
        pose = msg.pose.pose.position
        self.activation_gate.update_pose(float(pose.x), float(pose.y))

    def destroy_node(self):
        if hasattr(self, "server"):
            self.server.shutdown()
            self.server.server_close()
        if hasattr(self, "camera"):
            self.camera.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraWebNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
