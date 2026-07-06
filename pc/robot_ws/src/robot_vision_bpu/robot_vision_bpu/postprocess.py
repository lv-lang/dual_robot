from dataclasses import dataclass
from typing import Iterable, List, Sequence

from robot_vision_bpu.preprocess import PreprocessedImage
from robot_vision_bpu.runtime import RawDetection


@dataclass(frozen=True)
class PostprocessConfig:
    score_threshold: float
    nms_threshold: float
    max_detections: int
    class_names: Sequence[str]
    class_aware_nms: bool = True


@dataclass(frozen=True)
class Detection:
    class_id: int
    class_name: str
    score: float
    x_min: float
    y_min: float
    x_max: float
    y_max: float


def decode_detections(
    raw_detections: Iterable[RawDetection],
    preprocessed: PreprocessedImage,
    config: PostprocessConfig,
) -> List[Detection]:
    candidates = [
        _scale_detection(raw_detection, preprocessed, config)
        for raw_detection in raw_detections
        if raw_detection.score >= config.score_threshold
    ]
    return _nms(candidates, config)


def _scale_detection(
    raw_detection: RawDetection,
    preprocessed: PreprocessedImage,
    config: PostprocessConfig,
) -> Detection:
    x_min = (raw_detection.x_min - preprocessed.pad_x) / preprocessed.scale_x
    y_min = (raw_detection.y_min - preprocessed.pad_y) / preprocessed.scale_y
    x_max = (raw_detection.x_max - preprocessed.pad_x) / preprocessed.scale_x
    y_max = (raw_detection.y_max - preprocessed.pad_y) / preprocessed.scale_y

    return Detection(
        class_id=raw_detection.class_id,
        class_name=_class_name(raw_detection.class_id, config.class_names),
        score=float(raw_detection.score),
        x_min=_clamp(x_min, 0.0, float(preprocessed.source_width)),
        y_min=_clamp(y_min, 0.0, float(preprocessed.source_height)),
        x_max=_clamp(x_max, 0.0, float(preprocessed.source_width)),
        y_max=_clamp(y_max, 0.0, float(preprocessed.source_height)),
    )


def _class_name(class_id: int, class_names: Sequence[str]) -> str:
    if 0 <= class_id < len(class_names):
        return class_names[class_id]
    return str(class_id)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def _nms(candidates: Sequence[Detection], config: PostprocessConfig) -> List[Detection]:
    pending = sorted(candidates, key=lambda detection: detection.score, reverse=True)
    kept: List[Detection] = []

    while pending and len(kept) < config.max_detections:
        current = pending.pop(0)
        kept.append(current)
        remaining = []
        for candidate in pending:
            same_class = candidate.class_id == current.class_id
            if config.class_aware_nms and not same_class:
                remaining.append(candidate)
                continue
            if _iou(current, candidate) <= config.nms_threshold:
                remaining.append(candidate)
        pending = remaining

    return kept


def _iou(a: Detection, b: Detection) -> float:
    inter_x_min = max(a.x_min, b.x_min)
    inter_y_min = max(a.y_min, b.y_min)
    inter_x_max = min(a.x_max, b.x_max)
    inter_y_max = min(a.y_max, b.y_max)
    inter_width = max(0.0, inter_x_max - inter_x_min)
    inter_height = max(0.0, inter_y_max - inter_y_min)
    intersection = inter_width * inter_height
    if intersection <= 0.0:
        return 0.0

    area_a = max(0.0, a.x_max - a.x_min) * max(0.0, a.y_max - a.y_min)
    area_b = max(0.0, b.x_max - b.x_min) * max(0.0, b.y_max - b.y_min)
    union = area_a + area_b - intersection
    if union <= 0.0:
        return 0.0
    return intersection / union
