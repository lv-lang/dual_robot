import math
import os
from pathlib import Path

import yaml

from robot_web.models import POINT_KIND_BY_ID, TaskPointInfo, enum_name


def _yaw_from_quaternion(z, w):
    # 平面机器人只用 z/w 即可
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def find_default_task_points_file(explicit_path=None):
    if explicit_path:
        path = Path(explicit_path).expanduser()
        if path.is_dir():
            nested = path / "task_points.yaml"
            if nested.exists():
                return nested
        if path.is_file():
            return path

    env_path = os.environ.get("ROBOT_WEB_TASK_POINTS_FILE")
    if env_path:
        path = Path(env_path).expanduser()
        if path.is_dir():
            nested = path / "task_points.yaml"
            if nested.exists():
                return nested
        if path.is_file():
            return path

    # 优先权威实车点位 real_task_points.yaml(与控制平面/真实地图同版本),
    # 再退到旧 schematic task_points.yaml。
    try:
        from ament_index_python.packages import get_package_share_directory

        for pkg, rel in (
            ("robot_bringup", "config/real_task_points.yaml"),
            ("robot_dispatch", "config/task_points.yaml"),
        ):
            path = Path(get_package_share_directory(pkg)) / rel
            if path.exists():
                return path
    except Exception:
        pass

    candidates = [
        Path.cwd() / "src" / "robot_bringup" / "config" / "real_task_points.yaml",
        Path("/home/robot/robot_ws/src/robot_bringup/config/real_task_points.yaml"),
        Path.cwd() / "src" / "robot_dispatch" / "config" / "task_points.yaml",
        Path("/home/robot/robot_ws/src/robot_dispatch/config/task_points.yaml"),
    ]
    for path in candidates:
        if path.exists():
            return path
    return None


def load_task_points_from_yaml(path):
    if path is None:
        return {}
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    points = {}
    for point_id, raw in (data.get("points") or {}).items():
        kind = str(raw.get("kind", "")).upper()
        if kind == "WAITING_AREA":
            normalized_kind = "WAITING_AREA"
        elif kind in {"PICKUP", "DELIVERY", "INSPECTION"}:
            normalized_kind = kind
        else:
            normalized_kind = "UNKNOWN"
        has_xy = raw.get("x") is not None and raw.get("y") is not None
        points[str(point_id)] = TaskPointInfo(
            point_id=str(point_id),
            kind=normalized_kind,
            label=str(raw.get("label") or point_id),
            temporary=False,
            x=float(raw.get("x", 0.0) or 0.0),
            y=float(raw.get("y", 0.0) or 0.0),
            yaw=float(raw.get("yaw", 0.0) or 0.0),
            has_pose=bool(has_xy),
        )
    return points


def task_point_from_message(message):
    x = y = yaw = 0.0
    has_pose = False
    pose_stamped = getattr(message, "pose", None)
    pose = getattr(pose_stamped, "pose", None)
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    if position is not None:
        x = float(getattr(position, "x", 0.0))
        y = float(getattr(position, "y", 0.0))
        if orientation is not None:
            yaw = _yaw_from_quaternion(
                float(getattr(orientation, "z", 0.0)),
                float(getattr(orientation, "w", 1.0)),
            )
        has_pose = True
    return TaskPointInfo(
        point_id=str(getattr(message, "point_id", "")),
        kind=enum_name(getattr(message, "kind", 0), POINT_KIND_BY_ID),
        label=str(getattr(message, "label", "") or getattr(message, "point_id", "")),
        temporary=bool(getattr(message, "temporary", False)),
        x=x,
        y=y,
        yaw=yaw,
        has_pose=has_pose,
    )


def catalog_to_json(points):
    return [point.to_dict() for point in sorted(points.values(), key=lambda item: item.point_id)]
