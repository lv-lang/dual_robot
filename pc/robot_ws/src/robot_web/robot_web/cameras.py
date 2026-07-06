"""态势页摄像头/视觉画面配置。

当前视觉未接入: 默认双车各一路占位(stream_url 为空 → 前端显示占位)。
视觉就绪后(通常 web_video_server 把图像话题转成 MJPEG), 在 cameras.yaml 或
环境变量 ROBOT_WEB_CAMERAS(JSON) 填入 stream_url 即可, 前端无需改动。

cameras.yaml 示例:
  cameras:
    - robot_id: mecanum
      label: mecanum车检测画面
      stream_url: http://172.20.10.12:8080/stream?topic=/mecanum/camera/color/image_raw
      kind: mjpeg
    - robot_id: ackermann
      label: ackermann车检测画面
      stream_url: ""
      kind: mjpeg
"""

import json
import os
from pathlib import Path

import yaml

_DEFAULT = [
    {"robot_id": "mecanum", "label": "mecanum车检测画面", "stream_url": "", "kind": "mjpeg"},
    {"robot_id": "ackermann", "label": "ackermann车检测画面", "stream_url": "", "kind": "mjpeg"},
]


def _find_cameras_file(explicit_path=None):
    candidates = []
    if explicit_path:
        candidates.append(Path(explicit_path).expanduser())
    env = os.environ.get("ROBOT_WEB_CAMERAS_FILE")
    if env:
        candidates.append(Path(env).expanduser())
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            Path(get_package_share_directory("robot_bringup")) / "config" / "cameras.yaml"
        )
    except Exception:
        pass
    candidates += [
        Path.cwd() / "src" / "robot_bringup" / "config" / "cameras.yaml",
        Path("/home/robot/robot_ws/src/robot_bringup/config/cameras.yaml"),
    ]
    for path in candidates:
        if path and path.is_file():
            return path
    return None


def _normalize(entries):
    cameras = []
    for raw in entries or []:
        if not isinstance(raw, dict):
            continue
        cameras.append(
            {
                "robot_id": str(raw.get("robot_id", "")),
                "label": str(raw.get("label") or raw.get("robot_id") or "摄像头"),
                "stream_url": str(raw.get("stream_url", "") or ""),
                "kind": str(raw.get("kind", "mjpeg") or "mjpeg"),
            }
        )
    return cameras


def load_cameras(explicit_path=None):
    # 环境变量 JSON 优先(便于现场临时填地址)
    env_json = os.environ.get("ROBOT_WEB_CAMERAS")
    if env_json:
        try:
            return {"cameras": _normalize(json.loads(env_json))}
        except Exception:
            pass

    path = _find_cameras_file(explicit_path)
    if path is not None:
        try:
            with path.open("r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
            return {"cameras": _normalize(data.get("cameras"))}
        except Exception:
            pass

    return {"cameras": _normalize(_DEFAULT)}
