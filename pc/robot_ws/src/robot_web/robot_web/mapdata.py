"""加载当前实车地图供 App 态势图底图使用。

把 ROS map_server 的 .pgm 转成 PNG(base64) 并随 origin/resolution 下发,
前端据此把世界坐标对齐到底图像素。结果按 YAML 和图像 mtime 缓存。
"""

import base64
import io
import os
from pathlib import Path

import yaml

_CACHE = {}
DEFAULT_MAP_BASENAME = "real_competition_map.yaml"
FALLBACK_MAP_BASENAME = "lv_home.yaml"
DEFAULT_CROP_PADDING_PX = 32
BACKGROUND_THRESHOLD = 12


def _robot_bringup_map_candidate(map_basename):
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory("robot_bringup"))
            / "maps"
            / map_basename
        )
    except Exception:
        return None


def find_map_yaml(explicit_path=None):
    candidates = []
    if explicit_path:
        candidates.append(Path(explicit_path).expanduser())
    env = os.environ.get("ROBOT_WEB_MAP_YAML")
    if env:
        candidates.append(Path(env).expanduser())
    for map_basename in (DEFAULT_MAP_BASENAME, FALLBACK_MAP_BASENAME):
        ament_candidate = _robot_bringup_map_candidate(map_basename)
        if ament_candidate is not None:
            candidates.append(ament_candidate)
        candidates += [
            Path.cwd() / "src" / "robot_bringup" / "maps" / map_basename,
            Path("/home/robot/robot_ws/src/robot_bringup/maps") / map_basename,
        ]
    for path in candidates:
        if path and path.is_file():
            return path
    return None


def _background_value(image):
    width, height = image.size
    corners = [
        image.getpixel((0, 0)),
        image.getpixel((width - 1, 0)),
        image.getpixel((0, height - 1)),
        image.getpixel((width - 1, height - 1)),
    ]
    return max(set(corners), key=corners.count)


def _content_bbox(image):
    background = _background_value(image)
    mask = image.point(
        lambda value: 255 if abs(int(value) - int(background)) > BACKGROUND_THRESHOLD else 0
    )
    return mask.getbbox()


def _expand_bbox(bbox, image_size, padding_px):
    if bbox is None:
        return None
    left, top, right, bottom = bbox
    width, height = image_size
    padding = max(0, int(padding_px))
    return (
        max(0, left - padding),
        max(0, top - padding),
        min(width, right + padding),
        min(height, bottom + padding),
    )


def _crop_origin(origin, resolution, source_height, crop_box):
    left, _, _, bottom = crop_box
    return [
        float(origin[0]) + left * resolution,
        float(origin[1]) + (source_height - bottom) * resolution,
        float(origin[2]) if len(origin) > 2 else 0.0,
    ]


def load_map_payload(explicit_path=None, crop_unknown=True, crop_padding_px=DEFAULT_CROP_PADDING_PX):
    path = find_map_yaml(explicit_path)
    if path is None:
        return {"available": False, "reason": "map yaml not found"}

    with path.open("r", encoding="utf-8") as stream:
        meta = yaml.safe_load(stream) or {}

    try:
        yaml_mtime = path.stat().st_mtime
    except OSError:
        yaml_mtime = 0.0

    image_name = meta.get("image")
    image_path = (path.parent / image_name) if image_name else None
    try:
        image_mtime = image_path.stat().st_mtime if image_path else 0.0
    except OSError:
        image_mtime = 0.0

    cache_key = (str(path), bool(crop_unknown), int(crop_padding_px))
    cached = _CACHE.get(cache_key)
    if (
        cached
        and cached.get("yaml_mtime") == yaml_mtime
        and cached.get("image_mtime") == image_mtime
    ):
        return cached["payload"]

    origin = meta.get("origin", [0.0, 0.0, 0.0]) or [0.0, 0.0, 0.0]
    payload = {
        "available": False,
        "resolution": float(meta.get("resolution", 0.05)),
        "origin": [
            float(origin[0]),
            float(origin[1]),
            float(origin[2]) if len(origin) > 2 else 0.0,
        ],
        "map_version": str(meta.get("map_version", "")),
    }

    if image_path and image_path.is_file():
        try:
            from PIL import Image

            with Image.open(image_path) as image:
                image = image.convert("L")
                source_width, source_height = image.width, image.height
                crop_box = None
                if crop_unknown:
                    crop_box = _expand_bbox(
                        _content_bbox(image),
                        image.size,
                        crop_padding_px,
                    )
                if crop_box is not None:
                    image = image.crop(crop_box)
                    payload["origin"] = _crop_origin(
                        origin,
                        payload["resolution"],
                        source_height,
                        crop_box,
                    )
                width, height = image.width, image.height
                buffer = io.BytesIO()
                image.save(buffer, format="PNG")
                encoded = base64.b64encode(buffer.getvalue()).decode("ascii")
            payload.update(
                {
                    "available": True,
                    "width": width,
                    "height": height,
                    "image_base64": encoded,
                    "image_mime": "image/png",
                    "source_width": source_width,
                    "source_height": source_height,
                    "crop": {
                        "enabled": crop_box is not None,
                        "x": crop_box[0] if crop_box is not None else 0,
                        "y": crop_box[1] if crop_box is not None else 0,
                        "width": width,
                        "height": height,
                    },
                }
            )
        except ImportError:
            payload["reason"] = "Pillow not installed (pip install pillow)"
        except Exception as exc:  # noqa: BLE001
            payload["reason"] = f"image load failed: {exc}"
    else:
        payload["reason"] = "image file not found"

    _CACHE[cache_key] = {
        "yaml_mtime": yaml_mtime,
        "image_mtime": image_mtime,
        "payload": payload,
    }
    return payload
