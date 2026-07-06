import os
from pathlib import Path

import yaml

from robot_web.models import utc_now_iso


def _share_file(package_name, *relative_parts):
    try:
        from ament_index_python.packages import get_package_share_directory

        path = Path(get_package_share_directory(package_name)).joinpath(*relative_parts)
        if path.exists():
            return path
    except Exception:
        return None
    return None


def find_builtin_templates_file(explicit_path=None):
    if explicit_path:
        path = Path(explicit_path).expanduser()
        if path.exists():
            return path

    env_path = os.environ.get("ROBOT_WEB_BUILTIN_TEMPLATES_FILE")
    if env_path:
        path = Path(env_path).expanduser()
        if path.exists():
            return path

    shared = _share_file("robot_web", "config", "builtin_templates.yaml")
    if shared is not None:
        return shared

    source_path = Path.cwd() / "src" / "robot_web" / "config" / "builtin_templates.yaml"
    if source_path.exists():
        return source_path
    fallback = Path("/home/robot/robot_ws/src/robot_web/config/builtin_templates.yaml")
    if fallback.exists():
        return fallback
    return None


def load_builtin_templates(path):
    if path is None:
        return []
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    templates = []
    now = utc_now_iso()
    for raw in data.get("templates", []):
        templates.append({
            "template_id": str(raw["template_id"]),
            "display_name": str(raw["display_name"]),
            "task_type": str(raw["task_type"]).upper(),
            "target_point_ids": [str(item) for item in raw.get("target_point_ids", [])],
            "robot_preference": str(raw.get("robot_preference") or "auto"),
            "builtin": True,
            "sort_order": int(raw.get("sort_order", 0)),
            "created_at": raw.get("created_at") or now,
            "updated_at": raw.get("updated_at") or now,
        })
    return templates
