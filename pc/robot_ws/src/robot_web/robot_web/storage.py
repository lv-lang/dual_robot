import json
import sqlite3
from pathlib import Path
from uuid import uuid4

from robot_web.exceptions import GatewayError
from robot_web.models import utc_now_iso


class RobotWebStore:
    def __init__(self, db_path):
        self.db_path = str(db_path)
        if self.db_path != ":memory:":
            Path(self.db_path).expanduser().parent.mkdir(parents=True, exist_ok=True)
        self._init_schema()

    def _connect(self):
        connection = sqlite3.connect(self.db_path)
        connection.row_factory = sqlite3.Row
        return connection

    def _init_schema(self):
        with self._connect() as connection:
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS templates (
                    template_id TEXT PRIMARY KEY,
                    display_name TEXT NOT NULL,
                    task_type TEXT NOT NULL,
                    target_point_ids_json TEXT NOT NULL,
                    robot_preference TEXT NOT NULL,
                    sort_order INTEGER NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS logs (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    created_at TEXT NOT NULL,
                    level TEXT NOT NULL,
                    event_type TEXT NOT NULL,
                    message TEXT NOT NULL,
                    task_id TEXT,
                    template_id TEXT,
                    detail_json TEXT NOT NULL
                )
                """
            )

    @staticmethod
    def _template_from_row(row):
        return {
            "template_id": row["template_id"],
            "display_name": row["display_name"],
            "task_type": row["task_type"],
            "target_point_ids": json.loads(row["target_point_ids_json"]),
            "robot_preference": row["robot_preference"],
            "builtin": False,
            "sort_order": row["sort_order"],
            "created_at": row["created_at"],
            "updated_at": row["updated_at"],
        }

    def list_user_templates(self):
        with self._connect() as connection:
            rows = connection.execute(
                """
                SELECT * FROM templates
                ORDER BY sort_order ASC, display_name ASC, template_id ASC
                """
            ).fetchall()
        return [self._template_from_row(row) for row in rows]

    def get_user_template(self, template_id):
        with self._connect() as connection:
            row = connection.execute(
                "SELECT * FROM templates WHERE template_id = ?",
                (template_id,),
            ).fetchone()
        return None if row is None else self._template_from_row(row)

    def create_user_template(self, payload):
        now = utc_now_iso()
        template_id = "user_" + uuid4().hex[:12]
        with self._connect() as connection:
            connection.execute(
                """
                INSERT INTO templates (
                    template_id, display_name, task_type, target_point_ids_json,
                    robot_preference, sort_order, created_at, updated_at
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    template_id,
                    payload["display_name"],
                    payload["task_type"],
                    json.dumps(payload["target_point_ids"]),
                    payload["robot_preference"],
                    payload["sort_order"],
                    now,
                    now,
                ),
            )
        return self.get_user_template(template_id)

    def update_user_template(self, template_id, payload):
        if self.get_user_template(template_id) is None:
            raise GatewayError(404, "template_not_found", "template not found")
        now = utc_now_iso()
        with self._connect() as connection:
            connection.execute(
                """
                UPDATE templates
                SET display_name = ?,
                    task_type = ?,
                    target_point_ids_json = ?,
                    robot_preference = ?,
                    sort_order = ?,
                    updated_at = ?
                WHERE template_id = ?
                """,
                (
                    payload["display_name"],
                    payload["task_type"],
                    json.dumps(payload["target_point_ids"]),
                    payload["robot_preference"],
                    payload["sort_order"],
                    now,
                    template_id,
                ),
            )
        return self.get_user_template(template_id)

    def delete_user_template(self, template_id):
        with self._connect() as connection:
            cursor = connection.execute(
                "DELETE FROM templates WHERE template_id = ?",
                (template_id,),
            )
        if cursor.rowcount == 0:
            raise GatewayError(404, "template_not_found", "template not found")
        return {"deleted": True, "template_id": template_id}

    def reorder_user_templates(self, orders):
        if not isinstance(orders, list):
            raise GatewayError(400, "invalid_reorder", "orders must be a list")
        now = utc_now_iso()
        with self._connect() as connection:
            for item in orders:
                template_id = str(item.get("template_id", ""))
                sort_order = int(item.get("sort_order"))
                cursor = connection.execute(
                    """
                    UPDATE templates
                    SET sort_order = ?, updated_at = ?
                    WHERE template_id = ?
                    """,
                    (sort_order, now, template_id),
                )
                if cursor.rowcount == 0:
                    raise GatewayError(
                        404,
                        "template_not_found",
                        "template not found",
                        {"template_id": template_id},
                    )
        return self.list_user_templates()

    @staticmethod
    def _log_from_row(row):
        level = str(row["level"]).lower()
        detail = json.loads(row["detail_json"])
        return {
            "id": row["id"],
            "log_id": str(row["id"]),
            "created_at": row["created_at"],
            "timestamp": row["created_at"],
            "level": level,
            "event_type": row["event_type"],
            "event": row["event_type"],
            "message": row["message"],
            "task_id": row["task_id"],
            "task_display_name": detail.get("display_name") or detail.get("task_display_name", ""),
            "template_id": row["template_id"],
            "detail": detail,
        }

    def append_log(self, event_type, message, level="INFO", task_id=None, template_id=None, detail=None, timestamp=None):
        now = timestamp or utc_now_iso()
        with self._connect() as connection:
            cursor = connection.execute(
                """
                INSERT INTO logs (
                    created_at, level, event_type, message, task_id,
                    template_id, detail_json
                )
                VALUES (?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    now,
                    level,
                    event_type,
                    message,
                    task_id,
                    template_id,
                    json.dumps(detail or {}, sort_keys=True),
                ),
            )
            log_id = cursor.lastrowid
            row = connection.execute(
                "SELECT * FROM logs WHERE id = ?",
                (log_id,),
            ).fetchone()
            return self._log_from_row(row)

    def clear_logs(self, exclude_prefix=None, event_prefix=None):
        with self._connect() as connection:
            if event_prefix:
                cursor = connection.execute(
                    """
                    DELETE FROM logs
                    WHERE event_type LIKE ?
                    """,
                    (f"{event_prefix}%",),
                )
            elif exclude_prefix:
                cursor = connection.execute(
                    """
                    DELETE FROM logs
                    WHERE event_type NOT LIKE ?
                    """,
                    (f"{exclude_prefix}%",),
                )
            else:
                cursor = connection.execute("DELETE FROM logs")
        return int(cursor.rowcount)

    def list_logs(self, limit=100, event_prefix=None, exclude_prefix=None):
        limit = max(1, min(int(limit), 500))
        with self._connect() as connection:
            if event_prefix:
                rows = connection.execute(
                    """
                    SELECT * FROM logs
                    WHERE event_type LIKE ?
                    ORDER BY id DESC
                    LIMIT ?
                    """,
                    (f"{event_prefix}%", limit),
                ).fetchall()
            elif exclude_prefix:
                rows = connection.execute(
                    """
                    SELECT * FROM logs
                    WHERE event_type NOT LIKE ?
                    ORDER BY id DESC
                    LIMIT ?
                    """,
                    (f"{exclude_prefix}%", limit),
                ).fetchall()
            else:
                rows = connection.execute(
                    """
                    SELECT * FROM logs
                    ORDER BY id DESC
                    LIMIT ?
                    """,
                    (limit,),
                ).fetchall()
        return [self._log_from_row(row) for row in rows]
