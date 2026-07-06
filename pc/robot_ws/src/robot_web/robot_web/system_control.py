import json
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

from robot_web.exceptions import GatewayError
from robot_web.models import utc_now_iso


@dataclass(frozen=True)
class LaunchProfile:
    profile_id: str
    name: str
    command: tuple

    @property
    def command_text(self):
        return " ".join(self.command)

    def to_public(self):
        return {
            "id": self.profile_id,
            "name": self.name,
            "command": self.command_text,
        }


REAL_CONTROL_PLANE_PROFILE = LaunchProfile(
    profile_id="real_robot_control_plane",
    name="Real Robot Control Plane",
    command=(
        "ros2",
        "launch",
        "robot_bringup",
        "real_robot_control_plane.launch.py",
        "launch_rviz:=true",
    ),
)

# Backward-compatible symbol for older imports. The default profile is real-only.
PC_GAZEBO_RVIZ_PROFILE = REAL_CONTROL_PLANE_PROFILE


SYSTEM_EVENT_NAMES = {
    "system_start",
    "system_start_failed",
    "system_stop",
    "system_stop_failed",
    "system_restart",
    "system_restart_failed",
    "system_external_running",
    "system_recovered",
}


class ProcessProbe:
    """Small production probe for process and ROS graph checks."""

    def __init__(self, ros_timeout_sec=0.8):
        self.ros_timeout_sec = float(ros_timeout_sec)

    def process_alive(self, pid):
        try:
            pid = int(pid)
        except (TypeError, ValueError):
            return False
        if pid <= 0:
            return False
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return not self._is_zombie(pid)

    def process_command(self, pid):
        try:
            data = Path(f"/proc/{int(pid)}/cmdline").read_bytes()
        except (OSError, ValueError):
            return []
        return [part.decode(errors="replace") for part in data.split(b"\0") if part]

    def command_matches(self, pid, expected_command):
        command = self.process_command(pid)
        return _command_matches(command, expected_command)

    def process_group_alive(self, pgid):
        try:
            pgid = int(pgid)
        except (TypeError, ValueError):
            return False
        if pgid <= 0:
            return False
        pids = self._pids_for_pgid(pgid)
        if pids:
            return any(not self._is_zombie(pid) for pid in pids)
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def process_contains(self, needles):
        needles = [str(item) for item in needles]
        current_pid = os.getpid()
        for pid, args in self._process_rows():
            if pid == current_pid:
                continue
            if self._is_zombie(pid):
                continue
            if any(needle in args for needle in needles):
                return True
        return False

    def external_stack_running(self, owned_pgid=None):
        needles = (
            "real_robot_control_plane.launch.py",
            "robot_dispatch_node",
            "map_server",
            "rviz2",
        )
        current_pid = os.getpid()
        owned_pids = set()
        if owned_pgid:
            owned_pids = self._pids_for_pgid(owned_pgid)
        for pid, args in self._process_rows():
            if pid == current_pid or pid in owned_pids:
                continue
            if self._is_zombie(pid):
                continue
            if any(needle in args for needle in needles):
                return True
        return False

    def ros_nodes(self):
        return self._ros_list("node", "list")

    def ros_services(self):
        return self._ros_list("service", "list")

    def ros_topics(self):
        return self._ros_list("topic", "list")

    def request_global_estop(self):
        try:
            result = subprocess.run(
                [
                    "ros2",
                    "service",
                    "call",
                    "/robot_dispatch/emergency_stop",
                    "robot_interfaces/srv/EmergencyStop",
                    "{active: true, requester: robot_web, reason: system_control_stop}",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=4.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return False, str(exc)
        if result.returncode != 0:
            return False, (result.stderr or result.stdout).strip()
        return True, result.stdout.strip()

    def _ros_list(self, *args):
        try:
            result = subprocess.run(
                ["ros2", *args],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=self.ros_timeout_sec,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return None
        if result.returncode != 0:
            return None
        return {line.strip() for line in result.stdout.splitlines() if line.strip()}

    def _process_rows(self):
        try:
            result = subprocess.run(
                ["ps", "-eo", "pid=,pgid=,args="],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=1.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return []
        rows = []
        for line in result.stdout.splitlines():
            fields = line.strip().split(None, 2)
            if len(fields) < 3:
                continue
            try:
                rows.append((int(fields[0]), fields[2]))
            except ValueError:
                continue
        return rows

    def _pids_for_pgid(self, pgid):
        try:
            target = int(pgid)
        except (TypeError, ValueError):
            return set()
        try:
            result = subprocess.run(
                ["ps", "-eo", "pid=,pgid="],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=1.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return set()
        pids = set()
        for line in result.stdout.splitlines():
            fields = line.strip().split()
            if len(fields) != 2:
                continue
            try:
                pid = int(fields[0])
                row_pgid = int(fields[1])
            except ValueError:
                continue
            if row_pgid == target:
                pids.add(pid)
        return pids

    @staticmethod
    def _is_zombie(pid):
        try:
            stat = Path(f"/proc/{int(pid)}/stat").read_text()
        except (OSError, ValueError):
            return False
        fields = stat.split()
        return len(fields) > 2 and fields[2] == "Z"


class SystemControlService:
    def __init__(
        self,
        profile=REAL_CONTROL_PLANE_PROFILE,
        metadata_path=None,
        log_dir=None,
        probe=None,
        stop_timeout_sec=8.0,
        stop_settle_sec=3.0,
        start_grace_sec=5.0,
        require_safe_stop=None,
        popen_factory=None,
        now_func=None,
    ):
        self.profile = profile
        self.metadata_path = Path(metadata_path or default_metadata_path()).expanduser()
        self.log_dir = Path(log_dir or default_log_dir()).expanduser()
        self.probe = probe or ProcessProbe()
        self.stop_timeout_sec = float(stop_timeout_sec)
        self.stop_settle_sec = float(stop_settle_sec)
        self.start_grace_sec = float(start_grace_sec)
        self.require_safe_stop = (
            profile.profile_id == REAL_CONTROL_PLANE_PROFILE.profile_id
            if require_safe_stop is None
            else bool(require_safe_stop)
        )
        self.popen_factory = popen_factory or subprocess.Popen
        self.now_func = now_func or utc_now_iso
        self._process = None
        self._last_failure = ""
        self._last_log_path = ""
        self._event_sink = None
        self._pending_events = []
        self.metadata_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._recover_owned_process()

    @classmethod
    def default(cls):
        return cls()

    def set_event_sink(self, event_sink):
        self._event_sink = event_sink
        pending = list(self._pending_events)
        self._pending_events = []
        for event_type, message, level, detail in pending:
            self._emit_event(event_type, message, level=level, detail=detail)

    def status(self):
        managed = self._managed_metadata()
        external_running = False
        if managed is None:
            external_running = self.probe.external_stack_running()

        health = self._health_rows(managed, external_running)
        status = self._status_value(managed, external_running, health)
        can_start = status in {"stopped", "failed"} and not external_running
        can_stop = managed is not None and status in {"starting", "running", "degraded", "failed"}
        can_restart = managed is not None and status in {"starting", "running", "degraded", "failed"}
        summary = _summary_for_status(status)
        return {
            "status": status,
            "summary": summary,
            "managed": managed is not None,
            "external_running": external_running,
            "can_start": can_start,
            "can_stop": can_stop,
            "can_restart": can_restart,
            "profile": self.profile.to_public(),
            "pid": managed.get("pid") if managed else None,
            "pgid": managed.get("pgid") if managed else None,
            "started_at": managed.get("started_at", "") if managed else "",
            "updated_at": self.now_func(),
            "health": health,
        }

    def start(self):
        current = self.status()
        if current["managed"]:
            raise GatewayError(
                409,
                "system_already_running",
                "调度系统已由 App 管理运行中",
                {"status": current},
            )
        if current["external_running"]:
            raise GatewayError(
                409,
                "system_external_running",
                "检测到外部启动的调度系统，App 只能只读显示，不能启动新的管理栈",
                {"status": current},
            )

        started_at = self.now_func()
        log_path = self._new_log_path()
        self._last_log_path = str(log_path)
        log_file = log_path.open("ab", buffering=0)
        try:
            process = self.popen_factory(
                list(self.profile.command),
                stdout=log_file,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                cwd=str(Path.home()),
                start_new_session=True,
            )
        except Exception as exc:
            log_file.close()
            self._last_failure = str(exc)
            raise GatewayError(
                500,
                "system_start_failed",
                f"调度系统启动失败: {exc}",
            ) from exc
        log_file.close()
        self._last_failure = ""

        self._process = process
        try:
            pgid = os.getpgid(process.pid)
        except OSError:
            pgid = process.pid
        metadata = {
            "schema_version": 1,
            "owner": "robot_web",
            "profile_id": self.profile.profile_id,
            "profile_name": self.profile.name,
            "command": list(self.profile.command),
            "pid": process.pid,
            "pgid": pgid,
            "parent_pid": os.getpid(),
            "started_at": started_at,
            "log_path": str(log_path),
        }
        self._write_metadata(metadata)
        return self.status()

    def stop(self):
        managed = self._managed_metadata()
        if managed is None:
            current = self.status()
            if current["external_running"]:
                raise GatewayError(
                    409,
                    "system_external_running",
                    "检测到外部启动的调度系统，App 不允许停止",
                    {"status": current},
                )
            raise GatewayError(
                409,
                "system_not_running",
                "调度系统未由 App 管理运行",
                {"status": current},
            )

        self._last_log_path = managed.get("log_path") or self._last_log_path
        self._request_safe_stop_before_shutdown()
        self._terminate_process_group(managed)
        self._remove_metadata()
        self._process = None
        self._last_failure = ""
        self._wait_for_external_stack_clear()
        return self.status()

    def restart(self):
        managed = self._managed_metadata()
        if managed is None:
            current = self.status()
            if current["external_running"]:
                raise GatewayError(
                    409,
                    "system_external_running",
                    "检测到外部启动的调度系统，App 不允许重启",
                    {"status": current},
                )
            raise GatewayError(
                409,
                "system_not_running",
                "调度系统未由 App 管理运行",
                {"status": current},
            )
        self.stop()
        return self.start()

    def _request_safe_stop_before_shutdown(self):
        if not self.require_safe_stop:
            return
        ok, detail = self.probe.request_global_estop()
        if not ok:
            raise GatewayError(
                409,
                "system_safe_stop_failed",
                "停止控制平面前无法确认全局急停，已拒绝关闭",
                {"detail": detail},
            )

    def logs(self, limit=120):
        limit = _bounded_limit(limit)
        metadata = self._read_metadata()
        log_path = metadata.get("log_path") if metadata else ""
        if not log_path:
            log_path = self._last_log_path or _newest_log_path(self.log_dir)
        return _tail_launch_log(log_path, limit)

    def _managed_metadata(self):
        metadata = self._read_metadata()
        if metadata is None:
            return None
        if not self._metadata_matches_profile(metadata):
            self._remove_metadata()
            return None
        pid = metadata.get("pid")
        if self._process is not None and getattr(self._process, "pid", None) == pid:
            if self._process.poll() is None:
                return metadata
            self._last_failure = f"managed process exited with code {self._process.returncode}"
            return metadata
        if self.probe.process_alive(pid) and self.probe.command_matches(pid, self.profile.command):
            return metadata
        self._remove_metadata()
        return None

    def _recover_owned_process(self):
        metadata = self._read_metadata()
        if metadata is None:
            return
        if not self._metadata_matches_profile(metadata):
            self._remove_metadata()
            return
        pid = metadata.get("pid")
        if self.probe.process_alive(pid) and self.probe.command_matches(pid, self.profile.command):
            self._emit_event(
                "system_recovered",
                "恢复 App-managed 调度系统所有权",
                detail={"pid": pid, "pgid": metadata.get("pgid"), "profile_id": self.profile.profile_id},
            )
            return
        self._remove_metadata()

    def _status_value(self, managed, external_running, health):
        if external_running:
            return "external"
        if managed is None:
            return "failed" if self._last_failure else "stopped"
        if self._process is not None and self._process.poll() is not None:
            return "failed"
        start_age = _age_seconds(managed.get("started_at"))
        required_rows = [row for row in health if row.get("required")]
        missing_required = [
            row for row in required_rows
            if row.get("status") in {"missing", "failed"}
        ]
        if missing_required and start_age is not None and start_age < self.start_grace_sec:
            return "starting"
        if missing_required:
            return "degraded"
        return "running"

    def _health_rows(self, managed, external_running):
        should_probe_ros = managed is not None or external_running
        nodes = self.probe.ros_nodes() if should_probe_ros else None
        services = self.probe.ros_services() if should_probe_ros else None
        topics = self.probe.ros_topics() if should_probe_ros else None
        rows = []
        rows.append(_process_health_row(
            "process.managed_launch",
            "App-managed real control plane",
            bool(managed),
            required=True,
            detail_ok=f"pid={managed.get('pid')} pgid={managed.get('pgid')}" if managed else "",
            detail_missing="no App-managed launch process",
        ))
        rows.append(_process_health_row(
            "process.map_server",
            "PC map_server",
            self.probe.process_contains(("map_server",)),
            required=True,
            detail_missing="no matching process",
        ))
        rows.append(_process_health_row(
            "process.rviz2",
            "RViz",
            self.probe.process_contains(("rviz2",)),
            required=False,
            detail_missing="no matching process",
        ))

        rows.extend([
            _set_health_row("node.robot_dispatch", "robot_dispatch", "node", "/robot_dispatch", nodes),
            _set_health_row("node.map_server", "map_server", "node", "/map_server", nodes),
            _set_health_row("node.mecanum_mission_executor", "mecanum mission executor", "node", "/mecanum/mission_executor", nodes),
            _set_health_row("node.ackermann_mission_executor", "ackermann mission executor", "node", "/ackermann/mission_executor", nodes),
            _set_health_row("service.robot_dispatch_create_task", "robot_dispatch create_task", "service", "/robot_dispatch/create_task", services),
            _set_health_row("service.robot_dispatch_get_state", "robot_dispatch get_state", "service", "/robot_dispatch/get_state", services),
            _set_health_row("service.robot_dispatch_enable_system", "robot_dispatch enable_system", "service", "/robot_dispatch/enable_system", services),
            _set_health_row("service.robot_dispatch_recover_system", "robot_dispatch recover_system", "service", "/robot_dispatch/recover_system", services),
            _set_health_row("service.robot_dispatch_pause_task", "robot_dispatch pause_task", "service", "/robot_dispatch/pause_task", services),
            _set_health_row("service.robot_dispatch_resume_task", "robot_dispatch resume_task", "service", "/robot_dispatch/resume_task", services),
            _set_health_row("service.robot_dispatch_cancel_task", "robot_dispatch cancel_task", "service", "/robot_dispatch/cancel_task", services),
            _set_health_row("service.robot_dispatch_emergency_stop", "robot_dispatch emergency_stop", "service", "/robot_dispatch/emergency_stop", services),
            _set_health_row("topic.map", "shared map", "topic", "/map", topics),
            _set_health_row("topic.system_state", "robot_dispatch system_state", "topic", "/robot_dispatch/system_state", topics),
            _set_health_row("topic.mecanum_heartbeat", "mecanum heartbeat", "topic", "/mecanum/heartbeat", topics),
            _set_health_row("topic.mecanum_scan", "mecanum scan", "topic", "/mecanum/scan", topics),
            _set_health_row("topic.mecanum_odom", "mecanum odom", "topic", "/mecanum/odom", topics),
            _set_health_row("topic.mecanum_cmd_vel", "mecanum cmd_vel", "topic", "/mecanum/cmd_vel", topics),
            _set_health_row("topic.ackermann_heartbeat", "ackermann heartbeat", "topic", "/ackermann/heartbeat", topics),
            _set_health_row("topic.ackermann_scan", "ackermann scan", "topic", "/ackermann/scan", topics),
            _set_health_row("topic.ackermann_odom", "ackermann odom", "topic", "/ackermann/odom", topics),
            _set_health_row("topic.ackermann_cmd_vel", "ackermann cmd_vel", "topic", "/ackermann/cmd_vel", topics),
            _set_health_row("topic.robot_dispatch_markers", "robot_dispatch markers", "topic", "/robot_dispatch/markers", topics, required=False),
            _forbidden_topic_row("forbidden.cmd_vel", "forbidden global cmd_vel", "/cmd_vel", topics),
            _forbidden_topic_row("forbidden.odom", "forbidden global odom", "/odom", topics),
            _forbidden_topic_row("forbidden.scan", "forbidden global scan", "/scan", topics),
        ])
        return rows

    def _terminate_process_group(self, metadata):
        pgid = int(metadata.get("pgid") or 0)
        if pgid <= 0:
            raise GatewayError(500, "system_stop_failed", "缺少可停止的调度系统进程组")
        if pgid == os.getpgrp():
            raise GatewayError(500, "system_stop_failed", "拒绝停止 robot_web 所在进程组")
        if not self.probe.process_group_alive(pgid):
            return
        try:
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            return
        except OSError as exc:
            raise GatewayError(500, "system_stop_failed", f"停止调度系统失败: {exc}") from exc

        deadline = time.monotonic() + self.stop_timeout_sec
        while time.monotonic() < deadline:
            if not self.probe.process_group_alive(pgid):
                return
            time.sleep(0.05)

        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            return
        except OSError as exc:
            raise GatewayError(500, "system_stop_failed", f"强制停止调度系统失败: {exc}") from exc

        kill_deadline = time.monotonic() + 2.0
        while time.monotonic() < kill_deadline:
            if not self.probe.process_group_alive(pgid):
                return
            time.sleep(0.05)
        raise GatewayError(500, "system_stop_failed", "调度系统进程组未能完全退出")

    def _wait_for_external_stack_clear(self):
        deadline = time.monotonic() + self.stop_settle_sec
        while time.monotonic() < deadline:
            if not self.probe.external_stack_running():
                return
            time.sleep(0.05)

    def _metadata_matches_profile(self, metadata):
        return (
            metadata.get("owner") == "robot_web"
            and metadata.get("profile_id") == self.profile.profile_id
            and tuple(metadata.get("command") or ()) == self.profile.command
        )

    def _new_log_path(self):
        safe_time = self.now_func().replace(":", "-").replace("+", "_")
        return self.log_dir / f"{self.profile.profile_id}_{safe_time}.log"

    def _read_metadata(self):
        try:
            return json.loads(self.metadata_path.read_text())
        except FileNotFoundError:
            return None
        except (OSError, json.JSONDecodeError):
            self._remove_metadata()
            return None

    def _write_metadata(self, metadata):
        tmp_path = self.metadata_path.with_suffix(self.metadata_path.suffix + ".tmp")
        tmp_path.write_text(json.dumps(metadata, indent=2, sort_keys=True))
        tmp_path.replace(self.metadata_path)

    def _remove_metadata(self):
        try:
            self.metadata_path.unlink()
        except FileNotFoundError:
            pass

    def _emit_event(self, event_type, message, level="INFO", detail=None):
        if event_type not in SYSTEM_EVENT_NAMES:
            return
        if self._event_sink is None:
            self._pending_events.append((event_type, message, level, detail or {}))
            return
        self._event_sink(event_type, message, level=level, detail=detail or {})


def default_metadata_path():
    return Path.home() / ".ros" / "robot_web" / "system_control.json"


def default_log_dir():
    return Path.home() / ".ros" / "robot_web" / "system_launch_logs"


def _bounded_limit(limit):
    try:
        value = int(limit)
    except (TypeError, ValueError):
        value = 120
    return max(1, min(value, 500))


def _tail_launch_log(log_path, limit):
    if not log_path:
        return []
    try:
        lines = Path(log_path).read_text(errors="replace").splitlines()
    except OSError:
        return []
    start_line = max(0, len(lines) - limit)
    return [
        {
            "line_no": index + 1,
            "stream": "launch",
            "message": line,
            "timestamp": "",
        }
        for index, line in enumerate(lines[start_line:], start=start_line)
    ]


def _newest_log_path(log_dir):
    try:
        paths = sorted(Path(log_dir).glob("*.log"), key=lambda item: item.stat().st_mtime, reverse=True)
    except OSError:
        return ""
    return str(paths[0]) if paths else ""


def _command_matches(actual, expected):
    if not actual:
        return False
    actual = list(actual)
    expected = list(expected)
    if len(actual) < len(expected):
        return False
    if Path(actual[0]).name != Path(expected[0]).name:
        return False
    return actual[1:len(expected)] == expected[1:]


def _age_seconds(started_at):
    if not started_at:
        return None
    from datetime import datetime

    try:
        normalized = str(started_at).replace("Z", "+00:00")
        started = datetime.fromisoformat(normalized)
        now = datetime.fromisoformat(utc_now_iso())
        if started.tzinfo is None:
            return None
        return max(0.0, (now - started).total_seconds())
    except ValueError:
        return None


def _summary_for_status(status):
    return {
        "stopped": "调度系统未启动",
        "starting": "调度系统启动中",
        "running": "调度系统运行中",
        "degraded": "调度系统运行异常",
        "stopping": "调度系统停止中",
        "failed": "调度系统启动失败或已退出",
        "external": "检测到外部运行中的调度系统",
    }.get(status, "调度系统状态未知")


def _process_health_row(row_id, label, present, required=True, detail_ok="", detail_missing="missing"):
    return {
        "id": row_id,
        "label": label,
        "category": "process",
        "status": "ok" if present else "missing",
        "required": bool(required),
        "detail": detail_ok if present else detail_missing,
    }


def _set_health_row(row_id, label, category, expected, values, required=True):
    if values is None:
        return {
            "id": row_id,
            "label": label,
            "category": category,
            "status": "not_checked",
            "required": bool(required),
            "detail": "ROS graph not checked",
        }
    present = expected in values
    return {
        "id": row_id,
        "label": label,
        "category": category,
        "status": "ok" if present else "missing",
        "required": bool(required),
        "detail": expected if present else f"{expected} missing",
    }


def _forbidden_topic_row(row_id, label, topic, topics):
    if topics is None:
        return {
            "id": row_id,
            "label": label,
            "category": "topic",
            "status": "not_checked",
            "required": True,
            "detail": "ROS graph not checked",
        }
    present = topic in topics
    return {
        "id": row_id,
        "label": label,
        "category": "topic",
        "status": "failed" if present else "ok",
        "required": True,
        "detail": f"{topic} must not exist" if present else f"{topic} absent",
    }
