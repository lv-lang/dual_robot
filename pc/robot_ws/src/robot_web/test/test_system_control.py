import json
import os
import signal
import sys
import time

import pytest

from robot_web.exceptions import GatewayError
from robot_web.system_control import LaunchProfile, ProcessProbe, SystemControlService


class QuietProbe(ProcessProbe):
    def __init__(self, external=False):
        super().__init__(ros_timeout_sec=0.01)
        self.external = external

    def external_stack_running(self, owned_pgid=None):
        return self.external

    def process_contains(self, needles):
        return False

    def ros_nodes(self):
        return None

    def ros_services(self):
        return None

    def ros_topics(self):
        return None


class SettlingProbe(QuietProbe):
    def __init__(self, external_checks):
        super().__init__(external=False)
        self.external_checks = external_checks
        self.settling = False

    def external_stack_running(self, owned_pgid=None):
        if not self.settling:
            return False
        if self.external_checks > 0:
            self.external_checks -= 1
            return True
        return False


def fake_profile(script):
    return LaunchProfile(
        profile_id="test_fake",
        name="Test fake launch",
        command=(sys.executable, "-u", "-c", script),
    )


def make_service(tmp_path, script, probe=None, stop_timeout_sec=0.4, stop_settle_sec=3.0):
    return SystemControlService(
        profile=fake_profile(script),
        metadata_path=tmp_path / "system_control.json",
        log_dir=tmp_path / "logs",
        probe=probe or QuietProbe(),
        stop_timeout_sec=stop_timeout_sec,
        stop_settle_sec=stop_settle_sec,
    )


def wait_until(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_status_start_stop_and_log_tail_use_managed_process_group(tmp_path):
    service = make_service(
        tmp_path,
        "import time; print('fake launch ready', flush=True); time.sleep(60)",
    )

    initial = service.status()
    assert initial["status"] == "stopped"
    assert initial["can_start"] is True
    assert initial["managed"] is False

    started = service.start()
    pid = started["pid"]
    assert started["managed"] is True
    assert started["status"] in {"starting", "running", "degraded"}
    assert started["pgid"] == os.getpgid(pid)
    assert started["pgid"] != os.getpgrp()
    assert (tmp_path / "system_control.json").exists()

    assert wait_until(lambda: any("fake launch ready" in row["message"] for row in service.logs()))
    assert service.stop()["status"] == "stopped"
    assert wait_until(lambda: not service.probe.process_alive(pid))
    assert not (tmp_path / "system_control.json").exists()
    assert any("fake launch ready" in row["message"] for row in service.logs())


def test_process_probe_ignores_zombie_process_rows(monkeypatch):
    probe = ProcessProbe()

    monkeypatch.setattr(
        probe,
        "_process_rows",
        lambda: [
            (123456, "gzserver fake zombie"),
            (os.getpid(), "gzserver current test process"),
        ],
    )
    monkeypatch.setattr(probe, "_is_zombie", lambda pid: pid == 123456)

    assert probe.process_contains(("gzserver",)) is False
    assert probe.external_stack_running() is False


def test_restart_replaces_the_owned_process(tmp_path):
    service = make_service(
        tmp_path,
        "import time; print('restartable launch', flush=True); time.sleep(60)",
    )
    first = service.start()
    first_pid = first["pid"]

    restarted = service.restart()
    second_pid = restarted["pid"]

    try:
        assert restarted["managed"] is True
        assert second_pid != first_pid
        assert wait_until(lambda: not service.probe.process_alive(first_pid))
    finally:
        service.stop()


def test_start_rejects_already_managed_stack(tmp_path):
    service = make_service(
        tmp_path,
        "import time; print('already running', flush=True); time.sleep(60)",
    )
    service.start()

    try:
        with pytest.raises(GatewayError) as already_running:
            service.start()
        assert already_running.value.reason == "system_already_running"
    finally:
        service.stop()


def test_stop_uses_force_kill_when_sigterm_is_ignored(tmp_path):
    service = make_service(
        tmp_path,
        (
            "import signal, time; "
            "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
            "print('ignoring term', flush=True); "
            "time.sleep(60)"
        ),
        stop_timeout_sec=0.1,
    )
    started = service.start()
    pid = started["pid"]

    stopped = service.stop()

    assert stopped["status"] == "stopped"
    assert wait_until(lambda: not service.probe.process_alive(pid))


def test_stop_waits_for_remaining_process_group_children(tmp_path):
    child_pid_path = tmp_path / "child.pid"
    child_code = (
        "import os, pathlib, signal, time; "
        "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
        f"pathlib.Path({str(child_pid_path)!r}).write_text(str(os.getpid())); "
        "time.sleep(60)"
    )
    service = make_service(
        tmp_path,
        (
            "import subprocess, sys, time; "
            f"subprocess.Popen([sys.executable, '-u', '-c', {child_code!r}]); "
            "time.sleep(60)"
        ),
        stop_timeout_sec=0.1,
    )
    started = service.start()
    parent_pid = started["pid"]

    try:
        assert wait_until(child_pid_path.exists)
        child_pid = int(child_pid_path.read_text())
        stopped = service.stop()

        assert stopped["status"] == "stopped"
        assert wait_until(lambda: not service.probe.process_alive(parent_pid))
        assert wait_until(lambda: not service.probe.process_alive(child_pid))
        assert not service.probe.process_group_alive(started["pgid"])
    finally:
        if service.probe.process_alive(parent_pid):
            os.killpg(started["pgid"], signal.SIGKILL)


def test_stop_waits_for_external_probe_to_clear_after_process_exit(tmp_path):
    probe = SettlingProbe(external_checks=2)
    service = make_service(
        tmp_path,
        "import time; print('settling launch', flush=True); time.sleep(60)",
        probe=probe,
        stop_settle_sec=1.0,
    )
    service.start()
    probe.settling = True

    stopped = service.stop()

    assert stopped["status"] == "stopped"
    assert probe.external_checks == 0


def test_external_running_is_read_only_and_rejects_stop_restart(tmp_path):
    service = make_service(
        tmp_path,
        "import time; time.sleep(60)",
        probe=QuietProbe(external=True),
    )

    status = service.status()
    assert status["status"] == "external"
    assert status["external_running"] is True
    assert status["can_start"] is False
    assert status["can_stop"] is False
    assert status["can_restart"] is False

    with pytest.raises(GatewayError) as stop_error:
        service.stop()
    assert stop_error.value.reason == "system_external_running"

    with pytest.raises(GatewayError) as restart_error:
        service.restart()
    assert restart_error.value.reason == "system_external_running"


def test_valid_metadata_recovers_owned_process_after_service_restart(tmp_path):
    script = "import time; print('recoverable launch', flush=True); time.sleep(60)"
    service = make_service(tmp_path, script)
    started = service.start()
    recovered_events = []

    recovered = make_service(tmp_path, script)
    recovered.set_event_sink(
        lambda event_type, message, level="INFO", detail=None: recovered_events.append(event_type)
    )

    try:
        status = recovered.status()
        assert status["managed"] is True
        assert status["pid"] == started["pid"]
        assert recovered_events == ["system_recovered"]
    finally:
        recovered.stop()


def test_stale_metadata_is_removed_without_granting_ownership(tmp_path):
    metadata_path = tmp_path / "system_control.json"
    metadata_path.write_text(json.dumps({
        "owner": "robot_web",
        "profile_id": "test_fake",
        "command": [sys.executable, "-u", "-c", "import time; time.sleep(60)"],
        "pid": 99999999,
        "pgid": 99999999,
        "started_at": "2026-05-23T00:00:00+00:00",
        "log_path": str(tmp_path / "missing.log"),
    }))
    service = SystemControlService(
        profile=fake_profile("import time; time.sleep(60)"),
        metadata_path=metadata_path,
        log_dir=tmp_path / "logs",
        probe=QuietProbe(),
    )

    status = service.status()

    assert status["status"] == "stopped"
    assert status["managed"] is False
    assert not metadata_path.exists()


def test_missing_launch_log_returns_empty_tail(tmp_path):
    service = make_service(tmp_path, "import time; time.sleep(60)")

    assert service.logs(limit=10) == []

