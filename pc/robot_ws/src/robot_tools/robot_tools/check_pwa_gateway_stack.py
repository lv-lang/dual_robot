import argparse
import base64
import hashlib
import json
import os
import socket
import ssl
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple
from urllib.error import HTTPError, URLError
from urllib.parse import urljoin, urlparse
from urllib.request import Request, urlopen


DEFAULT_PROCESS_PATTERNS = (
    'gzserver',
    'robot_dispatch_node',
    'mission_executor_node',
    'robot_web',
)
DEFAULT_REQUIRED_TOPICS = (
    '/map',
    '/tf',
    '/tf_static',
    '/robot1/scan',
    '/robot1/odom',
    '/robot1/cmd_vel',
    '/robot2/scan',
    '/robot2/odom',
    '/robot2/cmd_vel',
    '/robot_dispatch/markers',
)
DEFAULT_FORBIDDEN_TOPICS = (
    '/cmd_vel',
    '/odom',
    '/scan',
)
DEFAULT_REQUIRED_SERVICES = (
    '/robot_dispatch/create_task',
    '/robot_dispatch/confirm_task_step',
    '/robot_dispatch/pause_task',
    '/robot_dispatch/resume_task',
    '/robot_dispatch/cancel_task',
    '/robot_dispatch/emergency_stop',
    '/robot_dispatch/get_state',
    '/robot_dispatch/get_task_points',
)
SYSTEM_STATUS_VALUES = (
    'stopped',
    'starting',
    'running',
    'degraded',
    'stopping',
    'failed',
    'external',
)
SYSTEM_HEALTH_STATUS_VALUES = (
    'ok',
    'missing',
    'failed',
    'not_checked',
)
SYSTEM_OPERATION_LOG_EVENTS = (
    'system_start',
    'system_start_failed',
    'system_stop',
    'system_stop_failed',
    'system_restart',
    'system_restart_failed',
    'system_external_running',
    'system_recovered',
)


@dataclass
class CheckResult:
    name: str
    ok: bool
    detail: str


class ResultRecorder:
    def __init__(self) -> None:
        self.results: List[CheckResult] = []

    def pass_(self, name: str, detail: str) -> None:
        self.results.append(CheckResult(name, True, detail))

    def fail(self, name: str, detail: str) -> None:
        self.results.append(CheckResult(name, False, detail))

    def exit_code(self) -> int:
        return 1 if any(not result.ok for result in self.results) else 0

    def print(self) -> None:
        for result in self.results:
            prefix = 'PASS' if result.ok else 'FAIL'
            print(f'{prefix} {result.name}: {result.detail}')
        failed = sum(1 for result in self.results if not result.ok)
        passed = len(self.results) - failed
        print(f'SUMMARY passed={passed} failed={failed}')


def _positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError('must be greater than 0')
    return parsed


def _as_list(defaults: Sequence[str], extras: Optional[Sequence[str]]) -> List[str]:
    values = list(defaults)
    if extras:
        values.extend(extras)
    return values


def _proc_cmdlines() -> Iterable[str]:
    proc_root = '/proc'
    for pid in os.listdir(proc_root):
        if not pid.isdigit():
            continue
        cmdline_path = os.path.join(proc_root, pid, 'cmdline')
        try:
            with open(cmdline_path, 'rb') as handle:
                raw = handle.read()
        except OSError:
            continue
        cmdline = raw.replace(b'\0', b' ').decode(errors='replace').strip()
        if cmdline:
            yield cmdline


def _check_processes(
    recorder: ResultRecorder,
    patterns: Sequence[str],
    require_rviz: bool,
) -> None:
    expected = list(patterns)
    if require_rviz:
        expected.append('rviz2')
    cmdlines = list(_proc_cmdlines())
    lowered = [cmdline.lower() for cmdline in cmdlines]
    for pattern in expected:
        needle = pattern.lower()
        matches = [cmdlines[index] for index, value in enumerate(lowered) if needle in value]
        if matches:
            recorder.pass_(
                f'process {pattern}',
                f'{len(matches)} match(es); first="{matches[0][:160]}"',
            )
        else:
            recorder.fail(f'process {pattern}', 'no matching /proc cmdline')


def _http_get(url: str, timeout_sec: float) -> Tuple[int, Dict[str, str], bytes]:
    request = Request(url, headers={'Accept': 'application/json,text/html,*/*'})
    with urlopen(request, timeout=timeout_sec) as response:
        body = response.read(1024 * 1024)
        headers = {key.lower(): value for key, value in response.headers.items()}
        return response.status, headers, body


def _extract_bool(data: Dict[str, Any], paths: Sequence[Sequence[str]]) -> Optional[bool]:
    for path in paths:
        current: Any = data
        for key in path:
            if not isinstance(current, dict) or key not in current:
                current = None
                break
            current = current[key]
        if isinstance(current, bool):
            return current
        if isinstance(current, str):
            lowered = current.lower()
            if lowered in ('online', 'ok', 'healthy', 'true', 'ready'):
                return True
            if lowered in ('offline', 'degraded', 'false', 'unavailable'):
                return False
    return None


def _check_health(
    recorder: ResultRecorder,
    url: str,
    timeout_sec: float,
    allow_dispatch_degraded: bool,
) -> None:
    try:
        status, _headers, body = _http_get(url, timeout_sec)
    except (HTTPError, URLError, TimeoutError, OSError) as exc:
        recorder.fail('robot_web health', f'{url} unreachable: {exc}')
        return
    try:
        data = json.loads(body.decode('utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        recorder.fail('robot_web health', f'{url} returned non-JSON health: {exc}')
        return

    backend_online = _extract_bool(
        data,
        (
            ('backend_online',),
            ('backend', 'online'),
            ('backend', 'ok'),
            ('backend', 'status'),
        ),
    )
    dispatch_online = _extract_bool(
        data,
        (
            ('dispatch_online',),
            ('dispatch', 'online'),
            ('dispatch', 'ok'),
            ('dispatch', 'status'),
        ),
    )

    if backend_online is False:
        recorder.fail('robot_web health', f'{url} status={status} backend_online=false')
        return
    if dispatch_online is False and not allow_dispatch_degraded:
        recorder.fail('robot_web health', f'{url} status={status} dispatch is degraded/offline')
        return
    if backend_online is None or dispatch_online is None:
        recorder.fail(
            'robot_web health',
            f'{url} status={status} JSON does not expose backend and dispatch status',
        )
        return

    recorder.pass_(
        'robot_web health',
        f'{url} status={status} backend_online={backend_online} dispatch_online={dispatch_online}',
    )


def _check_pwa_same_origin(
    recorder: ResultRecorder,
    url: str,
    timeout_sec: float,
    name: str = 'PWA same-origin',
) -> None:
    try:
        status, headers, body = _http_get(url, timeout_sec)
    except (HTTPError, URLError, TimeoutError, OSError) as exc:
        recorder.fail(name, f'{url} unreachable: {exc}')
        return
    content_type = headers.get('content-type', '')
    text_head = body[:512].decode('utf-8', errors='replace').lower()
    if status >= 400:
        recorder.fail(name, f'{url} returned HTTP {status}')
        return
    if '<html' not in text_head and 'text/html' not in content_type:
        recorder.fail(
            name,
            f'{url} did not look like an HTML app shell; content-type={content_type}',
        )
        return
    recorder.pass_(name, f'{url} status={status} content-type={content_type}')


def _load_json_url(
    recorder: ResultRecorder,
    name: str,
    url: str,
    timeout_sec: float,
) -> Optional[Tuple[int, Dict[str, Any]]]:
    try:
        status, _headers, body = _http_get(url, timeout_sec)
    except (HTTPError, URLError, TimeoutError, OSError) as exc:
        recorder.fail(name, f'{url} unreachable: {exc}')
        return None
    try:
        data = json.loads(body.decode('utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        recorder.fail(name, f'{url} returned non-JSON response: {exc}')
        return None
    if not isinstance(data, dict):
        recorder.fail(name, f'{url} JSON root is not an object')
        return None
    return status, data


def _require_str(data: Dict[str, Any], key: str) -> Optional[str]:
    value = data.get(key)
    return value if isinstance(value, str) else None


def _require_bool(data: Dict[str, Any], key: str) -> Optional[bool]:
    value = data.get(key)
    return value if isinstance(value, bool) else None


def _validate_system_health_rows(health: Any) -> Tuple[bool, str, int]:
    if not isinstance(health, list):
        return False, 'health is not a list', 0

    required_keys = ('id', 'label', 'category', 'status', 'required', 'detail')
    for index, row in enumerate(health):
        if not isinstance(row, dict):
            return False, f'health[{index}] is not an object', len(health)
        missing = [key for key in required_keys if key not in row]
        if missing:
            return False, f'health[{index}] missing keys: {", ".join(missing)}', len(health)
        for key in ('id', 'label', 'category', 'status', 'detail'):
            if not isinstance(row[key], str):
                return False, f'health[{index}].{key} is not a string', len(health)
        if row['status'] not in SYSTEM_HEALTH_STATUS_VALUES:
            return False, f'health[{index}].status={row["status"]!r} is not allowed', len(health)
        if not isinstance(row['required'], bool):
            return False, f'health[{index}].required is not a bool', len(health)
    return True, '', len(health)


def _check_system_status(recorder: ResultRecorder, url: str, timeout_sec: float) -> None:
    loaded = _load_json_url(recorder, 'system status API', url, timeout_sec)
    if loaded is None:
        return
    status_code, data = loaded

    required_strs = ('status', 'summary', 'started_at', 'updated_at')
    missing_or_bad_strs = [key for key in required_strs if _require_str(data, key) is None]
    required_bools = ('managed', 'external_running', 'can_start', 'can_stop', 'can_restart')
    missing_or_bad_bools = [key for key in required_bools if _require_bool(data, key) is None]
    missing_keys = [
        key for key in ('profile', 'pid', 'pgid', 'health') if key not in data
    ]

    errors = []
    if missing_or_bad_strs:
        errors.append('bad string fields: ' + ', '.join(missing_or_bad_strs))
    if missing_or_bad_bools:
        errors.append('bad bool fields: ' + ', '.join(missing_or_bad_bools))
    if missing_keys:
        errors.append('missing fields: ' + ', '.join(missing_keys))

    status_value = data.get('status')
    if isinstance(status_value, str) and status_value not in SYSTEM_STATUS_VALUES:
        errors.append(f'status={status_value!r} is not allowed')

    profile = data.get('profile')
    if not isinstance(profile, dict):
        errors.append('profile is not an object')
    else:
        for key in ('id', 'name', 'command'):
            if not isinstance(profile.get(key), str):
                errors.append(f'profile.{key} is not a string')
        command = profile.get('command')
        if isinstance(command, str):
            for token in (
                'ros2 launch robot_bringup robot_dispatch_gazebo.launch.py',
                'gui:=false',
                'launch_rviz:=true',
            ):
                if token not in command:
                    errors.append(f'profile.command missing {token!r}')

    for key in ('pid', 'pgid'):
        if key in data and data[key] is not None and not isinstance(data[key], int):
            errors.append(f'{key} is not an int or null')

    health_ok, health_error, health_count = _validate_system_health_rows(data.get('health'))
    if not health_ok:
        errors.append(health_error)

    managed = data.get('managed')
    external_running = data.get('external_running')
    can_stop = data.get('can_stop')
    can_restart = data.get('can_restart')
    if managed is True and external_running is True:
        errors.append('managed and external_running cannot both be true')
    if external_running is True and (can_stop is True or can_restart is True):
        errors.append('external_running must keep stop/restart disabled')
    if status_value == 'external' and external_running is not True:
        errors.append('status=external requires external_running=true')

    if errors:
        recorder.fail('system status API', f'{url} status={status_code}; ' + '; '.join(errors))
        return

    recorder.pass_(
        'system status API',
        (
            f'{url} status={status_code} system={status_value} managed={managed} '
            f'external_running={external_running} can_start={data.get("can_start")} '
            f'can_stop={can_stop} can_restart={can_restart} health_rows={health_count}'
        ),
    )


def _validate_launch_log_rows(rows: Any) -> Tuple[bool, str, int]:
    if not isinstance(rows, list):
        return False, 'launch_logs is not a list', 0
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            return False, f'launch_logs[{index}] is not an object', len(rows)
        for key in ('line_no', 'stream', 'message', 'timestamp'):
            if key not in row:
                return False, f'launch_logs[{index}] missing {key}', len(rows)
        if not isinstance(row['line_no'], int):
            return False, f'launch_logs[{index}].line_no is not an int', len(rows)
        for key in ('stream', 'message', 'timestamp'):
            if not isinstance(row[key], str):
                return False, f'launch_logs[{index}].{key} is not a string', len(rows)
    return True, '', len(rows)


def _validate_operation_log_rows(rows: Any) -> Tuple[bool, str, int]:
    if not isinstance(rows, list):
        return False, 'operation_logs is not a list', 0
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            return False, f'operation_logs[{index}] is not an object', len(rows)
        for key in ('log_id', 'timestamp', 'level', 'event', 'message'):
            if key not in row:
                return False, f'operation_logs[{index}] missing {key}', len(rows)
            if not isinstance(row[key], str):
                return False, f'operation_logs[{index}].{key} is not a string', len(rows)
        if row['event'] not in SYSTEM_OPERATION_LOG_EVENTS:
            return False, f'operation_logs[{index}].event={row["event"]!r} is not allowed', len(rows)
    return True, '', len(rows)


def _check_system_logs(recorder: ResultRecorder, url: str, timeout_sec: float) -> None:
    loaded = _load_json_url(recorder, 'system logs API', url, timeout_sec)
    if loaded is None:
        return
    status_code, data = loaded

    launch_ok, launch_error, launch_count = _validate_launch_log_rows(data.get('launch_logs'))
    operation_ok, operation_error, operation_count = _validate_operation_log_rows(
        data.get('operation_logs')
    )

    errors = []
    if not launch_ok:
        errors.append(launch_error)
    if not operation_ok:
        errors.append(operation_error)
    if errors:
        recorder.fail('system logs API', f'{url} status={status_code}; ' + '; '.join(errors))
        return

    recorder.pass_(
        'system logs API',
        f'{url} status={status_code} launch_logs={launch_count} operation_logs={operation_count}',
    )


def _default_ws_url(base_url: str, ws_path: str) -> str:
    parsed = urlparse(base_url)
    scheme = 'wss' if parsed.scheme == 'https' else 'ws'
    netloc = parsed.netloc
    return f'{scheme}://{netloc}{ws_path}'


def _host_header(parsed) -> str:
    if parsed.hostname is None:
        return ''
    default_port = 443 if parsed.scheme == 'wss' else 80
    if parsed.port is None or parsed.port == default_port:
        return parsed.hostname
    return f'{parsed.hostname}:{parsed.port}'


def _check_websocket(recorder: ResultRecorder, url: str, timeout_sec: float) -> None:
    parsed = urlparse(url)
    if parsed.scheme not in ('ws', 'wss') or not parsed.hostname:
        recorder.fail('WebSocket reachability', f'invalid WebSocket URL: {url}')
        return
    port = parsed.port or (443 if parsed.scheme == 'wss' else 80)
    path = parsed.path or '/'
    if parsed.query:
        path = f'{path}?{parsed.query}'
    key = base64.b64encode(os.urandom(16)).decode('ascii')
    request = (
        f'GET {path} HTTP/1.1\r\n'
        f'Host: {_host_header(parsed)}\r\n'
        'Upgrade: websocket\r\n'
        'Connection: Upgrade\r\n'
        f'Sec-WebSocket-Key: {key}\r\n'
        'Sec-WebSocket-Version: 13\r\n'
        '\r\n'
    ).encode('ascii')

    try:
        with socket.create_connection((parsed.hostname, port), timeout=timeout_sec) as sock:
            sock.settimeout(timeout_sec)
            stream = sock
            if parsed.scheme == 'wss':
                context = ssl.create_default_context()
                stream = context.wrap_socket(sock, server_hostname=parsed.hostname)
            stream.sendall(request)
            response = b''
            while b'\r\n\r\n' not in response and len(response) < 8192:
                chunk = stream.recv(1024)
                if not chunk:
                    break
                response += chunk
    except OSError as exc:
        recorder.fail('WebSocket reachability', f'{url} unreachable: {exc}')
        return

    text = response.decode('iso-8859-1', errors='replace')
    first_line = text.splitlines()[0] if text.splitlines() else ''
    headers = {}
    for line in text.split('\r\n')[1:]:
        if ':' in line:
            key_name, value = line.split(':', 1)
            headers[key_name.lower()] = value.strip()
    expected_accept = base64.b64encode(
        hashlib.sha1(
            (key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode('ascii')
        ).digest()
    ).decode('ascii')
    if ' 101 ' not in first_line:
        recorder.fail('WebSocket reachability', f'{url} handshake returned "{first_line}"')
        return
    if headers.get('sec-websocket-accept') != expected_accept:
        recorder.fail('WebSocket reachability', f'{url} missing/invalid Sec-WebSocket-Accept')
        return
    recorder.pass_('WebSocket reachability', f'{url} handshake={first_line}')


def _check_ros_graph(
    recorder: ResultRecorder,
    required_topics: Sequence[str],
    forbidden_topics: Sequence[str],
    required_services: Sequence[str],
    wait_sec: float,
) -> None:
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        recorder.fail('ROS graph', f'rclpy import failed; source ROS2 environment: {exc}')
        return

    class GraphProbe(Node):
        def __init__(self) -> None:
            super().__init__('check_pwa_gateway_stack')

    rclpy.init(args=None)
    node = GraphProbe()
    try:
        end_time = time.monotonic() + wait_sec
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
        topic_names = {name for name, _types in node.get_topic_names_and_types()}
        service_names = {name for name, _types in node.get_service_names_and_types()}
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    missing_topics = sorted(set(required_topics) - topic_names)
    present_forbidden = sorted(set(forbidden_topics) & topic_names)
    missing_services = sorted(set(required_services) - service_names)

    if missing_topics:
        recorder.fail('required ROS topics', ', '.join(missing_topics))
    else:
        recorder.pass_('required ROS topics', ', '.join(required_topics))

    if present_forbidden:
        recorder.fail('forbidden global ROS topics', ', '.join(present_forbidden))
    else:
        recorder.pass_('forbidden global ROS topics', ', '.join(forbidden_topics))

    if missing_services:
        recorder.fail('required ROS services', ', '.join(missing_services))
    else:
        recorder.pass_('required ROS services', ', '.join(required_services))


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Smoke-check the Vue3 PWA robot_web gateway validation stack.'
    )
    parser.add_argument('--base-url', default='http://127.0.0.1:8000')
    parser.add_argument('--health-path', default='/api/health')
    parser.add_argument('--pwa-path', default='/')
    parser.add_argument('--system-status-path', default='/api/system/status')
    parser.add_argument('--system-logs-path', default='/api/system/logs?limit=120')
    parser.add_argument('--system-pwa-path', default='/system')
    parser.add_argument('--ws-url', default='')
    parser.add_argument('--ws-path', default='/ws/status')
    parser.add_argument('--timeout-sec', type=_positive_float, default=3.0)
    parser.add_argument('--ros-wait-sec', type=_positive_float, default=2.0)
    parser.add_argument('--allow-dispatch-degraded', action='store_true')
    parser.add_argument('--require-rviz', action='store_true')
    parser.add_argument('--skip-processes', action='store_true')
    parser.add_argument('--skip-ros', action='store_true')
    parser.add_argument('--skip-http', action='store_true')
    parser.add_argument('--skip-system-control', action='store_true')
    parser.add_argument('--skip-ws', action='store_true')
    parser.add_argument(
        '--process-pattern',
        action='append',
        default=[],
        help='Additional /proc cmdline substring to require.',
    )
    parser.add_argument(
        '--topic',
        action='append',
        default=[],
        help='Additional ROS topic to require.',
    )
    parser.add_argument(
        '--service',
        action='append',
        default=[],
        help='Additional ROS service to require.',
    )
    parser.add_argument(
        '--forbidden-topic',
        action='append',
        default=[],
        help='Additional ROS topic that must be absent.',
    )
    return parser.parse_args(list(argv))


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    recorder = ResultRecorder()
    base_url = args.base_url.rstrip('/') + '/'
    health_url = urljoin(base_url, args.health_path.lstrip('/'))
    pwa_url = urljoin(base_url, args.pwa_path.lstrip('/'))
    system_status_url = urljoin(base_url, args.system_status_path.lstrip('/'))
    system_logs_url = urljoin(base_url, args.system_logs_path.lstrip('/'))
    system_pwa_url = urljoin(base_url, args.system_pwa_path.lstrip('/'))
    ws_url = args.ws_url or _default_ws_url(args.base_url, args.ws_path)

    if not args.skip_processes:
        _check_processes(
            recorder,
            _as_list(DEFAULT_PROCESS_PATTERNS, args.process_pattern),
            args.require_rviz,
        )
    if not args.skip_ros:
        _check_ros_graph(
            recorder,
            _as_list(DEFAULT_REQUIRED_TOPICS, args.topic),
            _as_list(DEFAULT_FORBIDDEN_TOPICS, args.forbidden_topic),
            _as_list(DEFAULT_REQUIRED_SERVICES, args.service),
            args.ros_wait_sec,
        )
    if not args.skip_http:
        _check_health(recorder, health_url, args.timeout_sec, args.allow_dispatch_degraded)
        _check_pwa_same_origin(recorder, pwa_url, args.timeout_sec)
        if not args.skip_system_control:
            _check_system_status(recorder, system_status_url, args.timeout_sec)
            _check_system_logs(recorder, system_logs_url, args.timeout_sec)
            _check_pwa_same_origin(
                recorder,
                system_pwa_url,
                args.timeout_sec,
                name='PWA system route',
            )
    if not args.skip_ws:
        _check_websocket(recorder, ws_url, args.timeout_sec)

    recorder.print()
    return recorder.exit_code()


if __name__ == '__main__':
    raise SystemExit(main())
