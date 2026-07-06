import json

from robot_tools import check_pwa_gateway_stack as smoke


def _json_response(data):
    return 200, {'content-type': 'application/json'}, json.dumps(data).encode('utf-8')


def _html_response():
    return 200, {'content-type': 'text/html; charset=utf-8'}, b'<!doctype html><html></html>'


def _system_status(**overrides):
    status = {
        'status': 'stopped',
        'summary': 'system stopped',
        'managed': False,
        'external_running': False,
        'can_start': True,
        'can_stop': False,
        'can_restart': False,
        'profile': {
            'id': 'pc_gazebo_rviz',
            'name': 'PC Gazebo + RViz',
            'command': (
                'ros2 launch robot_bringup robot_dispatch_gazebo.launch.py '
                'gui:=false launch_rviz:=true'
            ),
        },
        'pid': None,
        'pgid': None,
        'started_at': '',
        'updated_at': '2026-05-23T00:00:00Z',
        'health': [
            {
                'id': 'process.gzserver',
                'label': 'Gazebo server',
                'category': 'process',
                'status': 'missing',
                'required': True,
                'detail': 'no matching process',
            }
        ],
    }
    status.update(overrides)
    return status


def _system_logs():
    return {
        'launch_logs': [
            {
                'line_no': 1,
                'stream': 'launch',
                'message': '[INFO] ready',
                'timestamp': '',
            }
        ],
        'operation_logs': [
            {
                'log_id': '1',
                'timestamp': '2026-05-23T00:00:00Z',
                'level': 'info',
                'event': 'system_start',
                'message': 'system starting',
            }
        ],
    }


def test_default_http_smoke_checks_system_control_contract(monkeypatch):
    seen_urls = []

    def fake_http_get(url, timeout_sec):
        seen_urls.append(url)
        if url.endswith('/api/health'):
            return _json_response({'backend_online': True, 'dispatch_online': False})
        if url.endswith('/api/system/status'):
            return _json_response(_system_status())
        if url.endswith('/api/system/logs?limit=120'):
            return _json_response(_system_logs())
        if url.endswith('/') or url.endswith('/system'):
            return _html_response()
        raise AssertionError(f'unexpected URL: {url}')

    monkeypatch.setattr(smoke, '_http_get', fake_http_get)

    assert smoke.main(
        [
            '--base-url',
            'http://robot.test:8000',
            '--skip-processes',
            '--skip-ros',
            '--skip-ws',
            '--allow-dispatch-degraded',
        ]
    ) == 0
    assert 'http://robot.test:8000/api/system/status' in seen_urls
    assert 'http://robot.test:8000/api/system/logs?limit=120' in seen_urls
    assert 'http://robot.test:8000/system' in seen_urls


def test_system_status_rejects_external_running_stop_controls(monkeypatch):
    def fake_http_get(url, timeout_sec):
        return _json_response(
            _system_status(
                status='external',
                external_running=True,
                can_stop=True,
                can_restart=False,
            )
        )

    monkeypatch.setattr(smoke, '_http_get', fake_http_get)
    recorder = smoke.ResultRecorder()

    smoke._check_system_status(recorder, 'http://robot.test/api/system/status', 1.0)

    assert recorder.exit_code() == 1
    assert 'external_running must keep stop/restart disabled' in recorder.results[0].detail
