import json
import builtins

from robot_tools import demo_event_keyboard


class FakeResponse:
    status = 200

    def __init__(self, payload):
        self.payload = payload

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return False

    def read(self, _limit):
        return json.dumps(self.payload).encode('utf-8')


def test_once_posts_demo_event_key(monkeypatch, capsys):
    seen = {}

    def fake_urlopen(request, timeout):
        seen['url'] = request.full_url
        seen['method'] = request.get_method()
        seen['timeout'] = timeout
        seen['payload'] = json.loads(request.data.decode('utf-8'))
        return FakeResponse({'message': '发现当前巡检点 P3 有火情'})

    monkeypatch.setattr(demo_event_keyboard, 'urlopen', fake_urlopen)

    assert demo_event_keyboard.main([
        '--base-url',
        'http://robot.test:8000',
        '--once',
        '8',
    ]) == 0

    assert seen == {
        'url': 'http://robot.test:8000/api/demo-events/8',
        'method': 'POST',
        'timeout': 2.0,
        'payload': {'point_id': 'P3'},
    }
    assert '已发送 8' in capsys.readouterr().out


def test_once_can_override_inspection_point(monkeypatch):
    seen = {}

    def fake_urlopen(request, timeout):
        seen['url'] = request.full_url
        seen['payload'] = json.loads(request.data.decode('utf-8'))
        return FakeResponse({'message': '发现当前巡检点 P2 有火情'})

    monkeypatch.setattr(demo_event_keyboard, 'urlopen', fake_urlopen)

    assert demo_event_keyboard.main([
        '--base-url',
        'http://robot.test:8000',
        '--point',
        'p2',
        '--once',
        '8',
    ]) == 0

    assert seen == {
        'url': 'http://robot.test:8000/api/demo-events/8',
        'payload': {'point_id': 'P2'},
    }


def test_once_posts_space_clear_key(monkeypatch):
    seen = {}

    def fake_urlopen(request, timeout):
        seen['url'] = request.full_url
        seen['method'] = request.get_method()
        seen['timeout'] = timeout
        seen['payload'] = json.loads(request.data.decode('utf-8'))
        return FakeResponse({'message': '事件日志已清空'})

    monkeypatch.setattr(demo_event_keyboard, 'urlopen', fake_urlopen)

    assert demo_event_keyboard.main([
        '--base-url',
        'http://robot.test:8000',
        '--once',
        'space',
    ]) == 0

    assert seen == {
        'url': 'http://robot.test:8000/api/demo-events/clear',
        'method': 'POST',
        'timeout': 2.0,
        'payload': {},
    }


def test_interactive_quit_does_not_post_start(monkeypatch):
    seen = []

    def fake_urlopen(request, timeout):
        seen.append((request.full_url, json.loads(request.data.decode('utf-8'))))
        return FakeResponse({'message': 'ok'})

    monkeypatch.setattr(demo_event_keyboard, 'urlopen', fake_urlopen)
    monkeypatch.setattr(builtins, 'input', lambda _prompt: 'q')

    assert demo_event_keyboard.main(['--base-url', 'http://robot.test:8000']) == 0

    assert seen == []


def test_interactive_can_switch_inspection_point(monkeypatch, capsys):
    seen = []
    inputs = iter(['p1', '18', 'q'])

    def fake_urlopen(request, timeout):
        seen.append((request.full_url, json.loads(request.data.decode('utf-8'))))
        return FakeResponse({'message': 'ok'})

    monkeypatch.setattr(demo_event_keyboard, 'urlopen', fake_urlopen)
    monkeypatch.setattr(builtins, 'input', lambda _prompt: next(inputs))

    assert demo_event_keyboard.main(['--base-url', 'http://robot.test:8000']) == 0

    assert seen == [
        ('http://robot.test:8000/api/demo-events/18', {'point_id': 'P1'}),
    ]
    assert '当前巡检点已切换为 P1' in capsys.readouterr().out


def test_invalid_key_does_not_post(monkeypatch, capsys):
    def fake_urlopen(_request, _timeout):
        raise AssertionError('should not post invalid key')

    monkeypatch.setattr(demo_event_keyboard, 'urlopen', fake_urlopen)

    assert demo_event_keyboard._send_key('http://robot.test:8000', '0', 2.0) is False
    assert '无效输入' in capsys.readouterr().out
