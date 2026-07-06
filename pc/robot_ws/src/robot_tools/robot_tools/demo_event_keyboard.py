import argparse
import json
import sys
from typing import Dict, List, Optional
from urllib.error import HTTPError, URLError
from urllib.parse import urljoin
from urllib.request import Request, urlopen


DEMO_MENU: Dict[str, str] = {
    '1': 'mecanum 已确认到达取货点 A',
    '2': 'mecanum 已确认到达配送点 D',
    '3': 'ackermann 已确认到达取货点 B',
    '4': 'ackermann 已确认到达配送点 C',
    '5': '{point} 发现烟雾异常，分配 mecanum 前往复检',
    '6': '{point} 发现货物堆叠异常，分配 mecanum 前往复检',
    '7': '{point} 复检确认异常，已通知对应区域安全负责人前去处理',
    '8': '{point} 火灾警告',
    '9': '关闭火灾告警',
    '10': 'ackermann 已确认到达巡检点 {point}，并开始巡检',
    '11': '{point} 巡检正常',
    '12': 'ackermann 当前任务已结束，正在返回等待区',
    '13': 'mecanum 当前任务已结束，正在返回等待区',
    '14': 'mecanum 正在前往取货点 A',
    '15': 'ackermann 正在前往取货点 B',
    '16': 'mecanum 正在前往配送点 D',
    '17': 'ackermann 正在前往配送点 C',
    '18': 'ackermann 正在前往巡检点 {point}',
    '19': 'mecanum 正在前往巡检点 P3',
    ' ': '清空事件日志栏',
}
INSPECTION_EVENT_KEYS = {'5', '6', '7', '8', '9', '10', '11', '18'}
INSPECTION_POINTS = {'P1', 'P2', 'P3'}


def _positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError('must be greater than 0')
    return parsed


def _endpoint(base_url: str, key: str) -> str:
    if key == ' ':
        key = 'clear'
    return urljoin(base_url.rstrip('/') + '/', f'api/demo-events/{key}')


def _normalize_point(value: str) -> str:
    point_id = value.strip().upper()
    if point_id not in INSPECTION_POINTS:
        raise argparse.ArgumentTypeError('inspection point must be P1, P2, or P3')
    return point_id


def _payload_for_key(key: str, inspection_point: str) -> bytes:
    payload = {'point_id': inspection_point} if key in INSPECTION_EVENT_KEYS else {}
    return json.dumps(payload).encode('utf-8')


def _post_demo_event(base_url: str, key: str, timeout_sec: float, inspection_point: str = 'P3') -> str:
    request = Request(
        _endpoint(base_url, key),
        data=_payload_for_key(key, inspection_point),
        headers={'Content-Type': 'application/json', 'Accept': 'application/json'},
        method='POST',
    )
    with urlopen(request, timeout=timeout_sec) as response:
        body = response.read(1024 * 1024)
    data = json.loads(body.decode('utf-8'))
    message = str(data.get('message') or data.get('log', {}).get('message') or '')
    return message


def _menu_sort_key(key: str) -> int:
    if key == ' ':
        return 99
    return int(key)


def _print_menu(base_url: str, inspection_point: str) -> None:
    print('')
    print(f'robot_web: {base_url}')
    print('比赛演示事件模拟器')
    print(f'当前巡检点: {inspection_point}  可输入 p1 / p2 / p3 切换')
    for key in sorted(DEMO_MENU, key=_menu_sort_key):
        label = '空格' if key == ' ' else key
        print(f'{label}  {DEMO_MENU[key].format(point=inspection_point)}')
    print('q  退出')
    print('')


def _send_key(base_url: str, key: str, timeout_sec: float, inspection_point: str = 'P3') -> bool:
    if key not in DEMO_MENU:
        print(f'无效输入: {key}. 请输入 1-19、p1/p2/p3、空格 或 q')
        return False
    try:
        message = _post_demo_event(base_url, key, timeout_sec, inspection_point)
    except HTTPError as exc:
        try:
            detail = exc.read().decode('utf-8', errors='replace')
        except Exception:  # noqa: BLE001
            detail = str(exc)
        print(f'发送失败 HTTP {exc.code}: {detail}')
        return False
    except (URLError, TimeoutError, OSError) as exc:
        print(f'无法连接 robot_web: {exc}')
        return False
    print(f'已发送 {key}: {message}')
    return True


def _parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Trigger robot_web competition demo logs from terminal keys.',
    )
    parser.add_argument('--base-url', default='http://127.0.0.1:8000')
    parser.add_argument('--timeout-sec', type=_positive_float, default=2.0)
    parser.add_argument(
        '--point',
        type=_normalize_point,
        default='P3',
        help='Inspection point used by P-point demo logs: P1, P2, or P3.',
    )
    parser.add_argument(
        '--once',
        choices=[key for key in sorted(DEMO_MENU, key=_menu_sort_key) if key != ' '] + ['space'],
        help='Trigger one demo event and exit.',
    )
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    inspection_point = args.point
    if args.once:
        once_key = ' ' if args.once == 'space' else args.once
        return 0 if _send_key(args.base_url, once_key, args.timeout_sec, inspection_point) else 1

    _print_menu(args.base_url, inspection_point)
    while True:
        try:
            raw = input('输入数字或 p1/p2/p3> ')
        except (EOFError, KeyboardInterrupt):
            print('')
            return 0
        key = ' ' if raw == ' ' else raw.strip().lower()
        if key in {'q', 'quit', 'exit'}:
            return 0
        if not key:
            continue
        if key in {'p1', 'p2', 'p3'}:
            inspection_point = key.upper()
            print(f'当前巡检点已切换为 {inspection_point}')
            _print_menu(args.base_url, inspection_point)
            continue
        _send_key(args.base_url, key, args.timeout_sec, inspection_point)


if __name__ == '__main__':
    raise SystemExit(main())
