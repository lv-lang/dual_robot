import argparse
import sys
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from robot_tools.nav2_goal_utils import (
    ROBOT_NAMES,
    forbidden_topics,
    required_robot_topics,
    topic_names,
)


class RobotNamespaceChecker(Node):
    def __init__(self) -> None:
        super().__init__('check_robot_namespaces')

    def wait_for_graph(self, wait_sec: float) -> set:
        end_time = time.monotonic() + wait_sec
        names = topic_names(self)
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            names = topic_names(self)
        return names


def _positive_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError('must be 0 or greater')
    return parsed


def _parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Check robot1/robot2 topic namespace boundaries.')
    parser.add_argument(
        '--robots',
        nargs='+',
        choices=ROBOT_NAMES,
        default=list(ROBOT_NAMES),
        help='Robot namespaces expected to be running.',
    )
    parser.add_argument(
        '--wait-sec',
        type=_positive_float,
        default=2.0,
        help='Seconds to wait for ROS graph discovery.',
    )
    parser.add_argument(
        '--skip-required-topics',
        action='store_true',
        help='Only fail on forbidden topics.',
    )
    parser.add_argument(
        '--no-require-map',
        action='store_false',
        dest='require_map',
        help='Do not require the shared /map topic.',
    )
    parser.set_defaults(require_map=True)
    return parser.parse_args(argv)


def _print_topic_group(label: str, names: List[str]) -> None:
    if names:
        print(f'{label}: {", ".join(names)}')
    else:
        print(f'{label}: none')


def main(argv: Optional[List[str]] = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    node = RobotNamespaceChecker()
    try:
        names = node.wait_for_graph(args.wait_sec)
        forbidden = sorted(names & forbidden_topics(args.robots))
        required = sorted(required_robot_topics(args.robots) - names)
        ok = True

        if forbidden:
            _print_topic_group('ERROR forbidden topics present', forbidden)
            ok = False
        else:
            _print_topic_group(
                'OK forbidden topics absent',
                sorted(forbidden_topics(args.robots)),
            )

        if not args.skip_required_topics:
            if required:
                _print_topic_group('ERROR required topics missing', required)
                ok = False
            else:
                _print_topic_group(
                    'OK required namespaced topics present',
                    sorted(required_robot_topics(args.robots)),
                )

        if args.require_map:
            if '/map' in names:
                print('OK shared /map topic present')
            else:
                print('ERROR shared /map topic missing')
                ok = False

        return 0 if ok else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
