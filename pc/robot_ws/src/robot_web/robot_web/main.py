import argparse
import sys
from pathlib import Path

from robot_web.config import find_builtin_templates_file
from robot_web.gateway import RobotWebGateway
from robot_web.points import find_default_task_points_file, load_task_points_from_yaml
from robot_web.ros_adapter import RosDispatchClient
from robot_web.storage import RobotWebStore
from robot_web.system_control import SystemControlService


DEPENDENCY_COMMAND = "python3 -m pip install fastapi uvicorn httpx"


def default_db_path():
    return str(Path.home() / ".ros" / "robot_web.sqlite3")


def build_gateway(args):
    db_path = args.db_path or default_db_path()
    task_points_file = find_default_task_points_file(args.task_points_file or None)
    builtin_templates_file = find_builtin_templates_file(args.builtin_templates_file or None)
    fallback_points = load_task_points_from_yaml(task_points_file)
    store = RobotWebStore(db_path)
    dispatch = RosDispatchClient(service_timeout_sec=args.service_timeout_sec)
    return RobotWebGateway.from_paths(
        store=store,
        dispatch_client=dispatch,
        builtin_templates_file=builtin_templates_file,
        fallback_points=fallback_points,
        system_control=SystemControlService.default(),
    )


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Run robot_web App gateway.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--db-path", default="")
    parser.add_argument("--frontend-dist", default="")
    parser.add_argument("--task-points-file", default="")
    parser.add_argument("--builtin-templates-file", default="")
    parser.add_argument("--service-timeout-sec", type=float, default=1.0)
    parser.add_argument("--poll-interval-sec", type=float, default=1.0)
    parser.add_argument("--map-yaml", default="")
    parser.add_argument("--cameras-file", default="")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        import uvicorn
        from robot_web.app import create_app, find_frontend_dist
    except ModuleNotFoundError as exc:
        print(
            f"Missing Python dependency: {exc.name}. Install with: {DEPENDENCY_COMMAND}",
            file=sys.stderr,
        )
        return 2

    gateway = build_gateway(args)
    frontend_dist = find_frontend_dist(args.frontend_dist or None)
    app = create_app(
        gateway,
        frontend_dist=frontend_dist,
        poll_interval_sec=args.poll_interval_sec,
        map_yaml=args.map_yaml or None,
        cameras_file=args.cameras_file or None,
    )
    uvicorn.run(app, host=args.host, port=args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
