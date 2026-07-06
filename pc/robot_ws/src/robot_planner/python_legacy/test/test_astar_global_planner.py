from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

from global_planner.astar_global_planner import AStarGlobalPlanner, GridMap


def _grid(width, height, blocked):
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = 0.05
    msg.info.origin.orientation.w = 1.0
    return GridMap(
        msg=msg,
        blocked=blocked,
        width=width,
        height=height,
        resolution=0.05,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
    )


def test_nearest_free_cell_snaps_from_blocked_goal_to_local_free_space():
    blocked = [True] * 25
    blocked[2 * 5 + 3] = False
    planner = object.__new__(AStarGlobalPlanner)

    assert planner._nearest_free_cell((2, 2), _grid(5, 5, blocked), 0.08) == (3, 2)


def test_nearest_free_cell_respects_snap_radius():
    blocked = [True] * 25
    blocked[2 * 5 + 4] = False
    planner = object.__new__(AStarGlobalPlanner)

    assert planner._nearest_free_cell((2, 2), _grid(5, 5, blocked), 0.05) is None


def test_frame_check_allows_configured_map_aligned_odom_frame():
    planner = object.__new__(AStarGlobalPlanner)
    planner._params = SimpleNamespace(map_aligned_odom_frames=("robot1/odom",))
    grid = _grid(5, 5, [False] * 25)
    odom = Odometry()
    odom.header.frame_id = "robot1/odom"
    goal = PoseStamped()
    goal.header.frame_id = "map"

    assert planner._frame_error(grid, odom, goal) == ""


def test_frame_check_rejects_goal_frame_that_is_not_map():
    planner = object.__new__(AStarGlobalPlanner)
    planner._params = SimpleNamespace(map_aligned_odom_frames=("robot1/odom",))
    grid = _grid(5, 5, [False] * 25)
    odom = Odometry()
    odom.header.frame_id = "robot1/odom"
    goal = PoseStamped()
    goal.header.frame_id = "robot1/base_link"

    assert planner._frame_error(grid, odom, goal).startswith("frame_mismatch:")
