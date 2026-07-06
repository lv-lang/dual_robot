#!/usr/bin/env python3

import heapq
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


GridCell = Tuple[int, int]
_EIGHT_CONNECTED_OFFSETS: Tuple[GridCell, ...] = (
    (-1, -1), (0, -1), (1, -1),
    (-1, 0), (1, 0),
    (-1, 1), (0, 1), (1, 1),
)


@dataclass(frozen=True)
class PlannerParams:
    allow_diagonal: bool
    occupied_threshold: int
    unknown_as_obstacle: bool
    robot_radius: float
    inflation_radius: float
    snap_start_to_free_radius: float
    snap_goal_to_free_radius: float
    planning_timeout: float
    publish_empty_path_on_failure: bool
    simple_test_map: bool
    simple_test_map_width: int
    simple_test_map_height: int
    simple_test_map_resolution: float
    simple_test_map_frame_id: str
    simple_test_map_origin_x: float
    simple_test_map_origin_y: float
    simple_test_map_add_obstacles: bool
    map_aligned_odom_frames: Sequence[str]


@dataclass
class GridMap:
    msg: OccupancyGrid
    blocked: Sequence[bool]
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float


class AStarGlobalPlanner(Node):
    """robot1 OccupancyGrid A* planner.

    This node intentionally publishes only global path and planner state.
    Velocity control, local obstacle avoidance, and safety supervision belong
    to downstream nodes.
    """

    def __init__(self) -> None:
        super().__init__("astar_global_planner")

        self._params = self._load_params()
        self._grid: Optional[GridMap] = None
        self._odom: Optional[Odometry] = None
        self._goal: Optional[PoseStamped] = None
        self._pending_goal = False
        self._last_logged_failure = ""

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(OccupancyGrid, "/robot1/map", self._on_map, map_qos)
        self.create_subscription(Odometry, "/robot1/odom", self._on_odom, 10)
        self.create_subscription(PoseStamped, "/robot1/goal_pose", self._on_goal, 10)

        output_qos = QoSProfile(depth=1)
        output_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        output_qos.reliability = ReliabilityPolicy.RELIABLE

        self._path_pub = self.create_publisher(
            Path, "/robot1/global_path", output_qos)
        self._state_pub = self.create_publisher(
            String, "/robot1/planner_state", output_qos)

        if self._params.simple_test_map:
            self._grid = self._build_grid_map(self._create_simple_test_map())
            self._publish_state("simple_test_map_ready")
        else:
            self._publish_state("waiting_for_map")

    def _load_params(self) -> PlannerParams:
        self.declare_parameter("allow_diagonal", True)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("unknown_as_obstacle", True)
        self.declare_parameter("robot_radius", 0.16)
        self.declare_parameter("inflation_radius", 0.20)
        self.declare_parameter("snap_start_to_free_radius", 0.30)
        self.declare_parameter("snap_goal_to_free_radius", 0.80)
        self.declare_parameter("planning_timeout", 1.0)
        self.declare_parameter("publish_empty_path_on_failure", True)
        self.declare_parameter("simple_test_map", False)
        self.declare_parameter("simple_test_map_width", 80)
        self.declare_parameter("simple_test_map_height", 60)
        self.declare_parameter("simple_test_map_resolution", 0.10)
        self.declare_parameter("simple_test_map_frame_id", "map")
        self.declare_parameter("simple_test_map_origin_x", -4.0)
        self.declare_parameter("simple_test_map_origin_y", -3.0)
        self.declare_parameter("simple_test_map_add_obstacles", True)
        self.declare_parameter("map_aligned_odom_frames", ["robot1/odom"])

        return PlannerParams(
            allow_diagonal=self.get_parameter("allow_diagonal").value,
            occupied_threshold=int(self.get_parameter("occupied_threshold").value),
            unknown_as_obstacle=self.get_parameter("unknown_as_obstacle").value,
            robot_radius=float(self.get_parameter("robot_radius").value),
            inflation_radius=float(self.get_parameter("inflation_radius").value),
            snap_start_to_free_radius=float(
                self.get_parameter("snap_start_to_free_radius").value
            ),
            snap_goal_to_free_radius=float(
                self.get_parameter("snap_goal_to_free_radius").value
            ),
            planning_timeout=float(self.get_parameter("planning_timeout").value),
            publish_empty_path_on_failure=self.get_parameter(
                "publish_empty_path_on_failure"
            ).value,
            simple_test_map=self.get_parameter("simple_test_map").value,
            simple_test_map_width=int(self.get_parameter("simple_test_map_width").value),
            simple_test_map_height=int(self.get_parameter("simple_test_map_height").value),
            simple_test_map_resolution=float(
                self.get_parameter("simple_test_map_resolution").value
            ),
            simple_test_map_frame_id=self.get_parameter("simple_test_map_frame_id").value,
            simple_test_map_origin_x=float(
                self.get_parameter("simple_test_map_origin_x").value
            ),
            simple_test_map_origin_y=float(
                self.get_parameter("simple_test_map_origin_y").value
            ),
            simple_test_map_add_obstacles=self.get_parameter(
                "simple_test_map_add_obstacles"
            ).value,
            map_aligned_odom_frames=tuple(
                _clean_frame_id(str(frame_id))
                for frame_id in self.get_parameter("map_aligned_odom_frames").value
            ),
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        map_error = self._map_error(msg)
        if map_error:
            self._grid = None
            self._fail_plan(map_error)
            return

        had_grid = self._grid is not None
        self._grid = self._build_grid_map(msg)
        if not had_grid:
            self._publish_state("map_ready")
        if self._pending_goal:
            self._try_plan("map_update")

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg
        if self._pending_goal and self._grid is not None:
            self._try_plan("odom_update")

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goal = msg
        self._pending_goal = True
        self._try_plan("goal_update")

    def _try_plan(self, reason: str) -> None:
        if self._grid is None:
            if self._params.simple_test_map:
                self._grid = self._build_grid_map(self._create_simple_test_map())
            else:
                self._publish_state("waiting_for_map")
                return

        if self._odom is None:
            self._publish_state("waiting_for_odom")
            return

        if self._goal is None:
            self._publish_state("waiting_for_goal")
            return

        frame_error = self._frame_error(self._grid, self._odom, self._goal)
        if frame_error:
            self._fail_plan(frame_error)
            return

        start = self._world_to_grid(
            self._odom.pose.pose.position.x,
            self._odom.pose.pose.position.y,
            self._grid,
        )
        goal = self._world_to_grid(
            self._goal.pose.position.x,
            self._goal.pose.position.y,
            self._grid,
        )

        if start is None:
            self._fail_plan("start_outside_map")
            return
        if goal is None:
            self._fail_plan("goal_outside_map")
            return
        if not self._is_free(start, self._grid):
            snapped_start = self._nearest_free_cell(
                start, self._grid, self._params.snap_start_to_free_radius)
            if snapped_start is None:
                self._fail_plan("start_occupied")
                return
            start = snapped_start
        if not self._is_free(goal, self._grid):
            snapped_goal = self._nearest_free_cell(
                goal, self._grid, self._params.snap_goal_to_free_radius)
            if snapped_goal is None:
                self._fail_plan("goal_occupied")
                return
            goal = snapped_goal

        self._publish_state(f"planning:{reason}")
        path_cells, status = self._astar(start, goal, self._grid)
        if path_cells is None:
            self._fail_plan(status)
            return

        self._publish_path(path_cells, self._grid, self._goal)
        self._pending_goal = False
        self._last_logged_failure = ""
        self._publish_state(f"path_found:{len(path_cells)}")

    def _fail_plan(self, status: str) -> None:
        if self._params.publish_empty_path_on_failure:
            self._path_pub.publish(self._empty_path())
        self._publish_state(status)
        if status != self._last_logged_failure:
            self.get_logger().error(f"A* planning blocked: {status}")
            self._last_logged_failure = status

    def _map_error(self, msg: OccupancyGrid) -> str:
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        expected_cells = width * height
        actual_cells = len(msg.data)

        if width <= 0 or height <= 0:
            return f"invalid_map_size:width={width},height={height}"
        if resolution <= 0.0 or not math.isfinite(resolution):
            return f"invalid_map_resolution:{resolution}"
        if actual_cells != expected_cells:
            return (
                "invalid_map_data_length:"
                f"expected={expected_cells},actual={actual_cells}"
            )
        return ""

    def _frame_error(
        self, grid: GridMap, odom: Odometry, goal: PoseStamped
    ) -> str:
        map_frame = _clean_frame_id(grid.msg.header.frame_id)
        odom_frame = _clean_frame_id(odom.header.frame_id)
        goal_frame = _clean_frame_id(goal.header.frame_id)

        if not map_frame:
            return "missing_map_frame"
        if not odom_frame:
            return "missing_odom_frame"
        if not goal_frame:
            return "missing_goal_frame"
        if goal_frame != map_frame:
            return (
                "frame_mismatch:"
                f"map={map_frame},odom={odom_frame},goal={goal_frame}"
            )
        if (
            odom_frame != map_frame
            and odom_frame not in self._params.map_aligned_odom_frames
        ):
            return (
                "frame_mismatch:"
                f"map={map_frame},odom={odom_frame},goal={goal_frame}"
            )
        return ""

    def _build_grid_map(self, msg: OccupancyGrid) -> GridMap:
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        origin_x = float(msg.info.origin.position.x)
        origin_y = float(msg.info.origin.position.y)
        origin_yaw = _yaw_from_quaternion(msg.info.origin.orientation)

        blocked = self._build_blocked_grid(msg.data, width, height, resolution)
        return GridMap(
            msg=msg,
            blocked=blocked,
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            origin_yaw=origin_yaw,
        )

    def _build_blocked_grid(
        self, data: Sequence[int], width: int, height: int, resolution: float
    ) -> Sequence[bool]:
        blocked = [False] * (width * height)
        occupied_indices: List[int] = []
        for index, value in enumerate(data):
            is_blocked = value >= self._params.occupied_threshold
            if value < 0 and self._params.unknown_as_obstacle:
                is_blocked = True
            if is_blocked:
                blocked[index] = True
                occupied_indices.append(index)

        inflation_cells = int(
            math.ceil(
                max(0.0, self._params.robot_radius + self._params.inflation_radius)
                / resolution
            )
        )
        if inflation_cells <= 0:
            return blocked

        inflated = blocked[:]
        queue = deque((index, 0) for index in occupied_indices)
        while queue:
            index, distance = queue.popleft()
            if distance >= inflation_cells:
                continue

            x = index % width
            y = index // width
            next_distance = distance + 1
            for offset_x, offset_y in _EIGHT_CONNECTED_OFFSETS:
                next_x = x + offset_x
                next_y = y + offset_y
                if not (0 <= next_x < width and 0 <= next_y < height):
                    continue
                next_index = next_y * width + next_x
                if inflated[next_index]:
                    continue
                inflated[next_index] = True
                queue.append((next_index, next_distance))
        return inflated

    def _astar(
        self, start: GridCell, goal: GridCell, grid: GridMap
    ) -> Tuple[Optional[List[GridCell]], str]:
        deadline = time.monotonic() + max(0.0, self._params.planning_timeout)
        frontier: List[Tuple[float, int, GridCell]] = []
        heapq.heappush(frontier, (0.0, 0, start))
        came_from: Dict[GridCell, Optional[GridCell]] = {start: None}
        cost_so_far: Dict[GridCell, float] = {start: 0.0}
        sequence = 0

        while frontier:
            if time.monotonic() > deadline:
                return None, "planning_timeout"

            _, _, current = heapq.heappop(frontier)
            if current == goal:
                return _reconstruct_path(came_from, current), "path_found"

            for neighbor, step_cost in self._neighbors(current, grid):
                new_cost = cost_so_far[current] + step_cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    sequence += 1
                    priority = new_cost + self._heuristic(neighbor, goal)
                    heapq.heappush(frontier, (priority, sequence, neighbor))
                    came_from[neighbor] = current

        return None, "no_path"

    def _neighbors(self, cell: GridCell, grid: GridMap) -> Iterable[Tuple[GridCell, float]]:
        x, y = cell
        cardinal = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0))
        for dx, dy, cost in cardinal:
            neighbor = (x + dx, y + dy)
            if self._is_free(neighbor, grid):
                yield neighbor, cost

        if not self._params.allow_diagonal:
            return

        diagonal_cost = math.sqrt(2.0)
        for dx, dy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
            neighbor = (x + dx, y + dy)
            side_a = (x + dx, y)
            side_b = (x, y + dy)
            if (
                self._is_free(neighbor, grid)
                and self._is_free(side_a, grid)
                and self._is_free(side_b, grid)
            ):
                yield neighbor, diagonal_cost

    def _heuristic(self, cell: GridCell, goal: GridCell) -> float:
        dx = abs(cell[0] - goal[0])
        dy = abs(cell[1] - goal[1])
        if self._params.allow_diagonal:
            return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)
        return float(dx + dy)

    def _is_free(self, cell: GridCell, grid: GridMap) -> bool:
        x, y = cell
        if not (0 <= x < grid.width and 0 <= y < grid.height):
            return False
        return not grid.blocked[y * grid.width + x]

    def _nearest_free_cell(
        self, cell: GridCell, grid: GridMap, radius_m: float
    ) -> Optional[GridCell]:
        radius_cells = int(math.ceil(max(0.0, radius_m) / grid.resolution))
        best_cell: Optional[GridCell] = None
        best_distance_sq: Optional[int] = None
        cell_x, cell_y = cell

        for offset_y in range(-radius_cells, radius_cells + 1):
            for offset_x in range(-radius_cells, radius_cells + 1):
                distance_sq = offset_x * offset_x + offset_y * offset_y
                if distance_sq > radius_cells * radius_cells:
                    continue
                candidate = (cell_x + offset_x, cell_y + offset_y)
                if not self._is_free(candidate, grid):
                    continue
                if best_distance_sq is None or distance_sq < best_distance_sq:
                    best_cell = candidate
                    best_distance_sq = distance_sq

        return best_cell

    def _world_to_grid(
        self, world_x: float, world_y: float, grid: GridMap
    ) -> Optional[GridCell]:
        dx = world_x - grid.origin_x
        dy = world_y - grid.origin_y
        cos_yaw = math.cos(-grid.origin_yaw)
        sin_yaw = math.sin(-grid.origin_yaw)
        local_x = cos_yaw * dx - sin_yaw * dy
        local_y = sin_yaw * dx + cos_yaw * dy
        cell_x = int(math.floor(local_x / grid.resolution))
        cell_y = int(math.floor(local_y / grid.resolution))
        if not (0 <= cell_x < grid.width and 0 <= cell_y < grid.height):
            return None
        return cell_x, cell_y

    def _grid_to_world(self, cell: GridCell, grid: GridMap) -> Tuple[float, float]:
        local_x = (cell[0] + 0.5) * grid.resolution
        local_y = (cell[1] + 0.5) * grid.resolution
        cos_yaw = math.cos(grid.origin_yaw)
        sin_yaw = math.sin(grid.origin_yaw)
        world_x = grid.origin_x + cos_yaw * local_x - sin_yaw * local_y
        world_y = grid.origin_y + sin_yaw * local_x + cos_yaw * local_y
        return world_x, world_y

    def _publish_path(
        self, path_cells: Sequence[GridCell], grid: GridMap, goal: PoseStamped
    ) -> None:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = grid.msg.header.frame_id or "map"

        for index, cell in enumerate(path_cells):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = self._grid_to_world(cell, grid)
            pose.pose.position.z = 0.0

            if index == len(path_cells) - 1:
                pose.pose.orientation = goal.pose.orientation
            else:
                next_x, next_y = self._grid_to_world(path_cells[index + 1], grid)
                yaw = math.atan2(next_y - pose.pose.position.y, next_x - pose.pose.position.x)
                pose.pose.orientation = _quaternion_from_yaw(yaw)
            path.poses.append(pose)

        self._path_pub.publish(path)

    def _empty_path(self) -> Path:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        if self._grid is not None:
            path.header.frame_id = self._grid.msg.header.frame_id or "map"
        else:
            path.header.frame_id = "map"
        return path

    def _publish_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self._state_pub.publish(msg)

    def _create_simple_test_map(self) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._params.simple_test_map_frame_id
        msg.info.width = self._params.simple_test_map_width
        msg.info.height = self._params.simple_test_map_height
        msg.info.resolution = self._params.simple_test_map_resolution
        msg.info.origin.position.x = self._params.simple_test_map_origin_x
        msg.info.origin.position.y = self._params.simple_test_map_origin_y
        msg.info.origin.orientation.w = 1.0

        width = msg.info.width
        height = msg.info.height
        data = [0] * (width * height)
        if self._params.simple_test_map_add_obstacles:
            wall_x = width // 2
            gap_min_y = (height // 2) - 4
            gap_max_y = (height // 2) + 4
            for y in range(2, height - 2):
                if gap_min_y <= y <= gap_max_y:
                    continue
                data[y * width + wall_x] = 100

        msg.data = data
        return msg


def _yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _clean_frame_id(frame_id: str) -> str:
    return frame_id.strip().lstrip("/")


def _reconstruct_path(
    came_from: Dict[GridCell, Optional[GridCell]], current: GridCell
) -> List[GridCell]:
    path = [current]
    while came_from[current] is not None:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = AStarGlobalPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
