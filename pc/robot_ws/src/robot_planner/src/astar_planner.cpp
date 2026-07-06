#include "robot_planner/astar_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace robot_planner
{

void AStarPlanner::configure(const AStarParams & params)
{
  params_ = params;
}

void AStarPlanner::setMap(const nav_msgs::msg::OccupancyGrid & map)
{
  width_ = static_cast<int>(map.info.width);
  height_ = static_cast<int>(map.info.height);
  resolution_ = map.info.resolution;
  origin_x_ = map.info.origin.position.x;
  origin_y_ = map.info.origin.position.y;
  frame_id_ = map.header.frame_id.empty() ? "map" : map.header.frame_id;
  buildCostmap(map);
  has_map_ = width_ > 0 && height_ > 0 && !costmap_.empty();
}

bool AStarPlanner::hasMap() const
{
  return has_map_;
}

bool AStarPlanner::inBounds(int gx, int gy) const
{
  return gx >= 0 && gy >= 0 && gx < width_ && gy < height_;
}

int AStarPlanner::index(int gx, int gy) const
{
  return gy * width_ + gx;
}

bool AStarPlanner::worldToGrid(double wx, double wy, int & gx, int & gy) const
{
  if (!has_map_ || resolution_ <= 0.0) {
    return false;
  }
  gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  return inBounds(gx, gy);
}

Point2D AStarPlanner::gridToWorld(int gx, int gy) const
{
  return Point2D{
    origin_x_ + (static_cast<double>(gx) + 0.5) * resolution_,
    origin_y_ + (static_cast<double>(gy) + 0.5) * resolution_};
}

bool AStarPlanner::isFreeCell(int gx, int gy) const
{
  return inBounds(gx, gy) && costmap_[index(gx, gy)] < 254;
}

bool AStarPlanner::isFreeWorld(double wx, double wy) const
{
  int gx = 0;
  int gy = 0;
  return worldToGrid(wx, wy, gx, gy) && isFreeCell(gx, gy);
}

void AStarPlanner::buildCostmap(const nav_msgs::msg::OccupancyGrid & map)
{
  const int size = width_ * height_;
  costmap_.assign(size, 0);
  std::vector<int> lethal_indices;
  lethal_indices.reserve(size / 16);

  for (int i = 0; i < size; ++i) {
    const int8_t raw = map.data[static_cast<std::size_t>(i)];
    const bool occupied = raw >= params_.occupied_threshold;
    const bool unknown = raw < 0;
    if (occupied || (unknown && params_.unknown_as_obstacle)) {
      costmap_[i] = 254;
      lethal_indices.push_back(i);
    }
  }

  const double inflate_radius = std::max(params_.robot_radius, params_.inflation_radius);
  const int inflate_cells = static_cast<int>(std::ceil(inflate_radius / resolution_));
  if (inflate_cells <= 0) {
    return;
  }

  for (const int lethal : lethal_indices) {
    const int cx = lethal % width_;
    const int cy = lethal / width_;
    for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
      for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {
        const int nx = cx + dx;
        const int ny = cy + dy;
        if (!inBounds(nx, ny)) {
          continue;
        }
        const double dist = std::hypot(dx * resolution_, dy * resolution_);
        if (dist > inflate_radius) {
          continue;
        }
        const int nidx = index(nx, ny);
        if (dist <= params_.robot_radius) {
          costmap_[nidx] = 254;
        } else if (costmap_[nidx] < 254) {
          const double ratio = 1.0 - ((dist - params_.robot_radius) /
            std::max(1e-6, inflate_radius - params_.robot_radius));
          const uint8_t inflated = static_cast<uint8_t>(std::clamp(ratio, 0.0, 1.0) * 220.0);
          costmap_[nidx] = std::max(costmap_[nidx], inflated);
        }
      }
    }
  }
}

bool AStarPlanner::snapToFree(GridIndex & cell, double radius) const
{
  if (isFreeCell(cell.x, cell.y)) {
    return true;
  }

  const int radius_cells = static_cast<int>(std::ceil(radius / resolution_));
  GridIndex best = cell;
  double best_dist = std::numeric_limits<double>::infinity();
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int nx = cell.x + dx;
      const int ny = cell.y + dy;
      if (!isFreeCell(nx, ny)) {
        continue;
      }
      const double dist = std::hypot(dx * resolution_, dy * resolution_);
      if (dist <= radius && dist < best_dist) {
        best = GridIndex{nx, ny};
        best_dist = dist;
      }
    }
  }

  if (!std::isfinite(best_dist)) {
    return false;
  }
  cell = best;
  return true;
}

AStarPlanResult AStarPlanner::plan(const Point2D & start, const Point2D & goal) const
{
  AStarPlanResult result;
  if (!has_map_) {
    result.state = "NO_MAP";
    return result;
  }

  GridIndex start_cell;
  GridIndex goal_cell;
  if (!worldToGrid(start.x, start.y, start_cell.x, start_cell.y) ||
    !worldToGrid(goal.x, goal.y, goal_cell.x, goal_cell.y))
  {
    result.state = "OUT_OF_MAP";
    return result;
  }

  if (!snapToFree(start_cell, params_.snap_start_to_free_radius)) {
    result.state = "START_OCCUPIED";
    return result;
  }
  if (!snapToFree(goal_cell, params_.snap_goal_to_free_radius)) {
    result.state = "GOAL_OCCUPIED";
    return result;
  }

  struct QueueNode
  {
    int idx;
    double f;
    bool operator>(const QueueNode & other) const {return f > other.f;}
  };

  const int size = width_ * height_;
  const int start_idx = index(start_cell.x, start_cell.y);
  const int goal_idx = index(goal_cell.x, goal_cell.y);
  std::vector<double> g_score(size, std::numeric_limits<double>::infinity());
  std::vector<int> parent(size, -1);
  std::vector<uint8_t> closed(size, 0);
  std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> open;

  auto heuristic = [&](int idx) {
      const int x = idx % width_;
      const int y = idx / width_;
      return std::hypot(x - goal_cell.x, y - goal_cell.y);
    };

  g_score[start_idx] = 0.0;
  open.push(QueueNode{start_idx, heuristic(start_idx)});
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(params_.planning_timeout);

  const std::vector<std::pair<int, int>> neighbors4 = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  const std::vector<std::pair<int, int>> neighbors8 = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  const auto & neighbors = params_.allow_diagonal ? neighbors8 : neighbors4;

  while (!open.empty()) {
    if (std::chrono::steady_clock::now() > deadline) {
      result.state = "TIMEOUT";
      return result;
    }
    if (result.expanded_nodes++ > params_.max_iterations) {
      result.state = "MAX_ITERATIONS";
      return result;
    }

    const QueueNode current = open.top();
    open.pop();
    if (closed[current.idx]) {
      continue;
    }
    closed[current.idx] = 1;
    if (current.idx == goal_idx) {
      break;
    }

    const int cx = current.idx % width_;
    const int cy = current.idx / width_;
    for (const auto & [dx, dy] : neighbors) {
      const int nx = cx + dx;
      const int ny = cy + dy;
      if (!isFreeCell(nx, ny)) {
        continue;
      }
      const int nidx = index(nx, ny);
      if (closed[nidx]) {
        continue;
      }
      const double step = std::hypot(dx, dy);
      const double clearance_cost = (static_cast<double>(costmap_[nidx]) / 253.0) *
        params_.clearance_cost_weight;
      const double tentative = g_score[current.idx] + step * (1.0 + clearance_cost);
      if (tentative < g_score[nidx]) {
        g_score[nidx] = tentative;
        parent[nidx] = current.idx;
        open.push(QueueNode{nidx, tentative + heuristic(nidx)});
      }
    }
  }

  if (parent[goal_idx] < 0 && goal_idx != start_idx) {
    result.state = "NO_PATH";
    return result;
  }

  std::vector<Point2D> path;
  for (int idx = goal_idx; idx >= 0; idx = parent[idx]) {
    path.push_back(gridToWorld(idx % width_, idx / width_));
    if (idx == start_idx) {
      break;
    }
  }
  std::reverse(path.begin(), path.end());
  if (params_.simplify_path) {
    path = simplifyPath(path);
  }
  path = densifyPath(path);

  result.path = path;
  result.state = "PLANNED";
  result.success = !path.empty();
  return result;
}

bool AStarPlanner::hasLineOfSight(const Point2D & a, const Point2D & b) const
{
  const double dist = distance2D(a.x, a.y, b.x, b.y);
  const int steps = std::max(1, static_cast<int>(std::ceil(dist / (resolution_ * 0.5))));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const double x = a.x + (b.x - a.x) * t;
    const double y = a.y + (b.y - a.y) * t;
    if (!isFreeWorld(x, y)) {
      return false;
    }
  }
  return true;
}

std::vector<Point2D> AStarPlanner::simplifyPath(const std::vector<Point2D> & path) const
{
  if (path.size() <= 2) {
    return path;
  }
  std::vector<Point2D> simplified;
  std::size_t anchor = 0;
  simplified.push_back(path.front());
  while (anchor < path.size() - 1) {
    std::size_t next = path.size() - 1;
    while (next > anchor + 1 && !hasLineOfSight(path[anchor], path[next])) {
      --next;
    }
    simplified.push_back(path[next]);
    anchor = next;
  }
  return simplified;
}

std::vector<Point2D> AStarPlanner::densifyPath(const std::vector<Point2D> & path) const
{
  if (path.size() <= 1 || params_.densify_spacing <= 0.0) {
    return path;
  }
  std::vector<Point2D> dense;
  dense.push_back(path.front());
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Point2D & a = path[i - 1];
    const Point2D & b = path[i];
    const double dist = distance2D(a.x, a.y, b.x, b.y);
    const int segments = std::max(1, static_cast<int>(std::ceil(dist / params_.densify_spacing)));
    for (int s = 1; s <= segments; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(segments);
      dense.push_back(Point2D{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t});
    }
  }
  return dense;
}

}  // namespace robot_planner
