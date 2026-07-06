#ifndef ROBOT_PLANNER__ASTAR_PLANNER_HPP_
#define ROBOT_PLANNER__ASTAR_PLANNER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "robot_planner/planner_types.hpp"

namespace robot_planner
{

struct AStarParams
{
  bool allow_diagonal{true};
  int occupied_threshold{65};
  bool unknown_as_obstacle{false};
  double robot_radius{0.16};
  double inflation_radius{0.20};
  double snap_start_to_free_radius{0.30};
  double snap_goal_to_free_radius{0.80};
  double planning_timeout{1.0};
  int max_iterations{200000};
  double clearance_cost_weight{1.0};
  double densify_spacing{0.08};
  bool simplify_path{true};
};

struct AStarPlanResult
{
  std::vector<Point2D> path;
  std::string state{"WAITING"};
  int expanded_nodes{0};
  bool success{false};
};

class AStarPlanner
{
public:
  void configure(const AStarParams & params);
  void setMap(const nav_msgs::msg::OccupancyGrid & map);
  bool hasMap() const;
  AStarPlanResult plan(const Point2D & start, const Point2D & goal) const;

  bool worldToGrid(double wx, double wy, int & gx, int & gy) const;
  Point2D gridToWorld(int gx, int gy) const;
  bool isFreeWorld(double wx, double wy) const;

private:
  struct GridIndex
  {
    int x{0};
    int y{0};
  };

  bool inBounds(int gx, int gy) const;
  int index(int gx, int gy) const;
  bool isFreeCell(int gx, int gy) const;
  bool snapToFree(GridIndex & cell, double radius) const;
  bool hasLineOfSight(const Point2D & a, const Point2D & b) const;
  std::vector<Point2D> simplifyPath(const std::vector<Point2D> & path) const;
  std::vector<Point2D> densifyPath(const std::vector<Point2D> & path) const;
  void buildCostmap(const nav_msgs::msg::OccupancyGrid & map);

  AStarParams params_;
  bool has_map_{false};
  int width_{0};
  int height_{0};
  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::string frame_id_{"map"};
  std::vector<uint8_t> costmap_;
};

}  // namespace robot_planner

#endif  // ROBOT_PLANNER__ASTAR_PLANNER_HPP_
