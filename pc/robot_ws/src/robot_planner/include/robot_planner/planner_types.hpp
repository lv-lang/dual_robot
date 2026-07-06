#ifndef ROBOT_PLANNER__PLANNER_TYPES_HPP_
#define ROBOT_PLANNER__PLANNER_TYPES_HPP_

#include <string>
#include <vector>

namespace robot_planner
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Velocity2D
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct PlannerResult
{
  Velocity2D cmd;
  std::vector<Pose2D> trajectory;
  std::string state{"WAITING"};
  double score{0.0};
  bool valid{false};
};

double normalizeAngle(double angle);
double angleDiff(double from, double to);
double distance2D(double ax, double ay, double bx, double by);

}  // namespace robot_planner

#endif  // ROBOT_PLANNER__PLANNER_TYPES_HPP_
