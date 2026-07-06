#include "robot_planner/planner_types.hpp"

#include <cmath>

namespace robot_planner
{

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double angleDiff(double from, double to)
{
  return normalizeAngle(to - from);
}

double distance2D(double ax, double ay, double bx, double by)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  return std::hypot(dx, dy);
}

}  // namespace robot_planner
