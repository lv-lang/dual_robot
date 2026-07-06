#pragma once

#include <set>
#include <string>

#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "robot_dispatch/dispatch_types.hpp"
#include "robot_dispatch/task_points.hpp"

namespace robot_dispatch
{

constexpr const char * kMarkerTopic = "/robot_dispatch/markers";

struct MarkerSceneState
{
  std::set<std::string> locked_point_ids;
  std::set<std::string> active_target_ids;
  std::set<std::string> abnormal_point_ids;
  std::set<std::string> recheck_target_ids;
};

std_msgs::msg::ColorRGBA colorForPointKind(PointKind kind);
std_msgs::msg::ColorRGBA idlePointColor();
std_msgs::msg::ColorRGBA lockedPointColor();
std_msgs::msg::ColorRGBA activeTargetColor();
std_msgs::msg::ColorRGBA abnormalPointColor();
std_msgs::msg::ColorRGBA recheckTargetColor();

visualization_msgs::msg::MarkerArray buildTaskPointMarkers(
  const TaskPointConfig & config,
  const MarkerSceneState & state = MarkerSceneState{});

}  // namespace robot_dispatch
