#pragma once

#include <map>
#include <string>
#include <vector>

#include "robot_dispatch/dispatch_types.hpp"

namespace robot_dispatch
{

struct RobotPointConfig
{
  std::string robot_id;
  std::string waiting_area_id;
  bool preferred_delivery{false};
  bool preferred_inspection{false};
};

struct TaskPointConfig
{
  std::string frame_id{"map"};
  std::string map_version;
  std::map<std::string, TaskPoint> points;
  std::map<std::string, RobotPointConfig> robots;
  std::map<std::string, std::vector<std::string>> routes;
};

TaskPointConfig loadTaskPointsFromYaml(const std::string & path);

}  // namespace robot_dispatch
