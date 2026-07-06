#include "robot_dispatch/task_points.hpp"

#include <stdexcept>

#include "yaml-cpp/yaml.h"

namespace robot_dispatch
{

namespace
{

std::string normalizeFrameId(std::string frame_id)
{
  while (!frame_id.empty() && frame_id.front() == '/') {
    frame_id.erase(frame_id.begin());
  }
  return frame_id.empty() ? "map" : frame_id;
}

}  // namespace

TaskPointConfig loadTaskPointsFromYaml(const std::string & path)
{
  const auto yaml = YAML::LoadFile(path);
  TaskPointConfig config;
  if (yaml["frame_id"]) {
    config.frame_id = normalizeFrameId(yaml["frame_id"].as<std::string>());
  }
  if (yaml["map_version"]) {
    config.map_version = yaml["map_version"].as<std::string>();
  }

  if (!yaml["points"] || !yaml["points"].IsMap()) {
    throw std::runtime_error("task point config missing points map: " + path);
  }

  for (const auto point_node : yaml["points"]) {
    const auto id = point_node.first.as<std::string>();
    const auto value = point_node.second;
    TaskPoint point;
    point.id = id;
    point.kind = pointKindFromString(value["kind"].as<std::string>());
    point.label = value["label"] ? value["label"].as<std::string>() : id;
    point.pose.x = value["x"].as<double>();
    point.pose.y = value["y"].as<double>();
    point.pose.yaw = value["yaw"] ? value["yaw"].as<double>() : 0.0;
    config.points[id] = point;
  }

  if (yaml["robots"]) {
    for (const auto robot_node : yaml["robots"]) {
      const auto robot_id = robot_node.first.as<std::string>();
      const auto value = robot_node.second;
      RobotPointConfig robot;
      robot.robot_id = robot_id;
      robot.waiting_area_id = value["waiting_area"].as<std::string>();
      robot.preferred_delivery =
        value["preferred_delivery"] &&
        value["preferred_delivery"].as<bool>();
      robot.preferred_inspection =
        value["preferred_inspection"] &&
        value["preferred_inspection"].as<bool>();
      config.robots[robot_id] = robot;
    }
  }

  if (yaml["routes"]) {
    for (const auto route_node : yaml["routes"]) {
      const auto route_id = route_node.first.as<std::string>();
      for (const auto point_id : route_node.second) {
        config.routes[route_id].push_back(point_id.as<std::string>());
      }
    }
  }

  return config;
}

}  // namespace robot_dispatch
