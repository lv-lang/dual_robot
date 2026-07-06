#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "robot_interfaces/msg/robot_health.hpp"

namespace robot_mission
{

class RobotHealthMonitorNode : public rclcpp::Node
{
public:
  explicit RobotHealthMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using GetLifecycleState = lifecycle_msgs::srv::GetState;

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pollLifecycleState(
    const std::string & key,
    const rclcpp::Client<GetLifecycleState>::SharedPtr & client);
  void publishHealth();

  std::string robot_id_;
  std::string robot_namespace_;
  std::string map_version_;
  std::string map_bundle_hash_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double scan_timeout_sec_{1.0};
  double odom_timeout_sec_{1.0};
  double tf_timeout_sec_{2.0};
  bool mission_ready_{true};

  bool has_scan_{false};
  bool has_odom_{false};
  rclcpp::Time last_scan_;
  rclcpp::Time last_odom_;
  bool amcl_active_{false};
  bool nav2_active_{false};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<robot_interfaces::msg::RobotHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::string, rclcpp::Client<GetLifecycleState>::SharedPtr> lifecycle_clients_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace robot_mission
