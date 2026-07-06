#include "robot_mission/robot_health_monitor_node.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/exceptions.h"

namespace robot_mission
{

namespace
{

using namespace std::chrono_literals;

float ageSeconds(
  const rclcpp::Time & now,
  const bool has_value,
  const rclcpp::Time & stamp)
{
  if (!has_value) {
    return std::numeric_limits<float>::infinity();
  }
  return static_cast<float>((now - stamp).seconds());
}

void addReason(std::vector<std::string> * reasons, const bool ok, const std::string & reason)
{
  if (!ok) {
    reasons->push_back(reason);
  }
}

}  // namespace

RobotHealthMonitorNode::RobotHealthMonitorNode(const rclcpp::NodeOptions & options)
: Node("robot_health_monitor", options)
{
  robot_id_ = declare_parameter<std::string>("robot_id", get_namespace());
  robot_namespace_ = declare_parameter<std::string>("robot_namespace", get_namespace());
  map_version_ = declare_parameter<std::string>("map_version", "");
  map_bundle_hash_ = declare_parameter<std::string>("map_bundle_hash", "");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", robot_id_ + "/odom");
  base_frame_ = declare_parameter<std::string>("base_frame", robot_id_ + "/base_footprint");
  const auto scan_topic = declare_parameter<std::string>("scan_topic", "scan");
  const auto odom_topic = declare_parameter<std::string>("odom_topic", "odom");
  const auto amcl_state_service =
    declare_parameter<std::string>("amcl_state_service", "amcl/get_state");
  const auto controller_state_service =
    declare_parameter<std::string>("controller_state_service", "controller_server/get_state");
  scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 1.0);
  odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 1.0);
  tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 2.0);
  mission_ready_ = declare_parameter<bool>("mission_ready", true);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, rclcpp::SensorDataQoS(),
    std::bind(&RobotHealthMonitorNode::onScan, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS(),
    std::bind(&RobotHealthMonitorNode::onOdom, this, std::placeholders::_1));
  health_pub_ = create_publisher<robot_interfaces::msg::RobotHealth>(
    "health", rclcpp::QoS(1).best_effort());

  lifecycle_clients_["amcl"] = create_client<GetLifecycleState>(amcl_state_service);
  lifecycle_clients_["controller"] = create_client<GetLifecycleState>(controller_state_service);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  timer_ = create_wall_timer(500ms, std::bind(&RobotHealthMonitorNode::publishHealth, this));

  RCLCPP_INFO(
    get_logger(), "health monitor ready for %s map_version=%s",
    robot_id_.c_str(), map_version_.c_str());
}

void RobotHealthMonitorNode::onScan(const sensor_msgs::msg::LaserScan::SharedPtr)
{
  has_scan_ = true;
  last_scan_ = now();
}

void RobotHealthMonitorNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr)
{
  has_odom_ = true;
  last_odom_ = now();
}

void RobotHealthMonitorNode::pollLifecycleState(
  const std::string & key,
  const rclcpp::Client<GetLifecycleState>::SharedPtr & client)
{
  if (!client || !client->service_is_ready()) {
    if (key == "amcl") {
      amcl_active_ = false;
    } else if (key == "controller") {
      nav2_active_ = false;
    }
    return;
  }

  auto request = std::make_shared<GetLifecycleState::Request>();
  client->async_send_request(
    request,
    [this, key](rclcpp::Client<GetLifecycleState>::SharedFuture future) {
      const bool active =
        future.get()->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
      if (key == "amcl") {
        amcl_active_ = active;
      } else if (key == "controller") {
        nav2_active_ = active;
      }
    });
}

void RobotHealthMonitorNode::publishHealth()
{
  for (const auto & [key, client] : lifecycle_clients_) {
    pollLifecycleState(key, client);
  }

  const auto current_time = now();
  robot_interfaces::msg::RobotHealth msg;
  msg.header.stamp = current_time;
  msg.header.frame_id = map_frame_;
  msg.robot_id = robot_id_;
  msg.robot_namespace = robot_namespace_;
  msg.map_version = map_version_;
  msg.map_bundle_hash = map_bundle_hash_;
  msg.map_pose.header.stamp = current_time;
  msg.map_pose.header.frame_id = map_frame_;
  msg.map_pose.pose.orientation.w = 1.0;
  msg.scan_age_sec = ageSeconds(current_time, has_scan_, last_scan_);
  msg.odom_age_sec = ageSeconds(current_time, has_odom_, last_odom_);
  msg.tf_age_sec = std::numeric_limits<float>::infinity();

  bool map_to_odom_ok = false;
  try {
    const auto base_transform = tf_buffer_->lookupTransform(
      map_frame_, base_frame_, tf2::TimePointZero);
    msg.map_pose.header.stamp = base_transform.header.stamp;
    msg.map_pose.pose.position.x = base_transform.transform.translation.x;
    msg.map_pose.pose.position.y = base_transform.transform.translation.y;
    msg.map_pose.pose.position.z = base_transform.transform.translation.z;
    msg.map_pose.pose.orientation = base_transform.transform.rotation;

    const auto odom_transform = tf_buffer_->lookupTransform(
      map_frame_, odom_frame_, tf2::TimePointZero);
    msg.tf_age_sec = static_cast<float>(
      (current_time - rclcpp::Time(odom_transform.header.stamp)).seconds());
    map_to_odom_ok = msg.tf_age_sec <= tf_timeout_sec_;
  } catch (const tf2::TransformException &) {
    map_to_odom_ok = false;
  }

  msg.scan_ok = has_scan_ && msg.scan_age_sec <= scan_timeout_sec_;
  msg.odom_ok = has_odom_ && msg.odom_age_sec <= odom_timeout_sec_;
  msg.amcl_active = amcl_active_;
  msg.nav2_active = nav2_active_;
  msg.map_to_odom_ok = map_to_odom_ok;
  msg.mission_ready = mission_ready_;

  addReason(&msg.reasons, msg.scan_ok, "scan stale");
  addReason(&msg.reasons, msg.odom_ok, "odom stale");
  addReason(&msg.reasons, msg.amcl_active, "amcl inactive");
  addReason(&msg.reasons, msg.nav2_active, "nav2 inactive");
  addReason(&msg.reasons, msg.map_to_odom_ok, "map_to_odom unavailable");
  addReason(&msg.reasons, msg.mission_ready, "mission not ready");

  msg.health_state = msg.reasons.empty()
    ? robot_interfaces::msg::RobotHealth::OK
    : robot_interfaces::msg::RobotHealth::ERROR;
  health_pub_->publish(msg);
}

}  // namespace robot_mission
