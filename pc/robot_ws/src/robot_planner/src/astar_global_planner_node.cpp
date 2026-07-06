#include <memory>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "robot_planner/astar_planner.hpp"

namespace robot_planner
{

namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

class AStarGlobalPlannerNode : public rclcpp::Node
{
public:
  explicit AStarGlobalPlannerNode(const rclcpp::NodeOptions & options)
  : Node("astar_global_planner", options)
  {
    AStarParams params;
    params.allow_diagonal = getParam("allow_diagonal", true);
    params.occupied_threshold = getParam("occupied_threshold", 65);
    params.unknown_as_obstacle = getParam("unknown_as_obstacle", false);
    params.robot_radius = getParam("robot_radius", 0.16);
    params.inflation_radius = getParam("inflation_radius", 0.20);
    params.snap_start_to_free_radius = getParam("snap_start_to_free_radius", 0.30);
    params.snap_goal_to_free_radius = getParam("snap_goal_to_free_radius", 0.80);
    params.planning_timeout = getParam("planning_timeout", 1.0);
    params.max_iterations = getParam("max_iterations", 200000);
    params.clearance_cost_weight = getParam("clearance_cost_weight", 1.0);
    params.densify_spacing = getParam("densify_spacing", 0.08);
    params.simplify_path = getParam("simplify_path", true);
    publish_empty_path_on_failure_ = getParam("publish_empty_path_on_failure", true);

    map_topic_ = getParam("map_topic", std::string("/robot1/map"));
    odom_topic_ = getParam("odom_topic", std::string("/robot1/odom"));
    goal_pose_topic_ = getParam("goal_pose_topic", std::string("/robot1/goal_pose"));
    global_path_topic_ = getParam("global_path_topic", std::string("/robot1/global_path"));
    planner_state_topic_ = getParam("planner_state_topic", std::string("/robot1/planner_state"));

    planner_.configure(params);

    auto path_qos = rclcpp::QoS(1).transient_local().reliable();
    path_pub_ = create_publisher<nav_msgs::msg::Path>(global_path_topic_, path_qos);
    state_pub_ = create_publisher<std_msgs::msg::String>(planner_state_topic_, 10);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        planner_.setMap(*msg);
        map_frame_ = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
        publishState("MAP_READY", 0, 0);
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_.x = msg->pose.pose.position.x;
        current_pose_.y = msg->pose.pose.position.y;
        current_pose_.yaw = yawFromQuaternion(msg->pose.pose.orientation);
        has_odom_ = true;
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        handleGoal(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "A* global planner ready: %s + %s -> %s",
      map_topic_.c_str(), goal_pose_topic_.c_str(), global_path_topic_.c_str());
  }

private:
  template<typename T>
  T getParam(const std::string & name, const T & default_value)
  {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    return get_parameter(name).get_value<T>();
  }

  void handleGoal(const geometry_msgs::msg::PoseStamped & goal)
  {
    if (!planner_.hasMap()) {
      publishState("NO_MAP", 0, 0);
      if (publish_empty_path_on_failure_) {
        publishPath({}, "map");
      }
      return;
    }
    if (!has_odom_) {
      publishState("NO_ODOM", 0, 0);
      if (publish_empty_path_on_failure_) {
        publishPath({}, map_frame_);
      }
      return;
    }

    const Point2D start{current_pose_.x, current_pose_.y};
    const Point2D target{goal.pose.position.x, goal.pose.position.y};
    AStarPlanResult result = planner_.plan(start, target);
    if (result.success) {
      publishPath(result.path, map_frame_);
    } else if (publish_empty_path_on_failure_) {
      publishPath({}, map_frame_);
    }
    publishState(result.state, result.expanded_nodes, result.path.size());
  }

  void publishPath(const std::vector<Point2D> & path, const std::string & frame_id)
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id.empty() ? "map" : frame_id;
    msg.poses.reserve(path.size());
    for (const Point2D & point : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }
    path_pub_->publish(msg);
  }

  void publishState(const std::string & state, int expanded_nodes, std::size_t path_size)
  {
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << "{\"state\":\"" << state << "\","
        << "\"expanded_nodes\":" << expanded_nodes << ","
        << "\"path_size\":" << path_size << "}";
    msg.data = out.str();
    state_pub_->publish(msg);
  }

  AStarPlanner planner_;
  Pose2D current_pose_;
  bool has_odom_{false};
  bool publish_empty_path_on_failure_{true};
  std::string map_frame_{"map"};
  std::string map_topic_;
  std::string odom_topic_;
  std::string goal_pose_topic_;
  std::string global_path_topic_;
  std::string planner_state_topic_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
};

}  // namespace robot_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<robot_planner::AStarGlobalPlannerNode>(options));
  rclcpp::shutdown();
  return 0;
}
