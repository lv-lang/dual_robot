#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

#include "robot_planner/mpc_path_tracker.hpp"

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

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

}  // namespace

class MpcDwbPlannerNode : public rclcpp::Node
{
public:
  explicit MpcDwbPlannerNode(const rclcpp::NodeOptions & options)
  : Node("robot1_mpc_dwb_planner", options)
  {
    params_.control_frequency = getParam("control_frequency", 15.0);
    params_.xy_goal_tolerance = getParam("xy_goal_tolerance", 0.12);
    params_.lookahead_distance = getParam("lookahead_distance", 0.65);
    params_.reacquire_distance = getParam("reacquire_distance", 0.75);
    params_.closest_search_window = getParam("closest_search_window", 40);
    params_.min_vx = getParam("min_vx", 0.0);
    params_.min_tracking_vx = getParam("min_tracking_vx", 0.06);
    params_.max_vx = getParam("max_vx", 0.32);
    params_.max_wz = getParam("max_wz", 0.65);
    params_.rotate_in_place_heading_error = getParam("rotate_in_place_heading_error", 2.35);
    params_.target_speed = getParam("target_speed", 0.24);
    params_.acc_lim_x = getParam("acc_lim_x", 0.50);
    params_.acc_lim_theta = getParam("acc_lim_theta", 0.80);
    params_.vx_samples = getParam("vx_samples", 6);
    params_.wz_samples = getParam("wz_samples", 9);
    params_.horizon_steps = getParam("horizon_steps", 14);
    params_.dt = getParam("dt", 0.10);
    params_.path_weight = getParam("path_weight", 5.0);
    params_.goal_weight = getParam("goal_weight", 1.0);
    params_.heading_weight = getParam("heading_weight", 0.55);
    params_.progress_weight = getParam("progress_weight", 0.65);
    params_.velocity_weight = getParam("velocity_weight", 0.35);
    params_.smoothness_weight = getParam("smoothness_weight", 0.90);
    params_.angular_weight = getParam("angular_weight", 0.35);
    params_.hard_stop_distance = getParam("hard_stop_distance", 0.18);

    path_timeout_ = getParam("path_timeout", 0.0);
    odom_timeout_ = getParam("odom_timeout", 0.5);
    scan_timeout_ = getParam("scan_timeout", 0.5);
    require_fresh_scan_ = getParam("require_fresh_scan", true);
    front_check_distance_ = getParam("front_check_distance", 0.75);
    front_check_width_ = getParam("front_check_width", 0.34);

    global_path_topic_ = getParam("global_path_topic", std::string("/robot1/global_path"));
    odom_topic_ = getParam("odom_topic", std::string("/robot1/odom"));
    scan_topic_ = getParam("scan_topic", std::string("/robot1/scan"));
    cmd_vel_topic_ = getParam("cmd_vel_topic", std::string("/robot1/cmd_vel"));
    local_path_topic_ = getParam("local_path_topic", std::string("/robot1/local_path"));
    planner_state_topic_ = getParam("planner_state_topic", std::string("/robot1/planner_state"));

    tracker_.configure(params_);

    auto path_qos = rclcpp::QoS(1).transient_local().reliable();
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic_, path_qos,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        global_path_.clear();
        global_path_.reserve(msg->poses.size());
        path_frame_ = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
        for (const auto & pose : msg->poses) {
          global_path_.push_back(Point2D{pose.pose.position.x, pose.pose.position.y});
        }
        last_path_stamp_ = now();
        tracker_.reset();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 30,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_.x = msg->pose.pose.position.x;
        current_pose_.y = msg->pose.pose.position.y;
        current_pose_.yaw = yawFromQuaternion(msg->pose.pose.orientation);
        current_velocity_.vx = msg->twist.twist.linear.x;
        current_velocity_.vy = 0.0;
        current_velocity_.wz = msg->twist.twist.angular.z;
        last_odom_stamp_ = now();
        has_odom_ = true;
      });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        front_clearance_ = computeFrontClearance(*msg);
        last_scan_stamp_ = now();
        has_scan_ = true;
      });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>(local_path_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(planner_state_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, params_.control_frequency)),
      [this]() {onTimer();});

    RCLCPP_WARN(
      get_logger(),
      "MPC-DWB direct planner is publishing %s directly. First version uses diff-drive vx/wz; linear.y=0.",
      cmd_vel_topic_.c_str());
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

  void onTimer()
  {
    const rclcpp::Time current = now();
    if (global_path_.empty()) {
      publishZero("EMPTY_PATH");
      return;
    }
    if (!has_odom_ || (current - last_odom_stamp_).seconds() > odom_timeout_) {
      publishZero("STALE_ODOM");
      return;
    }
    if (path_timeout_ > 0.0 && (current - last_path_stamp_).seconds() > path_timeout_) {
      publishZero("STALE_PATH");
      return;
    }
    if (require_fresh_scan_ && (!has_scan_ || (current - last_scan_stamp_).seconds() > scan_timeout_)) {
      publishZero("STALE_SCAN");
      return;
    }

    PlannerResult result = tracker_.computeCommand(
      current_pose_, current_velocity_, global_path_, front_clearance_);
    publishCommand(result.cmd);
    publishLocalPath(result.trajectory);
    publishState(result.state, result.score, front_clearance_);
  }

  double computeFrontClearance(const sensor_msgs::msg::LaserScan & scan) const
  {
    double min_clearance = std::numeric_limits<double>::infinity();
    double angle = scan.angle_min;
    for (const float range : scan.ranges) {
      if (std::isfinite(range) && range >= scan.range_min && range <= scan.range_max) {
        const double x = static_cast<double>(range) * std::cos(angle);
        const double y = static_cast<double>(range) * std::sin(angle);
        if (x > 0.0 && x <= front_check_distance_ && std::abs(y) <= front_check_width_ * 0.5) {
          min_clearance = std::min(min_clearance, static_cast<double>(range));
        }
      }
      angle += scan.angle_increment;
    }
    return min_clearance;
  }

  void publishCommand(const Velocity2D & cmd)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = cmd.vx;
    msg.linear.y = 0.0;
    msg.angular.z = cmd.wz;
    cmd_pub_->publish(msg);
  }

  void publishZero(const std::string & state)
  {
    publishCommand(Velocity2D{});
    publishLocalPath({});
    publishState(state, 0.0, front_clearance_);
  }

  void publishLocalPath(const std::vector<Pose2D> & trajectory)
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = path_frame_;
    msg.poses.reserve(trajectory.size());
    for (const Pose2D & pose_2d : trajectory) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = pose_2d.x;
      pose.pose.position.y = pose_2d.y;
      pose.pose.orientation = quaternionFromYaw(pose_2d.yaw);
      msg.poses.push_back(pose);
    }
    local_path_pub_->publish(msg);
  }

  void publishState(const std::string & state, double score, double front_clearance)
  {
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << "{\"state\":\"" << state << "\","
        << "\"score\":" << score << ","
        << "\"front_clearance\":";
    if (std::isfinite(front_clearance)) {
      out << front_clearance;
    } else {
      out << "null";
    }
    out << ",\"model\":\"diff_drive_mpc_dwb\"}";
    msg.data = out.str();
    state_pub_->publish(msg);
  }

  MpcDwbParams params_;
  MpcPathTracker tracker_;
  Pose2D current_pose_;
  Velocity2D current_velocity_;
  std::vector<Point2D> global_path_;
  std::string path_frame_{"map"};
  double front_clearance_{std::numeric_limits<double>::infinity()};
  double path_timeout_{0.0};
  double odom_timeout_{0.5};
  double scan_timeout_{0.5};
  bool require_fresh_scan_{true};
  bool has_odom_{false};
  bool has_scan_{false};
  double front_check_distance_{0.75};
  double front_check_width_{0.34};

  std::string global_path_topic_;
  std::string odom_topic_;
  std::string scan_topic_;
  std::string cmd_vel_topic_;
  std::string local_path_topic_;
  std::string planner_state_topic_;

  rclcpp::Time last_path_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<robot_planner::MpcDwbPlannerNode>(options));
  rclcpp::shutdown();
  return 0;
}
