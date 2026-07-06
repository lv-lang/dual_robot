#include <chrono>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_dispatch/marker_builder.hpp"
#include "robot_dispatch/task_points.hpp"

namespace robot_dispatch
{

class TaskMarkerPublisher : public rclcpp::Node
{
public:
  TaskMarkerPublisher()
  : Node("task_marker_publisher")
  {
    const auto default_path =
      ament_index_cpp::get_package_share_directory("robot_dispatch") +
      "/config/task_points.yaml";
    const auto task_points_file =
      declare_parameter<std::string>("task_points_file", default_path);
    marker_topic_ = declare_parameter<std::string>("marker_topic", kMarkerTopic);
    config_ = loadTaskPointsFromYaml(task_points_file);
    publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(1).transient_local());
    timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() {
        publisher_->publish(buildTaskPointMarkers(config_));
      });
    RCLCPP_INFO(
      get_logger(), "publishing task markers from %s on %s",
      task_points_file.c_str(), marker_topic_.c_str());
  }

private:
  std::string marker_topic_;
  TaskPointConfig config_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_dispatch

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_dispatch::TaskMarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}

