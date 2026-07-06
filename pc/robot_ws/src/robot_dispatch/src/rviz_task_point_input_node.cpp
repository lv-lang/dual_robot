#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_interfaces/msg/task_point.hpp"
#include "robot_interfaces/srv/add_task_point.hpp"

namespace robot_dispatch
{
namespace
{

using namespace std::chrono_literals;

geometry_msgs::msg::PoseStamped poseFromClickedPoint(
  const geometry_msgs::msg::PointStamped & point)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = point.header;
  pose.pose.position.x = point.point.x;
  pose.pose.position.y = point.point.y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

const char * kindName(uint8_t kind)
{
  if (kind == robot_interfaces::msg::TaskPoint::KIND_PICKUP) {
    return "pickup";
  }
  if (kind == robot_interfaces::msg::TaskPoint::KIND_DELIVERY) {
    return "delivery";
  }
  return "inspection";
}

}  // namespace

class RvizTaskPointInputNode : public rclcpp::Node
{
public:
  RvizTaskPointInputNode()
  : Node("rviz_task_point_input")
  {
    const auto service_name = declare_parameter<std::string>(
      "add_task_point_service", "/robot_dispatch/add_task_point");
    const auto pickup_topic = declare_parameter<std::string>(
      "pickup_topic", "/robot_dispatch/rviz/add_pickup_point");
    const auto delivery_topic = declare_parameter<std::string>(
      "delivery_topic", "/robot_dispatch/rviz/add_delivery_point");
    const auto inspection_topic = declare_parameter<std::string>(
      "inspection_topic", "/robot_dispatch/rviz/add_inspection_point");

    add_point_client_ =
      create_client<robot_interfaces::srv::AddTaskPoint>(service_name);
    pickup_sub_ = subscribeTypedPoint(
      pickup_topic, robot_interfaces::msg::TaskPoint::KIND_PICKUP);
    delivery_sub_ = subscribeTypedPoint(
      delivery_topic, robot_interfaces::msg::TaskPoint::KIND_DELIVERY);
    inspection_sub_ = subscribeTypedPoint(
      inspection_topic, robot_interfaces::msg::TaskPoint::KIND_INSPECTION);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscribeTypedPoint(
    const std::string & topic,
    uint8_t kind)
  {
    return create_subscription<geometry_msgs::msg::PointStamped>(
      topic,
      rclcpp::QoS(10),
      [this, kind](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        addPoint(*msg, kind);
      });
  }

  void addPoint(const geometry_msgs::msg::PointStamped & point, uint8_t kind)
  {
    if (!add_point_client_->wait_for_service(200ms)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waiting for /robot_dispatch/add_task_point");
      return;
    }

    auto request = std::make_shared<robot_interfaces::srv::AddTaskPoint::Request>();
    request->requester = "rviz";
    request->kind = kind;
    request->pose = poseFromClickedPoint(point);

    add_point_client_->async_send_request(
      request,
      [this, kind](rclcpp::Client<robot_interfaces::srv::AddTaskPoint>::SharedFuture future) {
        const auto response = future.get();
        if (response->accepted) {
          RCLCPP_INFO(
            get_logger(), "added %s task point %s",
            kindName(kind), response->point.point_id.c_str());
        } else {
          RCLCPP_WARN(
            get_logger(), "rejected %s task point: %s",
            kindName(kind), response->message.c_str());
        }
      });
  }

  rclcpp::Client<robot_interfaces::srv::AddTaskPoint>::SharedPtr add_point_client_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pickup_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr delivery_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr inspection_sub_;
};

}  // namespace robot_dispatch

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_dispatch::RvizTaskPointInputNode>());
  rclcpp::shutdown();
  return 0;
}
