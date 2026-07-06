#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "robot_dispatch/robot_dispatch_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_dispatch::RobotDispatchNode>());
  rclcpp::shutdown();
  return 0;
}

