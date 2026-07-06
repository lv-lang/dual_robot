#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "robot_mission/mission_executor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_mission::MissionExecutorNode>());
  rclcpp::shutdown();
  return 0;
}

