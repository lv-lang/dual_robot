#include "robot_mission/robot_health_monitor_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_mission::RobotHealthMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
