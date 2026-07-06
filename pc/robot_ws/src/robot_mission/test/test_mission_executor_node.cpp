#include <gtest/gtest.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "robot_mission/mission_executor_node.hpp"

namespace robot_mission
{

TEST(MissionExecutorNode, ConstructsWithNav2ExecutionDisabledForTests)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("robot_name", "robot1"),
    rclcpp::Parameter("enable_nav2_execution", false),
  });
  auto node = std::make_shared<MissionExecutorNode>(options);
  ASSERT_NE(node, nullptr);
  EXPECT_STREQ(node->get_name(), "mission_executor");
}

}  // namespace robot_mission

