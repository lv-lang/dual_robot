#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/execute_mission.hpp"
#include "robot_interfaces/msg/dispatch_lease.hpp"
#include "robot_interfaces/msg/robot_heartbeat.hpp"

namespace robot_mission
{

class MissionExecutorNode : public rclcpp::Node
{
public:
  explicit MissionExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using ExecuteMission = robot_interfaces::action::ExecuteMission;
  using GoalHandleExecuteMission = rclcpp_action::ServerGoalHandle<ExecuteMission>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteMission::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleExecuteMission> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleExecuteMission> goal_handle);
  void execute(const std::shared_ptr<GoalHandleExecuteMission> goal_handle);
  bool executeNavigationStep(
    const std::shared_ptr<GoalHandleExecuteMission> & goal_handle,
    const robot_interfaces::msg::MissionStep & step,
    std::string * error);
  void publishFeedback(
    const std::shared_ptr<GoalHandleExecuteMission> & goal_handle,
    uint8_t execution_state,
    uint32_t step_index,
    const robot_interfaces::msg::MissionStep & step,
    const std::string & message);
  void onDispatchLease(const robot_interfaces::msg::DispatchLease::SharedPtr msg);
  bool dispatchLeaseAllowsMotion() const;
  void publishHeartbeat();
  void setMissionState(uint8_t state, const std::string & message);

  std::string robot_name_;
  std::string robot_id_;
  std::string robot_namespace_;
  std::string map_version_;
  std::string nav2_action_name_;
  double navigation_timeout_sec_{180.0};
  double dispatch_lease_timeout_sec_{5.0};
  bool enable_nav2_execution_{true};
  bool require_dispatch_lease_{true};

  rclcpp_action::Server<ExecuteMission>::SharedPtr mission_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  rclcpp::Publisher<robot_interfaces::msg::RobotHeartbeat>::SharedPtr heartbeat_pub_;
  rclcpp::Subscription<robot_interfaces::msg::DispatchLease>::SharedPtr dispatch_lease_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  mutable std::mutex mission_state_mutex_;
  uint8_t mission_state_{robot_interfaces::msg::RobotHeartbeat::IDLE};
  std::string mission_message_{"idle"};
  bool has_dispatch_lease_{false};
  bool dispatch_lease_run_allowed_{false};
  rclcpp::Time last_dispatch_lease_;
};

}  // namespace robot_mission
