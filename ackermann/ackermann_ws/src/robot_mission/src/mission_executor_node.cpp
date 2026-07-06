#include "robot_mission/mission_executor_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace robot_mission
{

namespace
{

using namespace std::chrono_literals;

robot_interfaces::action::ExecuteMission::Result resultMessage(
  uint8_t status,
  const std::string & task_id,
  uint32_t step_index,
  const std::string & step_id,
  const std::string & point_id,
  bool success,
  const std::string & message)
{
  robot_interfaces::action::ExecuteMission::Result result;
  result.status = status;
  result.task_id = task_id;
  result.final_step_index = step_index;
  result.final_step_id = step_id;
  result.final_point_id = point_id;
  result.success = success;
  result.message = message;
  return result;
}

}  // namespace

MissionExecutorNode::MissionExecutorNode(const rclcpp::NodeOptions & options)
: Node("mission_executor", options)
{
  robot_name_ = declare_parameter<std::string>("robot_name", get_namespace());
  robot_id_ = declare_parameter<std::string>("robot_id", robot_name_);
  robot_namespace_ = declare_parameter<std::string>("robot_namespace", get_namespace());
  map_version_ = declare_parameter<std::string>("map_version", "");
  nav2_action_name_ = declare_parameter<std::string>("nav2_action_name", "navigate_to_pose");
  navigation_timeout_sec_ =
    declare_parameter<double>("navigation_timeout_sec", 180.0);
  dispatch_lease_timeout_sec_ =
    declare_parameter<double>("dispatch_lease_timeout_sec", 5.0);
  enable_nav2_execution_ =
    declare_parameter<bool>("enable_nav2_execution", true);
  require_dispatch_lease_ =
    declare_parameter<bool>("require_dispatch_lease", true);

  mission_server_ = rclcpp_action::create_server<ExecuteMission>(
    this,
    "execute_mission",
    std::bind(&MissionExecutorNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MissionExecutorNode::handleCancel, this, std::placeholders::_1),
    std::bind(&MissionExecutorNode::handleAccepted, this, std::placeholders::_1));

  nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav2_action_name_);
  heartbeat_pub_ = create_publisher<robot_interfaces::msg::RobotHeartbeat>(
    "heartbeat", rclcpp::QoS(1).best_effort());
  dispatch_lease_sub_ = create_subscription<robot_interfaces::msg::DispatchLease>(
    "dispatch_lease", rclcpp::QoS(1).best_effort(),
    std::bind(&MissionExecutorNode::onDispatchLease, this, std::placeholders::_1));
  heartbeat_timer_ = create_wall_timer(500ms, std::bind(&MissionExecutorNode::publishHeartbeat, this));

  RCLCPP_INFO(
    get_logger(), "mission executor ready for %s using Nav2 action %s map_version=%s",
    robot_name_.c_str(), nav2_action_name_.c_str(), map_version_.c_str());
}

rclcpp_action::GoalResponse MissionExecutorNode::handleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const ExecuteMission::Goal> goal)
{
  if (goal->command.steps.empty()) {
    RCLCPP_WARN(get_logger(), "rejecting empty mission command");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!dispatchLeaseAllowsMotion()) {
    RCLCPP_WARN(get_logger(), "rejecting mission without active dispatch lease");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MissionExecutorNode::handleCancel(
  const std::shared_ptr<GoalHandleExecuteMission>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MissionExecutorNode::handleAccepted(
  const std::shared_ptr<GoalHandleExecuteMission> goal_handle)
{
  std::thread{std::bind(&MissionExecutorNode::execute, this, goal_handle)}.detach();
}

void MissionExecutorNode::execute(
  const std::shared_ptr<GoalHandleExecuteMission> goal_handle)
{
  setMissionState(robot_interfaces::msg::RobotHeartbeat::EXECUTING, "executing mission");
  const auto goal = goal_handle->get_goal();
  uint32_t step_index = 0;
  std::string final_step_id;
  std::string final_point_id;

  for (const auto & step : goal->command.steps) {
    final_step_id = step.step_id;
    final_point_id = step.point_id;
    if (!dispatchLeaseAllowsMotion()) {
      setMissionState(robot_interfaces::msg::RobotHeartbeat::ESTOP, "dispatch lease invalid");
      auto result = resultMessage(
        ExecuteMission::Result::STATUS_FAILED,
        goal->command.task_id,
        step_index,
        final_step_id,
        final_point_id,
        false,
        "dispatch lease invalid");
      goal_handle->abort(std::make_shared<ExecuteMission::Result>(result));
      return;
    }
    if (goal_handle->is_canceling()) {
      setMissionState(robot_interfaces::msg::RobotHeartbeat::CANCELING, "mission canceling");
      auto result = resultMessage(
        ExecuteMission::Result::STATUS_CANCELED,
        goal->command.task_id,
        step_index,
        final_step_id,
        final_point_id,
        false,
        "mission canceled");
      goal_handle->canceled(std::make_shared<ExecuteMission::Result>(result));
      setMissionState(robot_interfaces::msg::RobotHeartbeat::IDLE, "mission canceled");
      return;
    }

    if (step.step_type == robot_interfaces::msg::MissionStep::STEP_WAIT_CONFIRMATION) {
      publishFeedback(
        goal_handle,
        ExecuteMission::Feedback::EXECUTION_WAITING_CONFIRMATION,
        step_index,
        step,
        "waiting confirmation");
      ++step_index;
      continue;
    }

    const uint8_t feedback_state =
      step.step_type == robot_interfaces::msg::MissionStep::STEP_RETURN_HOME
      ? ExecuteMission::Feedback::EXECUTION_RETURNING_HOME
      : ExecuteMission::Feedback::EXECUTION_NAVIGATING;
    publishFeedback(goal_handle, feedback_state, step_index, step, "navigating");

    std::string error;
    if (!executeNavigationStep(goal_handle, step, &error)) {
      if (goal_handle->is_canceling()) {
        setMissionState(robot_interfaces::msg::RobotHeartbeat::CANCELING, "mission canceling");
        auto result = resultMessage(
          ExecuteMission::Result::STATUS_CANCELED,
          goal->command.task_id,
          step_index,
          final_step_id,
          final_point_id,
          false,
          "mission canceled");
        goal_handle->canceled(std::make_shared<ExecuteMission::Result>(result));
        setMissionState(robot_interfaces::msg::RobotHeartbeat::IDLE, "mission canceled");
        return;
      }
      auto result = resultMessage(
        ExecuteMission::Result::STATUS_FAILED,
        goal->command.task_id,
        step_index,
        final_step_id,
        final_point_id,
        false,
        error);
      goal_handle->abort(std::make_shared<ExecuteMission::Result>(result));
      setMissionState(
        error == "dispatch lease invalid"
        ? robot_interfaces::msg::RobotHeartbeat::ESTOP
        : robot_interfaces::msg::RobotHeartbeat::ERROR,
        error);
      return;
    }
    ++step_index;
  }

  auto result = resultMessage(
    ExecuteMission::Result::STATUS_SUCCEEDED,
    goal->command.task_id,
    step_index == 0 ? 0 : step_index - 1,
    final_step_id,
    final_point_id,
    true,
    "mission succeeded");
  goal_handle->succeed(std::make_shared<ExecuteMission::Result>(result));
  setMissionState(robot_interfaces::msg::RobotHeartbeat::IDLE, "mission succeeded");
}

bool MissionExecutorNode::executeNavigationStep(
  const std::shared_ptr<GoalHandleExecuteMission> & goal_handle,
  const robot_interfaces::msg::MissionStep & step,
  std::string * error)
{
  if (!enable_nav2_execution_) {
    return true;
  }

  if (!nav2_client_->wait_for_action_server(2s)) {
    if (error != nullptr) {
      *error = "Nav2 action server unavailable: " + nav2_action_name_;
    }
    return false;
  }

  NavigateToPose::Goal nav_goal;
  nav_goal.pose = step.target_pose;
  auto goal_future = nav2_client_->async_send_goal(nav_goal);
  if (goal_future.wait_for(std::chrono::duration<double>(navigation_timeout_sec_)) !=
    std::future_status::ready)
  {
    if (error != nullptr) {
      *error = "timed out sending Nav2 goal";
    }
    return false;
  }
  const auto nav_goal_handle = goal_future.get();
  if (!nav_goal_handle) {
    if (error != nullptr) {
      *error = "Nav2 goal rejected";
    }
    return false;
  }

  auto result_future = nav2_client_->async_get_result(nav_goal_handle);
  const auto start = now();
  while (rclcpp::ok()) {
    if (!dispatchLeaseAllowsMotion()) {
      nav2_client_->async_cancel_goal(nav_goal_handle);
      if (error != nullptr) {
        *error = "dispatch lease invalid";
      }
      return false;
    }
    if (goal_handle->is_canceling()) {
      nav2_client_->async_cancel_goal(nav_goal_handle);
      if (error != nullptr) {
        *error = "mission canceled";
      }
      return false;
    }
    if (result_future.wait_for(100ms) == std::future_status::ready) {
      const auto wrapped = result_future.get();
      if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
        return true;
      }
      if (error != nullptr) {
        *error = "Nav2 goal failed";
      }
      return false;
    }
    if ((now() - start).seconds() > navigation_timeout_sec_) {
      nav2_client_->async_cancel_goal(nav_goal_handle);
      if (error != nullptr) {
        *error = "Nav2 goal timeout";
      }
      return false;
    }
  }
  if (error != nullptr) {
    *error = "rclcpp shutdown";
  }
  return false;
}

void MissionExecutorNode::publishFeedback(
  const std::shared_ptr<GoalHandleExecuteMission> & goal_handle,
  uint8_t execution_state,
  uint32_t step_index,
  const robot_interfaces::msg::MissionStep & step,
  const std::string & message)
{
  auto feedback = std::make_shared<ExecuteMission::Feedback>();
  feedback->execution_state = execution_state;
  feedback->task_id = goal_handle->get_goal()->command.task_id;
  feedback->current_step_index = step_index;
  feedback->current_step_id = step.step_id;
  feedback->current_point_id = step.point_id;
  feedback->current_goal = step.target_pose;
  feedback->message = message;
  goal_handle->publish_feedback(feedback);
}

void MissionExecutorNode::onDispatchLease(
  const robot_interfaces::msg::DispatchLease::SharedPtr msg)
{
  if (!msg->robot_id.empty() && msg->robot_id != robot_id_) {
    return;
  }
  if (!msg->robot_namespace.empty() && msg->robot_namespace != robot_namespace_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mission_state_mutex_);
  has_dispatch_lease_ = true;
  dispatch_lease_run_allowed_ = msg->run_allowed;
  last_dispatch_lease_ = now();
}

bool MissionExecutorNode::dispatchLeaseAllowsMotion() const
{
  if (!require_dispatch_lease_ || !enable_nav2_execution_) {
    return true;
  }
  std::lock_guard<std::mutex> lock(mission_state_mutex_);
  if (!has_dispatch_lease_ || !dispatch_lease_run_allowed_) {
    return false;
  }
  return (now() - last_dispatch_lease_).seconds() <= dispatch_lease_timeout_sec_;
}

void MissionExecutorNode::publishHeartbeat()
{
  robot_interfaces::msg::RobotHeartbeat heartbeat;
  heartbeat.header.stamp = now();
  heartbeat.header.frame_id = robot_namespace_;
  heartbeat.robot_id = robot_id_;
  heartbeat.robot_namespace = robot_namespace_;
  heartbeat.map_version = map_version_;
  {
    std::lock_guard<std::mutex> lock(mission_state_mutex_);
    heartbeat.mission_state = mission_state_;
    heartbeat.message = mission_message_;
  }
  heartbeat_pub_->publish(heartbeat);
}

void MissionExecutorNode::setMissionState(uint8_t state, const std::string & message)
{
  std::lock_guard<std::mutex> lock(mission_state_mutex_);
  mission_state_ = state;
  mission_message_ = message;
}

}  // namespace robot_mission
