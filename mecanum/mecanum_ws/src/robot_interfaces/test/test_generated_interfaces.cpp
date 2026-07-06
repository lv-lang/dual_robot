#include <gtest/gtest.h>

#include <robot_interfaces/action/execute_mission.hpp>
#include <robot_interfaces/msg/dispatch_lease.hpp>
#include <robot_interfaces/msg/mission_command.hpp>
#include <robot_interfaces/msg/mission_step.hpp>
#include <robot_interfaces/msg/resource_lock.hpp>
#include <robot_interfaces/msg/robot_heartbeat.hpp>
#include <robot_interfaces/msg/robot_health.hpp>
#include <robot_interfaces/msg/robot_state.hpp>
#include <robot_interfaces/msg/system_state.hpp>
#include <robot_interfaces/msg/task.hpp>
#include <robot_interfaces/msg/task_point.hpp>
#include <robot_interfaces/msg/task_state.hpp>
#include <robot_interfaces/srv/add_task_point.hpp>
#include <robot_interfaces/srv/cancel_task.hpp>
#include <robot_interfaces/srv/clear_temporary_task_points.hpp>
#include <robot_interfaces/srv/confirm_task_step.hpp>
#include <robot_interfaces/srv/create_task.hpp>
#include <robot_interfaces/srv/emergency_stop.hpp>
#include <robot_interfaces/srv/enable_system.hpp>
#include <robot_interfaces/srv/get_dispatch_state.hpp>
#include <robot_interfaces/srv/get_task_points.hpp>
#include <robot_interfaces/srv/pause_task.hpp>
#include <robot_interfaces/srv/recover_system.hpp>
#include <robot_interfaces/srv/resume_task.hpp>

TEST(GeneratedInterfaces, ExposeTaskRobotAndResourceConstants)
{
  robot_interfaces::msg::Task task;
  task.task_type = robot_interfaces::msg::Task::TYPE_DELIVERY;
  task.state.state = robot_interfaces::msg::TaskState::PENDING;
  task.business_result = robot_interfaces::msg::Task::RESULT_NONE;

  robot_interfaces::msg::RobotState robot_state;
  robot_state.robot_id = "robot1";
  robot_state.robot_namespace = "/robot1";
  robot_state.state = robot_interfaces::msg::RobotState::IDLE;

  robot_interfaces::msg::ResourceLock lock;
  lock.resource_type = robot_interfaces::msg::ResourceLock::TYPE_PICKUP;
  lock.status = robot_interfaces::msg::ResourceLock::STATUS_LOCKED;

  robot_interfaces::msg::RobotHeartbeat heartbeat;
  heartbeat.robot_id = "mecanum";
  heartbeat.mission_state = robot_interfaces::msg::RobotHeartbeat::IDLE;
  heartbeat.map_version = "real-v1";

  robot_interfaces::msg::RobotHealth health;
  health.robot_id = "mecanum";
  health.health_state = robot_interfaces::msg::RobotHealth::OK;
  health.scan_ok = true;
  health.map_bundle_hash = "hash";

  robot_interfaces::msg::SystemState system_state;
  system_state.state = robot_interfaces::msg::SystemState::STANDBY;
  system_state.requires_operator_action = true;

  robot_interfaces::msg::TaskPoint point;
  point.point_id = "RVIZ_INSPECTION_1";
  point.kind = robot_interfaces::msg::TaskPoint::KIND_INSPECTION;
  point.temporary = true;

  EXPECT_EQ(robot_interfaces::msg::Task::TYPE_DELIVERY, task.task_type);
  EXPECT_EQ(robot_interfaces::msg::TaskState::PENDING, task.state.state);
  EXPECT_EQ(robot_interfaces::msg::RobotState::IDLE, robot_state.state);
  EXPECT_EQ(robot_interfaces::msg::RobotHeartbeat::IDLE, heartbeat.mission_state);
  EXPECT_EQ(robot_interfaces::msg::RobotHealth::OK, health.health_state);
  EXPECT_TRUE(health.scan_ok);
  EXPECT_EQ(robot_interfaces::msg::SystemState::STANDBY, system_state.state);
  EXPECT_EQ(11, robot_interfaces::msg::TaskState::WAITING_RESOURCE);
  EXPECT_EQ(9, robot_interfaces::msg::RobotState::WAITING_RESOURCE);
  EXPECT_EQ(robot_interfaces::msg::ResourceLock::STATUS_LOCKED, lock.status);
  EXPECT_EQ(robot_interfaces::msg::TaskPoint::KIND_INSPECTION, point.kind);
  EXPECT_TRUE(point.temporary);
}

TEST(GeneratedInterfaces, ExposeDispatchLease)
{
  robot_interfaces::msg::DispatchLease lease;
  lease.robot_id = "ackermann";
  lease.robot_namespace = "/ackermann";
  lease.lease_seq = 7;
  lease.run_allowed = true;
  lease.system_state = robot_interfaces::msg::SystemState::READY;

  EXPECT_EQ("ackermann", lease.robot_id);
  EXPECT_TRUE(lease.run_allowed);
  EXPECT_EQ(robot_interfaces::msg::SystemState::READY, lease.system_state);
}

TEST(GeneratedInterfaces, SupportPointIdAndMapPoseMissionCommands)
{
  robot_interfaces::msg::MissionStep step;
  step.sequence = 0;
  step.step_type = robot_interfaces::msg::MissionStep::STEP_NAVIGATE;
  step.point_id = "P1";
  step.target_pose.header.frame_id = "map";
  step.target_pose.pose.position.x = 1.0;
  step.target_pose.pose.orientation.w = 1.0;

  robot_interfaces::msg::MissionCommand command;
  command.task_id = "task_1";
  command.task_type = robot_interfaces::msg::MissionCommand::TYPE_INSPECTION;
  command.assigned_robot_id = "robot2";
  command.steps.push_back(step);

  ASSERT_EQ(1u, command.steps.size());
  EXPECT_EQ("P1", command.steps.front().point_id);
  EXPECT_EQ("map", command.steps.front().target_pose.header.frame_id);
}

TEST(GeneratedInterfaces, ExposeControlServicesAndMissionAction)
{
  robot_interfaces::srv::CreateTask::Request create_request;
  create_request.task_type = robot_interfaces::msg::Task::TYPE_RECHECK;

  robot_interfaces::srv::ConfirmTaskStep::Request confirm_request;
  confirm_request.result = robot_interfaces::srv::ConfirmTaskStep::Request::RESULT_ABNORMAL;

  robot_interfaces::srv::CancelTask::Request cancel_request;
  robot_interfaces::srv::PauseTask::Request pause_request;
  robot_interfaces::srv::ResumeTask::Request resume_request;
  robot_interfaces::srv::EmergencyStop::Request estop_request;
  robot_interfaces::srv::EnableSystem::Request enable_request;
  robot_interfaces::srv::RecoverSystem::Request recover_request;
  robot_interfaces::srv::GetDispatchState::Response state_response;
  robot_interfaces::srv::AddTaskPoint::Request add_point_request;
  robot_interfaces::srv::GetTaskPoints::Response points_response;
  robot_interfaces::srv::ClearTemporaryTaskPoints::Response clear_points_response;
  robot_interfaces::action::ExecuteMission::Goal goal;
  robot_interfaces::msg::Task state_task;
  robot_interfaces::msg::TaskPoint point;

  cancel_request.task_id = "task_1";
  pause_request.task_id = "task_1";
  resume_request.task_id = "task_1";
  estop_request.active = true;
  enable_request.operator_confirmed = true;
  recover_request.operator_confirmed = true;
  add_point_request.kind = robot_interfaces::msg::TaskPoint::KIND_PICKUP;
  point.point_id = "RVIZ_PICKUP_1";
  points_response.points.push_back(point);
  clear_points_response.cleared_count = 1;
  goal.command.task_id = "task_1";
  state_task.task_id = "task_1";
  state_response.tasks.push_back(state_task);

  EXPECT_EQ(robot_interfaces::msg::Task::TYPE_RECHECK, create_request.task_type);
  EXPECT_EQ(robot_interfaces::srv::ConfirmTaskStep::Request::RESULT_ABNORMAL, confirm_request.result);
  EXPECT_TRUE(estop_request.active);
  EXPECT_TRUE(enable_request.operator_confirmed);
  EXPECT_TRUE(recover_request.operator_confirmed);
  EXPECT_EQ(robot_interfaces::msg::TaskPoint::KIND_PICKUP, add_point_request.kind);
  EXPECT_EQ(1u, points_response.points.size());
  EXPECT_EQ(1u, clear_points_response.cleared_count);
  EXPECT_EQ("task_1", goal.command.task_id);
  EXPECT_EQ(1u, state_response.tasks.size());
}
