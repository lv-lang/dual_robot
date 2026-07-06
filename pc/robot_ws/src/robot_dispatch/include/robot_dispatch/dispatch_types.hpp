#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace robot_dispatch
{

enum class TaskType
{
  DELIVERY,
  INSPECTION,
  RECHECK
};

enum class TaskState
{
  CREATED,
  PENDING,
  ASSIGNED,
  RUNNING,
  WAITING_CONFIRMATION,
  WAITING_RESOURCE,
  PAUSED,
  RESUMING,
  SUCCEEDED,
  FAILED,
  CANCELED
};

enum class RobotState
{
  IDLE,
  ASSIGNED,
  EXECUTING,
  WAITING_CONFIRMATION,
  WAITING_RESOURCE,
  RETURNING_HOME,
  PAUSED,
  ESTOP,
  ERROR
};

enum class ConfirmationResult
{
  OK,
  ABNORMAL,
  REJECT
};

enum class PointKind
{
  WAITING_AREA,
  PICKUP,
  DELIVERY,
  INSPECTION
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct TaskPoint
{
  std::string id;
  PointKind kind{PointKind::INSPECTION};
  std::string label;
  Pose2D pose;
  bool temporary{false};
};

struct RobotRecord
{
  std::string id;
  std::string waiting_area_id;
  RobotState state{RobotState::IDLE};
  int active_task_id{0};
};

struct TaskRecord
{
  int id{0};
  TaskType type{TaskType::DELIVERY};
  TaskState state{TaskState::CREATED};
  std::vector<std::string> target_point_ids;
  std::string assigned_robot_id;
  std::string preferred_robot_id{"auto"};
  std::uint64_t created_sequence{0};
  std::size_t current_step_index{0};
  int parent_task_id{0};
  std::string originating_robot_id;
  std::string abnormal_point_id;
  std::string business_result;
  std::string failure_reason;
};

struct LockHolder
{
  int task_id{0};
  TaskType task_type{TaskType::DELIVERY};
  std::string robot_id;
  int parent_task_id{0};
  std::string abnormal_point_id;
};

struct ResourceLock
{
  std::string point_id;
  std::vector<LockHolder> holders;
};

struct DispatchDecision
{
  int task_id{0};
  TaskType task_type{TaskType::DELIVERY};
  std::string robot_id;
  std::vector<std::string> locked_point_ids;
  bool interrupted_return_home{false};
};

const char * toString(TaskType type);
const char * toString(TaskState state);
const char * toString(RobotState state);
const char * toString(ConfirmationResult result);
const char * toString(PointKind kind);

TaskType taskTypeFromString(const std::string & value);
PointKind pointKindFromString(const std::string & value);

}  // namespace robot_dispatch
