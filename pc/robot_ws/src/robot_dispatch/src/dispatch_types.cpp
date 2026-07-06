#include "robot_dispatch/dispatch_types.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace robot_dispatch
{

namespace
{

std::string lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

}  // namespace

const char * toString(TaskType type)
{
  switch (type) {
    case TaskType::DELIVERY:
      return "DELIVERY";
    case TaskType::INSPECTION:
      return "INSPECTION";
    case TaskType::RECHECK:
      return "RECHECK";
  }
  return "UNKNOWN";
}

const char * toString(TaskState state)
{
  switch (state) {
    case TaskState::CREATED:
      return "CREATED";
    case TaskState::PENDING:
      return "PENDING";
    case TaskState::ASSIGNED:
      return "ASSIGNED";
    case TaskState::RUNNING:
      return "RUNNING";
    case TaskState::WAITING_CONFIRMATION:
      return "WAITING_CONFIRMATION";
    case TaskState::WAITING_RESOURCE:
      return "WAITING_RESOURCE";
    case TaskState::PAUSED:
      return "PAUSED";
    case TaskState::RESUMING:
      return "RESUMING";
    case TaskState::SUCCEEDED:
      return "SUCCEEDED";
    case TaskState::FAILED:
      return "FAILED";
    case TaskState::CANCELED:
      return "CANCELED";
  }
  return "UNKNOWN";
}

const char * toString(RobotState state)
{
  switch (state) {
    case RobotState::IDLE:
      return "IDLE";
    case RobotState::ASSIGNED:
      return "ASSIGNED";
    case RobotState::EXECUTING:
      return "EXECUTING";
    case RobotState::WAITING_CONFIRMATION:
      return "WAITING_CONFIRMATION";
    case RobotState::WAITING_RESOURCE:
      return "WAITING_RESOURCE";
    case RobotState::RETURNING_HOME:
      return "RETURNING_HOME";
    case RobotState::PAUSED:
      return "PAUSED";
    case RobotState::ESTOP:
      return "ESTOP";
    case RobotState::ERROR:
      return "ERROR";
  }
  return "UNKNOWN";
}

const char * toString(ConfirmationResult result)
{
  switch (result) {
    case ConfirmationResult::OK:
      return "OK";
    case ConfirmationResult::ABNORMAL:
      return "ABNORMAL";
    case ConfirmationResult::REJECT:
      return "REJECT";
  }
  return "UNKNOWN";
}

const char * toString(PointKind kind)
{
  switch (kind) {
    case PointKind::WAITING_AREA:
      return "WAITING_AREA";
    case PointKind::PICKUP:
      return "PICKUP";
    case PointKind::DELIVERY:
      return "DELIVERY";
    case PointKind::INSPECTION:
      return "INSPECTION";
  }
  return "UNKNOWN";
}

TaskType taskTypeFromString(const std::string & value)
{
  const auto normalized = lower(value);
  if (normalized == "delivery") {
    return TaskType::DELIVERY;
  }
  if (normalized == "inspection") {
    return TaskType::INSPECTION;
  }
  if (normalized == "recheck") {
    return TaskType::RECHECK;
  }
  throw std::invalid_argument("unknown task type: " + value);
}

PointKind pointKindFromString(const std::string & value)
{
  const auto normalized = lower(value);
  if (normalized == "waiting_area") {
    return PointKind::WAITING_AREA;
  }
  if (normalized == "pickup") {
    return PointKind::PICKUP;
  }
  if (normalized == "delivery") {
    return PointKind::DELIVERY;
  }
  if (normalized == "inspection") {
    return PointKind::INSPECTION;
  }
  throw std::invalid_argument("unknown point kind: " + value);
}

}  // namespace robot_dispatch
