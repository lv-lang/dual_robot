#include "robot_dispatch/robot_dispatch_node.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/exceptions.h"
#include "yaml-cpp/yaml.h"

namespace robot_dispatch
{

namespace
{

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

geometry_msgs::msg::PoseStamped poseStampedFromPoint(
  const TaskPointConfig & config,
  const TaskPoint & point,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = config.frame_id.empty() || config.frame_id == "/map"
    ? "map"
    : config.frame_id;
  pose.header.stamp = stamp;
  pose.pose.position.x = point.pose.x;
  pose.pose.position.y = point.pose.y;
  pose.pose.orientation = quaternionFromYaw(point.pose.yaw);
  return pose;
}

uint8_t toInterfaceTaskType(TaskType type)
{
  switch (type) {
    case TaskType::DELIVERY:
      return robot_interfaces::msg::Task::TYPE_DELIVERY;
    case TaskType::INSPECTION:
      return robot_interfaces::msg::Task::TYPE_INSPECTION;
    case TaskType::RECHECK:
      return robot_interfaces::msg::Task::TYPE_RECHECK;
  }
  return robot_interfaces::msg::Task::TYPE_DELIVERY;
}

TaskType fromInterfaceTaskType(uint8_t type)
{
  if (type == robot_interfaces::msg::Task::TYPE_INSPECTION) {
    return TaskType::INSPECTION;
  }
  if (type == robot_interfaces::msg::Task::TYPE_RECHECK) {
    return TaskType::RECHECK;
  }
  return TaskType::DELIVERY;
}

uint8_t toInterfaceTaskState(TaskState state)
{
  switch (state) {
    case TaskState::CREATED:
      return robot_interfaces::msg::TaskState::CREATED;
    case TaskState::PENDING:
      return robot_interfaces::msg::TaskState::PENDING;
    case TaskState::ASSIGNED:
      return robot_interfaces::msg::TaskState::ASSIGNED;
    case TaskState::RUNNING:
      return robot_interfaces::msg::TaskState::RUNNING;
    case TaskState::WAITING_CONFIRMATION:
      return robot_interfaces::msg::TaskState::WAITING_CONFIRMATION;
    case TaskState::WAITING_RESOURCE:
      return robot_interfaces::msg::TaskState::WAITING_RESOURCE;
    case TaskState::PAUSED:
      return robot_interfaces::msg::TaskState::PAUSED;
    case TaskState::RESUMING:
      return robot_interfaces::msg::TaskState::RESUMING;
    case TaskState::SUCCEEDED:
      return robot_interfaces::msg::TaskState::SUCCEEDED;
    case TaskState::FAILED:
      return robot_interfaces::msg::TaskState::FAILED;
    case TaskState::CANCELED:
      return robot_interfaces::msg::TaskState::CANCELED;
  }
  return robot_interfaces::msg::TaskState::FAILED;
}

uint8_t toInterfaceRobotState(RobotState state)
{
  switch (state) {
    case RobotState::IDLE:
      return robot_interfaces::msg::RobotState::IDLE;
    case RobotState::ASSIGNED:
      return robot_interfaces::msg::RobotState::ASSIGNED;
    case RobotState::EXECUTING:
      return robot_interfaces::msg::RobotState::EXECUTING;
    case RobotState::WAITING_CONFIRMATION:
      return robot_interfaces::msg::RobotState::WAITING_CONFIRMATION;
    case RobotState::WAITING_RESOURCE:
      return robot_interfaces::msg::RobotState::WAITING_RESOURCE;
    case RobotState::RETURNING_HOME:
      return robot_interfaces::msg::RobotState::RETURNING_HOME;
    case RobotState::PAUSED:
      return robot_interfaces::msg::RobotState::PAUSED;
    case RobotState::ESTOP:
      return robot_interfaces::msg::RobotState::ESTOP;
    case RobotState::ERROR:
      return robot_interfaces::msg::RobotState::ERROR;
  }
  return robot_interfaces::msg::RobotState::ERROR;
}

uint8_t toInterfaceResourceType(PointKind kind)
{
  switch (kind) {
    case PointKind::WAITING_AREA:
      return robot_interfaces::msg::ResourceLock::TYPE_WAITING_AREA;
    case PointKind::PICKUP:
      return robot_interfaces::msg::ResourceLock::TYPE_PICKUP;
    case PointKind::DELIVERY:
      return robot_interfaces::msg::ResourceLock::TYPE_DELIVERY;
    case PointKind::INSPECTION:
      return robot_interfaces::msg::ResourceLock::TYPE_INSPECTION;
  }
  return robot_interfaces::msg::ResourceLock::TYPE_INSPECTION;
}

uint8_t toInterfacePointKind(PointKind kind)
{
  switch (kind) {
    case PointKind::WAITING_AREA:
      return robot_interfaces::msg::TaskPoint::KIND_WAITING_AREA;
    case PointKind::PICKUP:
      return robot_interfaces::msg::TaskPoint::KIND_PICKUP;
    case PointKind::DELIVERY:
      return robot_interfaces::msg::TaskPoint::KIND_DELIVERY;
    case PointKind::INSPECTION:
      return robot_interfaces::msg::TaskPoint::KIND_INSPECTION;
  }
  return robot_interfaces::msg::TaskPoint::KIND_INSPECTION;
}

std::optional<PointKind> fromInterfacePointKind(uint8_t kind)
{
  switch (kind) {
    case robot_interfaces::msg::TaskPoint::KIND_WAITING_AREA:
      return PointKind::WAITING_AREA;
    case robot_interfaces::msg::TaskPoint::KIND_PICKUP:
      return PointKind::PICKUP;
    case robot_interfaces::msg::TaskPoint::KIND_DELIVERY:
      return PointKind::DELIVERY;
    case robot_interfaces::msg::TaskPoint::KIND_INSPECTION:
      return PointKind::INSPECTION;
    default:
      return std::nullopt;
  }
}

bool isTerminalTaskState(TaskState state)
{
  return state == TaskState::SUCCEEDED ||
         state == TaskState::FAILED ||
         state == TaskState::CANCELED;
}

const char * temporaryPointPrefix(PointKind kind)
{
  switch (kind) {
    case PointKind::PICKUP:
      return "RVIZ_PICKUP_";
    case PointKind::DELIVERY:
      return "RVIZ_DELIVERY_";
    case PointKind::INSPECTION:
      return "RVIZ_INSPECTION_";
    case PointKind::WAITING_AREA:
      return "RVIZ_WAIT_";
  }
  return "RVIZ_POINT_";
}

std::string defaultTemporaryLabel(PointKind kind)
{
  switch (kind) {
    case PointKind::PICKUP:
      return "rviz pickup";
    case PointKind::DELIVERY:
      return "rviz delivery";
    case PointKind::INSPECTION:
      return "rviz inspection";
    case PointKind::WAITING_AREA:
      return "rviz waiting area";
  }
  return "rviz point";
}

bool isMapFrame(const std::string & frame_id)
{
  return frame_id.empty() || frame_id == "map" || frame_id == "/map";
}

ConfirmationResult fromInterfaceConfirmation(uint8_t result)
{
  if (result == robot_interfaces::srv::ConfirmTaskStep::Request::RESULT_ABNORMAL) {
    return ConfirmationResult::ABNORMAL;
  }
  if (result == robot_interfaces::srv::ConfirmTaskStep::Request::RESULT_REJECT) {
    return ConfirmationResult::REJECT;
  }
  return ConfirmationResult::OK;
}

uint8_t businessResultFromString(const std::string & result)
{
  if (result == "OK") {
    return robot_interfaces::msg::Task::RESULT_OK;
  }
  if (result == "ABNORMAL") {
    return robot_interfaces::msg::Task::RESULT_ABNORMAL;
  }
  if (result == "REJECT") {
    return robot_interfaces::msg::Task::RESULT_REJECT;
  }
  return robot_interfaces::msg::Task::RESULT_NONE;
}

std::string trim(std::string value)
{
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), [](unsigned char c) {
    return !std::isspace(c);
  }));
  value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char c) {
    return !std::isspace(c);
  }).base(), value.end());
  return value;
}

std::optional<std::string> normalizePreferredRobotId(const std::string & preferred_robot_id)
{
  const auto normalized = trim(preferred_robot_id);
  if (normalized.empty() || normalized == "auto") {
    return std::string{"auto"};
  }
  if (normalized == "mecanum" || normalized == "ackermann" ||
      normalized == "robot1" || normalized == "robot2") {
    return normalized;
  }
  return std::nullopt;
}

std::string taskSummary(const std::vector<robot_interfaces::msg::Task> & tasks)
{
  std::ostringstream out;
  out << "[";
  for (std::size_t i = 0; i < tasks.size(); ++i) {
    const auto & task = tasks[i];
    if (i != 0) {
      out << ",";
    }
    out << "{\"id\":\"" << task.task_id << "\",\"type\":" << int(task.task_type)
        << ",\"state\":" << int(task.state.state) << ",\"robot\":\""
        << task.assigned_robot_id << "\",\"step\":" << task.current_step_index
        << "}";
  }
  out << "]";
  return out.str();
}

std::string robotSummary(const std::vector<robot_interfaces::msg::RobotState> & robots)
{
  std::ostringstream out;
  out << "[";
  for (std::size_t i = 0; i < robots.size(); ++i) {
    const auto & robot = robots[i];
    if (i != 0) {
      out << ",";
    }
    out << "{\"id\":\"" << robot.robot_id << "\",\"state\":"
        << int(robot.state) << ",\"task\":\"" << robot.current_task_id << "\"}";
  }
  out << "]";
  return out.str();
}

std::string lockSummary(const std::vector<robot_interfaces::msg::ResourceLock> & locks)
{
  std::ostringstream out;
  out << "[";
  for (std::size_t i = 0; i < locks.size(); ++i) {
    const auto & lock = locks[i];
    if (i != 0) {
      out << ",";
    }
    out << "{\"resource\":\"" << lock.resource_id << "\",\"status\":"
        << int(lock.status) << ",\"task\":\"" << lock.locked_by_task_id << "\"}";
  }
  out << "]";
  return out.str();
}

std::string expandUserPath(const std::string & path)
{
  if (path == "~") {
    const char * home = std::getenv("HOME");
    return home == nullptr ? path : std::string(home);
  }
  if (path.rfind("~/", 0) == 0) {
    const char * home = std::getenv("HOME");
    return home == nullptr ? path : std::string(home) + path.substr(1);
  }
  return path;
}

std::string jsonEscape(const std::string & value)
{
  std::ostringstream out;
  for (const char c : value) {
    switch (c) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << c;
        break;
    }
  }
  return out.str();
}

TaskType taskTypeFromPersisted(const std::string & value)
{
  try {
    return taskTypeFromString(value);
  } catch (const std::exception &) {
    return TaskType::DELIVERY;
  }
}

}  // namespace

RobotDispatchNode::RobotDispatchNode(const rclcpp::NodeOptions & options)
: Node("robot_dispatch", options)
{
  std::string default_task_points;
  try {
    default_task_points =
      ament_index_cpp::get_package_share_directory("robot_dispatch") +
      "/config/task_points.yaml";
  } catch (const std::exception &) {
    default_task_points = "src/robot_dispatch/config/task_points.yaml";
  }

  task_points_file_ =
    declare_parameter<std::string>("task_points_file", default_task_points);
  enable_mission_execution_ =
    declare_parameter<bool>("enable_mission_execution", true);
  enforce_real_system_gates_ =
    declare_parameter<bool>("enforce_real_system_gates", false);
  safety_state_file_ = expandUserPath(
    declare_parameter<std::string>(
      "safety_state_file", "~/.ros/robot_dispatch_safety_state.json"));
  const auto marker_topic =
    declare_parameter<std::string>("marker_topic", kMarkerTopic);

  try {
    task_points_ = loadTaskPointsFromYaml(task_points_file_);
    task_points_valid_ = true;
  } catch (const std::exception & exc) {
    task_points_valid_ = false;
    safety_state_file_warning_ = true;
    safety_state_file_warning_message_ =
      std::string("task point config unavailable: ") + exc.what();
    RCLCPP_ERROR(get_logger(), "%s", safety_state_file_warning_message_.c_str());
  }
  map_version_ = declare_parameter<std::string>("map_version", task_points_.map_version);
  map_bundle_hash_ = declare_parameter<std::string>("map_bundle_hash", "");

  auto configured_robot_ids =
    declare_parameter<std::vector<std::string>>("robot_ids", std::vector<std::string>{});
  if (task_points_.robots.empty()) {
    if (configured_robot_ids.empty()) {
      configured_robot_ids = enforce_real_system_gates_
        ? std::vector<std::string>{"mecanum", "ackermann"}
        : std::vector<std::string>{"robot1", "robot2"};
    }
    for (const auto & robot_id : configured_robot_ids) {
      const auto waiting_area = robot_id == "ackermann" || robot_id == "robot2" ? "W2" : "W1";
      core_.registerRobot(robot_id, waiting_area);
      robot_health_[robot_id].robot_id = robot_id;
      robot_health_[robot_id].robot_namespace = "/" + robot_id;
    }
  } else {
    for (const auto & [robot_id, robot] : task_points_.robots) {
      core_.registerRobot(robot_id, robot.waiting_area_id);
      robot_health_[robot_id].robot_id = robot_id;
      robot_health_[robot_id].robot_namespace = "/" + robot_id;
    }
  }

  for (const auto & [robot_id, _] : robot_health_) {
    const auto param_name = robot_id + "_execute_mission_action";
    const auto default_action = "/" + robot_id + "/execute_mission";
    const auto action_name = declare_parameter<std::string>(param_name, default_action);
    mission_clients_[robot_id] =
      rclcpp_action::create_client<ExecuteMission>(this, action_name);
  }

  tasks_pub_ = create_publisher<std_msgs::msg::String>(
    "/robot_dispatch/tasks", rclcpp::QoS(1).transient_local());
  robot_states_pub_ = create_publisher<std_msgs::msg::String>(
    "/robot_dispatch/robot_states", rclcpp::QoS(1).transient_local());
  resource_locks_pub_ = create_publisher<std_msgs::msg::String>(
    "/robot_dispatch/resource_locks", rclcpp::QoS(1).transient_local());
  system_state_pub_ = create_publisher<robot_interfaces::msg::SystemState>(
    "/robot_dispatch/system_state", rclcpp::QoS(1).transient_local());
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    marker_topic, rclcpp::QoS(1).transient_local());

  for (const auto & [robot_id, _] : robot_health_) {
    const auto ns = "/" + robot_id;
    heartbeat_subs_.push_back(create_subscription<robot_interfaces::msg::RobotHeartbeat>(
      ns + "/heartbeat", rclcpp::QoS(5).best_effort(),
      std::bind(&RobotDispatchNode::onHeartbeat, this, std::placeholders::_1)));
    health_subs_.push_back(create_subscription<robot_interfaces::msg::RobotHealth>(
      ns + "/health", rclcpp::QoS(1).best_effort(),
      std::bind(&RobotDispatchNode::onHealth, this, std::placeholders::_1)));
    dispatch_lease_pubs_[robot_id] =
      create_publisher<robot_interfaces::msg::DispatchLease>(
        ns + "/dispatch_lease", rclcpp::QoS(1).best_effort());
  }

  create_task_srv_ = create_service<robot_interfaces::srv::CreateTask>(
    "/robot_dispatch/create_task",
    std::bind(&RobotDispatchNode::handleCreateTask, this, std::placeholders::_1, std::placeholders::_2));
  cancel_task_srv_ = create_service<robot_interfaces::srv::CancelTask>(
    "/robot_dispatch/cancel_task",
    std::bind(&RobotDispatchNode::handleCancelTask, this, std::placeholders::_1, std::placeholders::_2));
  pause_task_srv_ = create_service<robot_interfaces::srv::PauseTask>(
    "/robot_dispatch/pause_task",
    std::bind(&RobotDispatchNode::handlePauseTask, this, std::placeholders::_1, std::placeholders::_2));
  resume_task_srv_ = create_service<robot_interfaces::srv::ResumeTask>(
    "/robot_dispatch/resume_task",
    std::bind(&RobotDispatchNode::handleResumeTask, this, std::placeholders::_1, std::placeholders::_2));
  confirm_task_step_srv_ = create_service<robot_interfaces::srv::ConfirmTaskStep>(
    "/robot_dispatch/confirm_task_step",
    std::bind(&RobotDispatchNode::handleConfirmTaskStep, this, std::placeholders::_1, std::placeholders::_2));
  emergency_stop_srv_ = create_service<robot_interfaces::srv::EmergencyStop>(
    "/robot_dispatch/emergency_stop",
    std::bind(&RobotDispatchNode::handleEmergencyStop, this, std::placeholders::_1, std::placeholders::_2));
  enable_system_srv_ = create_service<robot_interfaces::srv::EnableSystem>(
    "/robot_dispatch/enable_system",
    std::bind(&RobotDispatchNode::handleEnableSystem, this, std::placeholders::_1, std::placeholders::_2));
  recover_system_srv_ = create_service<robot_interfaces::srv::RecoverSystem>(
    "/robot_dispatch/recover_system",
    std::bind(&RobotDispatchNode::handleRecoverSystem, this, std::placeholders::_1, std::placeholders::_2));
  get_state_srv_ = create_service<robot_interfaces::srv::GetDispatchState>(
    "/robot_dispatch/get_state",
    std::bind(&RobotDispatchNode::handleGetDispatchState, this, std::placeholders::_1, std::placeholders::_2));
  add_task_point_srv_ = create_service<robot_interfaces::srv::AddTaskPoint>(
    "/robot_dispatch/add_task_point",
    std::bind(&RobotDispatchNode::handleAddTaskPoint, this, std::placeholders::_1, std::placeholders::_2));
  clear_temporary_points_srv_ =
    create_service<robot_interfaces::srv::ClearTemporaryTaskPoints>(
    "/robot_dispatch/clear_temporary_task_points",
    std::bind(
      &RobotDispatchNode::handleClearTemporaryTaskPoints, this,
      std::placeholders::_1, std::placeholders::_2));
  get_task_points_srv_ = create_service<robot_interfaces::srv::GetTaskPoints>(
    "/robot_dispatch/get_task_points",
    std::bind(&RobotDispatchNode::handleGetTaskPoints, this, std::placeholders::_1, std::placeholders::_2));

  if (enforce_real_system_gates_) {
    system_mode_ = SystemMode::WAITING_ROBOTS;
    loadSafetyStateLocked();
  } else {
    system_mode_ = SystemMode::READY;
  }
  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(500), [this]() {publishState();});
  health_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    [this]() {
      std::lock_guard<std::mutex> lock(mutex_);
      evaluateSystemStateLocked();
      publishStateLocked();
    });
  dispatch_lease_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    [this]() {
      std::lock_guard<std::mutex> lock(mutex_);
      publishDispatchLeasesLocked();
    });
  publishState();
  RCLCPP_INFO(
    get_logger(), "robot_dispatch ready with task points %s map_version=%s real_gates=%s",
    task_points_file_.c_str(), map_version_.c_str(), enforce_real_system_gates_ ? "true" : "false");
}

void RobotDispatchNode::handleCreateTask(
  const std::shared_ptr<robot_interfaces::srv::CreateTask::Request> request,
  std::shared_ptr<robot_interfaces::srv::CreateTask::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::string gate_reason;
  if (!taskCreationAllowedLocked(&gate_reason)) {
    response->accepted = false;
    response->message = gate_reason;
    return;
  }
  const auto point_ids = pointIdsFromSteps(request->steps);
  std::string error;
  if (point_ids.empty() || !validatePointIds(point_ids, &error)) {
    response->accepted = false;
    response->message = point_ids.empty() ? "task requires at least one point" : error;
    return;
  }

  const auto preferred_robot_id = normalizePreferredRobotId(request->preferred_robot_id);
  if (!preferred_robot_id.has_value()) {
    response->accepted = false;
    response->message = "invalid preferred_robot_id: " + request->preferred_robot_id;
    return;
  }
  if (*preferred_robot_id != "auto" && core_.robot(*preferred_robot_id) == nullptr) {
    response->accepted = false;
    response->message = "unknown preferred_robot_id: " + *preferred_robot_id;
    return;
  }

  const auto task_type = fromInterfaceTaskType(request->task_type);
  const int parent_id =
    request->parent_task_id.empty() ? 0 : parseTaskId(request->parent_task_id);
  const int task_id = core_.createTask(
    task_type, point_ids, parent_id, request->excluded_robot_id,
    task_type == TaskType::RECHECK ? point_ids.front() : "",
    *preferred_robot_id);
  response->accepted = true;
  response->task_id = formatTaskId(task_id);
  dispatchAndSendLocked();
  response->task = taskMessage(*core_.task(task_id));
  response->message = "created";
  publishStateLocked();
}

void RobotDispatchNode::handleCancelTask(
  const std::shared_ptr<robot_interfaces::srv::CancelTask::Request> request,
  std::shared_ptr<robot_interfaces::srv::CancelTask::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const int task_id = parseTaskId(request->task_id);
  const auto before = core_.task(task_id);
  const std::string assigned_robot = before ? before->assigned_robot_id : "";
  response->accepted = core_.cancelTask(task_id);
  if (response->accepted) {
    cancelActiveTaskMissionLocked(task_id);
    if (!assigned_robot.empty()) {
      startReturnHomeLocked(assigned_robot);
    }
    response->task = taskMessage(*core_.task(task_id));
    response->message = "canceled; robot returning to waiting area";
  } else {
    response->message = "cancel rejected";
  }
  publishStateLocked();
}

void RobotDispatchNode::handlePauseTask(
  const std::shared_ptr<robot_interfaces::srv::PauseTask::Request> request,
  std::shared_ptr<robot_interfaces::srv::PauseTask::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const int task_id = parseTaskId(request->task_id);
  response->accepted = core_.pauseTask(task_id);
  if (response->accepted) {
    cancelActiveTaskMissionLocked(task_id);
    response->task = taskMessage(*core_.task(task_id));
    response->message = "paused";
  } else {
    response->message = "pause rejected";
  }
  publishStateLocked();
}

void RobotDispatchNode::handleResumeTask(
  const std::shared_ptr<robot_interfaces::srv::ResumeTask::Request> request,
  std::shared_ptr<robot_interfaces::srv::ResumeTask::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const int task_id = parseTaskId(request->task_id);
  response->accepted = core_.resumeTask(task_id);
  if (response->accepted) {
    const auto task = core_.task(task_id);
    if (task != nullptr) {
      if (task->state == TaskState::RESUMING) {
        sendCurrentStepLocked(*task);
      }
      response->task = taskMessage(*task);
      if (task->state == TaskState::WAITING_RESOURCE) {
        response->message = "resumed; waiting for resource";
      } else {
        response->message = "resumed";
      }
    }
  } else {
    response->message = "resume rejected";
  }
  publishStateLocked();
}

void RobotDispatchNode::handleConfirmTaskStep(
  const std::shared_ptr<robot_interfaces::srv::ConfirmTaskStep::Request> request,
  std::shared_ptr<robot_interfaces::srv::ConfirmTaskStep::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const int task_id = parseTaskId(request->task_id);
  response->accepted = core_.confirmTaskStep(
    task_id, fromInterfaceConfirmation(request->result));
  if (!response->accepted) {
    response->message = "confirmation rejected";
    publishStateLocked();
    return;
  }

  const auto task = core_.task(task_id);
  if (task != nullptr) {
    if (task->state == TaskState::RUNNING || task->state == TaskState::RESUMING) {
      sendCurrentStepLocked(*task);
    } else if (task->state == TaskState::SUCCEEDED && !task->assigned_robot_id.empty()) {
      startReturnHomeLocked(task->assigned_robot_id);
    }
    response->task = taskMessage(*task);
  }

  for (const auto & candidate : core_.tasks()) {
    if (candidate.parent_task_id == task_id && candidate.type == TaskType::RECHECK) {
      response->derived_task_id = formatTaskId(candidate.id);
    }
  }
  dispatchAndSendLocked();
  response->message = "confirmed";
  publishStateLocked();
}

void RobotDispatchNode::handleEmergencyStop(
  const std::shared_ptr<robot_interfaces::srv::EmergencyStop::Request> request,
  std::shared_ptr<robot_interfaces::srv::EmergencyStop::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (request->active) {
    if (enforce_real_system_gates_) {
      triggerSystemStopLocked(SystemMode::ESTOPPED, "manual_estop");
    } else {
      core_.emergencyStop();
    }
    cancelAllActiveMissionsLocked();
  }
  response->accepted = true;
  fillState(&response->affected_tasks, &response->robot_states, nullptr);
  response->message = request->active
    ? "emergency stop active"
    : "emergency stop inactive request ignored; use recover_system";
  publishStateLocked();
}

void RobotDispatchNode::handleEnableSystem(
  const std::shared_ptr<robot_interfaces::srv::EnableSystem::Request> request,
  std::shared_ptr<robot_interfaces::srv::EnableSystem::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  evaluateSystemStateLocked();
  std::string reason;
  if (!request->operator_confirmed) {
    response->accepted = false;
    response->message = "operator confirmation required";
  } else if (!enforce_real_system_gates_) {
    system_mode_ = SystemMode::READY;
    response->accepted = true;
    response->message = "system enabled";
  } else if (system_mode_ != SystemMode::STANDBY) {
    response->accepted = false;
    response->message = "system can only be enabled from STANDBY";
  } else if (!healthStableForEnableLocked()) {
    response->accepted = false;
    response->message = "robot health has not been stable for 5 seconds";
  } else if (!allRobotsHealthyLocked(&reason)) {
    response->accepted = false;
    response->message = reason;
  } else {
    system_mode_ = SystemMode::READY;
    response->accepted = true;
    response->message = "system enabled";
  }
  response->system_state = systemStateMessageLocked();
  publishStateLocked();
}

void RobotDispatchNode::handleRecoverSystem(
  const std::shared_ptr<robot_interfaces::srv::RecoverSystem::Request> request,
  std::shared_ptr<robot_interfaces::srv::RecoverSystem::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  evaluateSystemStateLocked();
  std::string reason;
  if (!request->operator_confirmed) {
    response->accepted = false;
    response->message = "operator confirmation required";
  } else if (system_mode_ != SystemMode::ESTOPPED && system_mode_ != SystemMode::INTERLOCKED) {
    response->accepted = false;
    response->message = "system is not ESTOPPED or INTERLOCKED";
  } else if (!healthStableForEnableLocked()) {
    response->accepted = false;
    response->message = "robot health has not been stable for 5 seconds";
  } else if (!allRobotsHealthyLocked(&reason)) {
    response->accepted = false;
    response->message = reason;
  } else {
    core_.clearResourceLocks();
    system_mode_ = SystemMode::READY;
    clearPersistedSafetyStateLocked();
    response->accepted = true;
    response->message = "system recovered";
  }
  response->system_state = systemStateMessageLocked();
  publishStateLocked();
}

void RobotDispatchNode::handleGetDispatchState(
  const std::shared_ptr<robot_interfaces::srv::GetDispatchState::Request>,
  std::shared_ptr<robot_interfaces::srv::GetDispatchState::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  fillState(&response->tasks, &response->robot_states, &response->resource_locks);
  response->system_state = systemStateMessageLocked();
  response->message = "ok";
}

void RobotDispatchNode::handleAddTaskPoint(
  const std::shared_ptr<robot_interfaces::srv::AddTaskPoint::Request> request,
  std::shared_ptr<robot_interfaces::srv::AddTaskPoint::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto kind = fromInterfacePointKind(request->kind);
  if (!kind.has_value() || *kind == PointKind::WAITING_AREA) {
    response->accepted = false;
    response->message = "rviz temporary point kind must be pickup, delivery, or inspection";
    return;
  }
  if (!isMapFrame(request->pose.header.frame_id)) {
    response->accepted = false;
    response->message = "rviz temporary task points must use map frame";
    return;
  }
  const double x = request->pose.pose.position.x;
  const double y = request->pose.pose.position.y;
  if (!std::isfinite(x) || !std::isfinite(y)) {
    response->accepted = false;
    response->message = "rviz temporary task point has invalid coordinates";
    return;
  }

  TaskPoint point;
  point.id = nextTemporaryPointId(*kind);
  point.kind = *kind;
  point.label = request->label.empty() ? defaultTemporaryLabel(*kind) : request->label;
  point.pose.x = x;
  point.pose.y = y;
  point.pose.yaw = 0.0;
  point.temporary = true;
  task_points_.points[point.id] = point;

  response->accepted = true;
  response->point = taskPointMessage(task_points_.points.at(point.id));
  response->message = "added";
  publishStateLocked();
}

void RobotDispatchNode::handleClearTemporaryTaskPoints(
  const std::shared_ptr<robot_interfaces::srv::ClearTemporaryTaskPoints::Request>,
  std::shared_ptr<robot_interfaces::srv::ClearTemporaryTaskPoints::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto blocked = activeTemporaryPointReferencesLocked();
  std::sort(blocked.begin(), blocked.end());
  blocked.erase(std::unique(blocked.begin(), blocked.end()), blocked.end());
  if (!blocked.empty()) {
    response->accepted = false;
    response->blocked_point_ids = blocked;
    response->message = "temporary task points are referenced by active tasks";
    return;
  }

  uint32_t cleared = 0;
  for (auto it = task_points_.points.begin(); it != task_points_.points.end(); ) {
    if (it->second.temporary) {
      it = task_points_.points.erase(it);
      ++cleared;
    } else {
      ++it;
    }
  }
  response->accepted = true;
  response->cleared_count = cleared;
  response->message = "cleared";
  publishStateLocked();
}

void RobotDispatchNode::handleGetTaskPoints(
  const std::shared_ptr<robot_interfaces::srv::GetTaskPoints::Request>,
  std::shared_ptr<robot_interfaces::srv::GetTaskPoints::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & [_, point] : task_points_.points) {
    response->points.push_back(taskPointMessage(point));
  }
  response->message = "ok";
}

bool RobotDispatchNode::validatePointIds(
  const std::vector<std::string> & point_ids,
  std::string * error) const
{
  for (const auto & point_id : point_ids) {
    if (task_points_.points.count(point_id) == 0) {
      if (error != nullptr) {
        *error = "unknown task point: " + point_id;
      }
      return false;
    }
  }
  return true;
}

std::vector<std::string> RobotDispatchNode::pointIdsFromSteps(
  const std::vector<robot_interfaces::msg::MissionStep> & steps) const
{
  std::vector<std::string> point_ids;
  for (const auto & step : steps) {
    if (!step.point_id.empty()) {
      point_ids.push_back(step.point_id);
    }
  }
  return point_ids;
}

robot_interfaces::msg::Task RobotDispatchNode::taskMessage(const TaskRecord & task) const
{
  robot_interfaces::msg::Task msg;
  msg.task_id = formatTaskId(task.id);
  msg.task_type = toInterfaceTaskType(task.type);
  msg.state.state = toInterfaceTaskState(task.state);
  msg.state.stamp = now();
  msg.state.reason = task.failure_reason;
  msg.assigned_robot_id = task.assigned_robot_id;
  msg.preferred_robot_id = task.preferred_robot_id;
  msg.updated_at = now();
  msg.current_step_index = static_cast<uint32_t>(task.current_step_index);
  msg.parent_task_id = task.parent_task_id == 0 ? "" : formatTaskId(task.parent_task_id);
  msg.excluded_robot_id = task.originating_robot_id;
  msg.business_result = businessResultFromString(task.business_result);
  msg.message = task.failure_reason;
  for (std::size_t i = 0; i < task.target_point_ids.size(); ++i) {
    msg.steps.push_back(missionStepForPoint(task, task.target_point_ids[i], i));
  }
  for (const auto & lock : core_.resourceLocks()) {
    for (const auto & holder : lock.holders) {
      if (holder.task_id == task.id) {
        msg.locked_resource_ids.push_back(lock.point_id);
      }
    }
  }
  return msg;
}

robot_interfaces::msg::TaskPoint RobotDispatchNode::taskPointMessage(const TaskPoint & point) const
{
  robot_interfaces::msg::TaskPoint msg;
  msg.point_id = point.id;
  msg.kind = toInterfacePointKind(point.kind);
  msg.label = point.label;
  msg.pose = poseStampedFromPoint(task_points_, point, now());
  msg.temporary = point.temporary;
  msg.stamp = now();
  return msg;
}

robot_interfaces::msg::RobotState RobotDispatchNode::robotStateMessage(
  const RobotRecord & robot) const
{
  robot_interfaces::msg::RobotState msg;
  msg.robot_id = robot.id;
  msg.robot_namespace = "/" + robot.id;
  msg.state = toInterfaceRobotState(robot.state);
  msg.current_task_id = robot.active_task_id == 0 ? "" : formatTaskId(robot.active_task_id);
  msg.stamp = now();
  // 优先用板端实时 map_pose(<=5s 新鲜); 否则回退到合成的等待区坐标位姿
  auto health_it = robot_health_.find(robot.id);
  if (health_it != robot_health_.end() && health_it->second.has_map_pose &&
      (now() - health_it->second.map_pose_stamp).seconds() <= 5.0) {
    msg.pose = health_it->second.map_pose;
    msg.current_point_id = robot.waiting_area_id;
  } else {
    auto point_it = task_points_.points.find(robot.waiting_area_id);
    if (point_it != task_points_.points.end()) {
      msg.pose = poseStampedFromPoint(task_points_, point_it->second, now());
      msg.current_point_id = robot.waiting_area_id;
    }
  }
  return msg;
}

robot_interfaces::msg::ResourceLock RobotDispatchNode::resourceLockMessage(
  const ResourceLock & lock) const
{
  robot_interfaces::msg::ResourceLock msg;
  msg.resource_id = lock.point_id;
  msg.point_id = lock.point_id;
  msg.status = lock.holders.size() > 1
    ? robot_interfaces::msg::ResourceLock::STATUS_SHARED_ABNORMAL
    : robot_interfaces::msg::ResourceLock::STATUS_LOCKED;
  auto point_it = task_points_.points.find(lock.point_id);
  if (point_it != task_points_.points.end()) {
    msg.resource_type = toInterfaceResourceType(point_it->second.kind);
    msg.pose = poseStampedFromPoint(task_points_, point_it->second, now());
  }
  if (!lock.holders.empty()) {
    msg.locked_by_task_id = formatTaskId(lock.holders.front().task_id);
    msg.locked_by_robot_id = lock.holders.front().robot_id;
  }
  for (const auto & holder : lock.holders) {
    msg.shared_task_ids.push_back(formatTaskId(holder.task_id));
  }
  msg.stamp = now();
  return msg;
}

robot_interfaces::msg::SystemState RobotDispatchNode::systemStateMessageLocked() const
{
  robot_interfaces::msg::SystemState msg;
  msg.header.stamp = now();
  msg.header.frame_id = "map";
  msg.state = toInterfaceSystemState(system_mode_);
  msg.task_creation_allowed = system_mode_ == SystemMode::READY;
  msg.requires_operator_action =
    system_mode_ == SystemMode::STANDBY ||
    system_mode_ == SystemMode::ESTOPPED ||
    system_mode_ == SystemMode::INTERLOCKED;
  msg.healthy_for_enable = !enforce_real_system_gates_ || healthStableForEnableLocked();
  msg.map_version = map_version_;
  msg.message = toSystemStateString(system_mode_);
  if (!task_points_valid_) {
    msg.warnings.push_back("real_task_points_invalid");
  }
  if (safety_state_file_warning_) {
    msg.warnings.push_back("safety_state_file_error");
  }
  std::string health_reason;
  if (enforce_real_system_gates_ && !allRobotsHealthyLocked(&health_reason)) {
    msg.warnings.push_back(health_reason);
  }
  return msg;
}

robot_interfaces::msg::MissionStep RobotDispatchNode::missionStepForPoint(
  const TaskRecord & task,
  const std::string & point_id,
  std::size_t sequence) const
{
  robot_interfaces::msg::MissionStep step;
  step.sequence = static_cast<uint32_t>(sequence);
  step.step_type = robot_interfaces::msg::MissionStep::STEP_NAVIGATE;
  step.step_id = formatTaskId(task.id) + "_" + point_id;
  step.point_id = point_id;
  step.resource_id = point_id;
  step.requires_confirmation = true;
  step.label = point_id;
  auto point_it = task_points_.points.find(point_id);
  if (point_it != task_points_.points.end()) {
    step.target_pose = poseStampedFromPoint(task_points_, point_it->second, now());
    step.label = point_it->second.label;
  }
  return step;
}

void RobotDispatchNode::fillState(
  std::vector<robot_interfaces::msg::Task> * tasks,
  std::vector<robot_interfaces::msg::RobotState> * robots,
  std::vector<robot_interfaces::msg::ResourceLock> * locks) const
{
  if (tasks != nullptr) {
    tasks->clear();
    for (const auto & task : core_.tasks()) {
      tasks->push_back(taskMessage(task));
    }
  }
  if (robots != nullptr) {
    robots->clear();
    for (const auto & robot : core_.robots()) {
      robots->push_back(robotStateMessage(robot));
    }
  }
  if (locks != nullptr) {
    locks->clear();
    for (const auto & lock : core_.resourceLocks()) {
      locks->push_back(resourceLockMessage(lock));
    }
  }
}

void RobotDispatchNode::dispatchAndSendLocked()
{
  std::string reason;
  if (!taskCreationAllowedLocked(&reason)) {
    return;
  }
  const auto decisions = core_.dispatchAll();
  for (const auto & decision : decisions) {
    if (decision.interrupted_return_home) {
      cancelReturnHomeGoalLocked(decision.robot_id);
    }
    const auto task = core_.task(decision.task_id);
    if (task != nullptr) {
      sendCurrentStepLocked(*task);
    }
  }
}

bool RobotDispatchNode::taskCreationAllowedLocked(std::string * reason) const
{
  if (!enforce_real_system_gates_) {
    return true;
  }
  if (system_mode_ != SystemMode::READY) {
    if (reason != nullptr) {
      *reason = std::string("system is not READY: ") + toSystemStateString(system_mode_);
    }
    return false;
  }
  if (!task_points_valid_) {
    if (reason != nullptr) {
      *reason = "real task points unavailable";
    }
    return false;
  }
  std::string health_reason;
  if (!allRobotsHealthyLocked(&health_reason)) {
    if (reason != nullptr) {
      *reason = health_reason;
    }
    return false;
  }
  return true;
}

void RobotDispatchNode::sendCurrentStepLocked(const TaskRecord & task)
{
  if (!enable_mission_execution_ || task.assigned_robot_id.empty() ||
      task.current_step_index >= task.target_point_ids.size()) {
    return;
  }
  auto client_it = mission_clients_.find(task.assigned_robot_id);
  if (client_it == mission_clients_.end() || !client_it->second->action_server_is_ready()) {
    RCLCPP_WARN(
      get_logger(), "mission executor for %s is not ready; task %s stays assigned",
      task.assigned_robot_id.c_str(), formatTaskId(task.id).c_str());
    return;
  }

  auto current_task = task;
  current_task.state = TaskState::RUNNING;
  const auto point_id = task.target_point_ids[task.current_step_index];
  ExecuteMission::Goal goal;
  goal.command.command_id = formatTaskId(task.id) + "_step_" +
    std::to_string(task.current_step_index);
  goal.command.task_id = formatTaskId(task.id);
  goal.command.task_type = toInterfaceTaskType(task.type);
  goal.command.assigned_robot_id = task.assigned_robot_id;
  goal.command.steps.push_back(
    missionStepForPoint(task, point_id, task.current_step_index));
  goal.command.parent_task_id =
    task.parent_task_id == 0 ? "" : formatTaskId(task.parent_task_id);
  goal.command.excluded_robot_id = task.originating_robot_id;

  core_.markTaskRunning(task.id);
  rclcpp_action::Client<ExecuteMission>::SendGoalOptions options;
  options.goal_response_callback =
    [this, task_id = task.id, robot_id = task.assigned_robot_id](
      GoalHandleExecuteMission::SharedPtr goal_handle)
    {
      if (!goal_handle) {
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      const auto task = core_.task(task_id);
      if (task == nullptr ||
          task->state == TaskState::PAUSED ||
          task->state == TaskState::CANCELED ||
          task->state == TaskState::FAILED) {
        auto client_it = mission_clients_.find(robot_id);
        if (client_it != mission_clients_.end()) {
          client_it->second->async_cancel_goal(goal_handle);
        }
        return;
      }
      active_task_goals_[task_id] = goal_handle;
      active_task_goal_robots_[task_id] = robot_id;
    };
  options.result_callback =
    [this, task_id = task.id](const GoalHandleExecuteMission::WrappedResult & result) {
      onMissionResult(task_id, result);
    };
  client_it->second->async_send_goal(goal, options);
}

void RobotDispatchNode::cancelActiveTaskMissionLocked(int task_id)
{
  auto goal_it = active_task_goals_.find(task_id);
  if (goal_it == active_task_goals_.end()) {
    return;
  }

  const auto robot_it = active_task_goal_robots_.find(task_id);
  const auto client_it = robot_it == active_task_goal_robots_.end()
    ? mission_clients_.end()
    : mission_clients_.find(robot_it->second);
  if (client_it != mission_clients_.end()) {
    client_it->second->async_cancel_goal(goal_it->second);
  }
  active_task_goals_.erase(goal_it);
  active_task_goal_robots_.erase(task_id);
}

void RobotDispatchNode::cancelReturnHomeGoalLocked(const std::string & robot_id)
{
  if (robot_id.empty()) {
    return;
  }
  return_home_cancel_requested_.insert(robot_id);
  auto goal_it = active_return_home_goals_.find(robot_id);
  if (goal_it == active_return_home_goals_.end()) {
    return;
  }
  const auto client_it = mission_clients_.find(robot_id);
  if (client_it != mission_clients_.end()) {
    client_it->second->async_cancel_goal(goal_it->second);
  }
  active_return_home_goals_.erase(goal_it);
}

void RobotDispatchNode::cancelAllActiveMissionsLocked()
{
  for (const auto & [task_id, goal_handle] : active_task_goals_) {
    const auto robot_it = active_task_goal_robots_.find(task_id);
    const auto client_it = robot_it == active_task_goal_robots_.end()
      ? mission_clients_.end()
      : mission_clients_.find(robot_it->second);
    if (client_it != mission_clients_.end()) {
      client_it->second->async_cancel_goal(goal_handle);
    }
  }
  active_task_goals_.clear();
  active_task_goal_robots_.clear();

  for (const auto & [robot_id, goal_handle] : active_return_home_goals_) {
    const auto client_it = mission_clients_.find(robot_id);
    if (client_it != mission_clients_.end()) {
      client_it->second->async_cancel_goal(goal_handle);
    }
  }
  active_return_home_goals_.clear();
  return_home_cancel_requested_.clear();
}

void RobotDispatchNode::startReturnHomeLocked(const std::string & robot_id)
{
  if (robot_id.empty()) {
    return;
  }
  core_.setRobotState(robot_id, RobotState::RETURNING_HOME);
  if (!sendReturnHomeLocked(robot_id) && !enable_mission_execution_) {
    core_.completeReturn(robot_id);
  }
}

bool RobotDispatchNode::sendReturnHomeLocked(const std::string & robot_id)
{
  if (!enable_mission_execution_) {
    return false;
  }
  auto robot = core_.robot(robot_id);
  if (robot == nullptr) {
    return false;
  }
  auto point_it = task_points_.points.find(robot->waiting_area_id);
  auto client_it = mission_clients_.find(robot_id);
  if (point_it == task_points_.points.end() ||
      client_it == mission_clients_.end() ||
      !client_it->second->action_server_is_ready()) {
    RCLCPP_WARN(
      get_logger(), "cannot return %s to waiting area; mission executor or waiting area unavailable",
      robot_id.c_str());
    return false;
  }

  TaskRecord return_task;
  return_task.id = 0;
  return_task.type = TaskType::DELIVERY;
  return_task.assigned_robot_id = robot_id;
  return_task.target_point_ids = {robot->waiting_area_id};
  ExecuteMission::Goal goal;
  goal.command.command_id = robot_id + "_return_home";
  goal.command.task_id = "return_home";
  goal.command.assigned_robot_id = robot_id;
  goal.command.steps.push_back(missionStepForPoint(return_task, robot->waiting_area_id, 0));
  goal.command.steps.back().step_type = robot_interfaces::msg::MissionStep::STEP_RETURN_HOME;
  rclcpp_action::Client<ExecuteMission>::SendGoalOptions options;
  options.goal_response_callback =
    [this, robot_id](GoalHandleExecuteMission::SharedPtr goal_handle) {
      if (!goal_handle) {
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      if (return_home_cancel_requested_.erase(robot_id) > 0) {
        active_return_home_goals_[robot_id] = goal_handle;
        auto client_it = mission_clients_.find(robot_id);
        if (client_it != mission_clients_.end()) {
          client_it->second->async_cancel_goal(goal_handle);
        }
        return;
      }
      active_return_home_goals_[robot_id] = goal_handle;
    };
  options.result_callback =
    [this, robot_id](const GoalHandleExecuteMission::WrappedResult & result) {
      onReturnHomeResult(robot_id, result);
    };
  client_it->second->async_send_goal(goal, options);
  return true;
}

void RobotDispatchNode::publishStateLocked()
{
  publishSummaryLocked();
  publishMarkersLocked();
}

void RobotDispatchNode::publishState()
{
  std::lock_guard<std::mutex> lock(mutex_);
  publishStateLocked();
}

void RobotDispatchNode::onHeartbeat(const robot_interfaces::msg::RobotHeartbeat::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto robot_id = msg->robot_id;
  if (robot_id.empty()) {
    robot_id = msg->robot_namespace;
    while (!robot_id.empty() && robot_id.front() == '/') {
      robot_id.erase(robot_id.begin());
    }
  }
  auto it = robot_health_.find(robot_id);
  if (it == robot_health_.end()) {
    return;
  }
  auto & health = it->second;
  health.has_heartbeat = true;
  health.last_heartbeat = now();
  health.robot_namespace = msg->robot_namespace.empty() ? "/" + robot_id : msg->robot_namespace;
  health.map_version = msg->map_version;
  health.mission_state = msg->mission_state;
  health.message = msg->message;
}

void RobotDispatchNode::onHealth(const robot_interfaces::msg::RobotHealth::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto robot_id = msg->robot_id;
  if (robot_id.empty()) {
    robot_id = msg->robot_namespace;
    while (!robot_id.empty() && robot_id.front() == '/') {
      robot_id.erase(robot_id.begin());
    }
  }
  auto it = robot_health_.find(robot_id);
  if (it == robot_health_.end()) {
    return;
  }
  auto & health = it->second;
  health.has_health = true;
  health.last_health = now();
  health.robot_namespace = msg->robot_namespace.empty() ? "/" + robot_id : msg->robot_namespace;
  health.map_version = msg->map_version;
  health.map_bundle_hash = msg->map_bundle_hash;
  health.has_scan = msg->scan_ok;
  health.has_odom = msg->odom_ok;
  health.scan_age_sec = msg->scan_age_sec;
  health.odom_age_sec = msg->odom_age_sec;
  health.tf_age_sec = msg->tf_age_sec;
  health.amcl_active = msg->amcl_active;
  health.nav2_active = msg->nav2_active;
  health.map_to_odom_ok = msg->map_to_odom_ok;
  health.mission_ready = msg->mission_ready;
  health.health_state = msg->health_state;
  health.health_reasons = msg->reasons;
  // 存板端实时地图位姿, 供态势图/RViz 显示 (ADR-0033: health 不门禁, 但 map_pose 仍用)
  health.map_pose = msg->map_pose;
  health.has_map_pose = true;
  health.map_pose_stamp = now();
}

void RobotDispatchNode::onScan(
  const std::string & robot_id,
  const sensor_msgs::msg::LaserScan::SharedPtr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = robot_health_.find(robot_id);
  if (it == robot_health_.end()) {
    return;
  }
  it->second.has_scan = true;
  it->second.last_scan = now();
}

void RobotDispatchNode::onOdom(
  const std::string & robot_id,
  const nav_msgs::msg::Odometry::SharedPtr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = robot_health_.find(robot_id);
  if (it == robot_health_.end()) {
    return;
  }
  it->second.has_odom = true;
  it->second.last_odom = now();
}

void RobotDispatchNode::pollLifecycleStates()
{
  if (!enforce_real_system_gates_) {
    return;
  }
  for (const auto & [key, client] : lifecycle_clients_) {
    if (!client || !client->service_is_ready()) {
      continue;
    }
    auto request = std::make_shared<GetLifecycleState::Request>();
    client->async_send_request(
      request,
      [this, key](rclcpp::Client<GetLifecycleState>::SharedFuture future) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto split = key.find(':');
        if (split == std::string::npos) {
          return;
        }
        auto robot_it = robot_health_.find(key.substr(0, split));
        if (robot_it == robot_health_.end()) {
          return;
        }
        const bool active =
          future.get()->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        if (key.substr(split + 1) == "amcl") {
          robot_it->second.amcl_active = active;
        } else {
          robot_it->second.nav2_active = active;
        }
      });
  }
}

void RobotDispatchNode::publishDispatchLeasesLocked()
{
  const bool run_allowed = system_mode_ == SystemMode::READY;
  ++dispatch_lease_seq_;
  for (const auto & [robot_id, pub] : dispatch_lease_pubs_) {
    if (!pub) {
      continue;
    }
    robot_interfaces::msg::DispatchLease lease;
    lease.header.stamp = now();
    lease.header.frame_id = "map";
    lease.robot_id = robot_id;
    lease.robot_namespace = "/" + robot_id;
    lease.lease_seq = dispatch_lease_seq_;
    lease.run_allowed = run_allowed;
    lease.system_state = toInterfaceSystemState(system_mode_);
    lease.reason = run_allowed ? "ready" : toSystemStateString(system_mode_);
    pub->publish(lease);
  }
}

bool RobotDispatchNode::robotHealthOkLocked(const std::string & robot_id, std::string * reason) const
{
  const auto it = robot_health_.find(robot_id);
  if (it == robot_health_.end()) {
    if (reason != nullptr) {
      *reason = "unknown robot: " + robot_id;
    }
    return false;
  }
  const auto & health = it->second;
  const auto current_time = now();
  if (!health.has_heartbeat || (current_time - health.last_heartbeat).seconds() > 3.0) {
    if (reason != nullptr) {
      *reason = robot_id + " heartbeat stale";
    }
    return false;
  }
  if (health.mission_state != robot_interfaces::msg::RobotHeartbeat::IDLE &&
      health.mission_state != robot_interfaces::msg::RobotHeartbeat::EXECUTING) {
    if (reason != nullptr) {
      *reason = robot_id + " mission state blocks READY";
    }
    return false;
  }
  if (health.map_version != map_version_) {
    if (reason != nullptr) {
      *reason = robot_id + " map_version mismatch";
    }
    return false;
  }
  if (!map_bundle_hash_.empty() && health.map_bundle_hash != map_bundle_hash_) {
    if (reason != nullptr) {
      *reason = robot_id + " map_bundle_hash mismatch";
    }
    return false;
  }
  if (!health.has_health || (current_time - health.last_health).seconds() > 3.0) {
    if (reason != nullptr) {
      *reason = robot_id + " health stale";
    }
    return false;
  }
  if (health.health_state != robot_interfaces::msg::RobotHealth::OK) {
    if (reason != nullptr) {
      std::ostringstream detail;
      for (std::size_t i = 0; i < health.health_reasons.size(); ++i) {
        if (i > 0) {
          detail << ", ";
        }
        detail << health.health_reasons[i];
      }
      *reason = robot_id + " health state blocks READY" +
        (detail.str().empty() ? std::string{} : ": " + detail.str());
    }
    return false;
  }
  if (!health.has_scan || health.scan_age_sec > 1.0F) {
    if (reason != nullptr) {
      *reason = robot_id + " scan stale";
    }
    return false;
  }
  if (!health.has_odom || health.odom_age_sec > 1.0F) {
    if (reason != nullptr) {
      *reason = robot_id + " odom stale";
    }
    return false;
  }
  if (!health.amcl_active) {
    if (reason != nullptr) {
      *reason = robot_id + " AMCL inactive";
    }
    return false;
  }
  if (!health.nav2_active) {
    if (reason != nullptr) {
      *reason = robot_id + " Nav2 controller inactive";
    }
    return false;
  }
  if (!health.map_to_odom_ok || health.tf_age_sec > 2.0F) {
    if (reason != nullptr) {
      *reason = robot_id + " map->odom TF stale";
    }
    return false;
  }
  if (!health.mission_ready) {
    if (reason != nullptr) {
      *reason = robot_id + " mission not ready";
    }
    return false;
  }
  return true;
}

bool RobotDispatchNode::allRobotsHealthyLocked(std::string * reason) const
{
  if (!task_points_valid_) {
    if (reason != nullptr) {
      *reason = "real task points unavailable";
    }
    return false;
  }
  if (map_version_.empty()) {
    if (reason != nullptr) {
      *reason = "map_version is empty";
    }
    return false;
  }
  for (const auto & [robot_id, _] : robot_health_) {
    if (!robotHealthOkLocked(robot_id, reason)) {
      return false;
    }
  }
  return !robot_health_.empty();
}

bool RobotDispatchNode::healthStableForEnableLocked() const
{
  if (!health_ok_since_.has_value()) {
    return false;
  }
  return (now() - *health_ok_since_).seconds() >= 5.0;
}

void RobotDispatchNode::evaluateSystemStateLocked()
{
  if (!enforce_real_system_gates_) {
    system_mode_ = SystemMode::READY;
    return;
  }
  std::string reason;
  const bool healthy = allRobotsHealthyLocked(&reason);
  if (healthy) {
    if (!health_ok_since_.has_value()) {
      health_ok_since_ = now();
    }
  } else {
    health_ok_since_.reset();
  }

  if (system_mode_ == SystemMode::ESTOPPED || system_mode_ == SystemMode::INTERLOCKED) {
    return;
  }
  if (system_mode_ == SystemMode::READY) {
    if (!healthy) {
      triggerSystemStopLocked(SystemMode::INTERLOCKED, "robot_health_interlock: " + reason);
    }
    return;
  }
  if (healthy && healthStableForEnableLocked()) {
    system_mode_ = SystemMode::STANDBY;
  } else {
    system_mode_ = SystemMode::WAITING_ROBOTS;
  }
}

void RobotDispatchNode::triggerSystemStopLocked(SystemMode mode, const std::string & reason)
{
  system_mode_ = mode;
  const auto failure_reason = mode == SystemMode::ESTOPPED ? "manual_estop" : reason;
  core_.failAllActiveTasks(failure_reason, false);
  cancelAllActiveMissionsLocked();
  persistSafetyStateLocked(failure_reason);
}

bool RobotDispatchNode::loadSafetyStateLocked()
{
  if (safety_state_file_.empty() || !std::filesystem::exists(safety_state_file_)) {
    return false;
  }
  try {
    const auto yaml = YAML::LoadFile(safety_state_file_);
    if (!yaml["version"] || yaml["version"].as<int>() != 1) {
      throw std::runtime_error("unsupported safety state file version");
    }
    const auto state = yaml["system_state"].as<std::string>();
    if (state == "ESTOPPED") {
      system_mode_ = SystemMode::ESTOPPED;
    } else if (state == "INTERLOCKED") {
      system_mode_ = SystemMode::INTERLOCKED;
    } else {
      return false;
    }

    std::vector<ResourceLock> restored_locks;
    if (yaml["frozen_locks"] && yaml["frozen_locks"].IsSequence()) {
      for (const auto lock_node : yaml["frozen_locks"]) {
        ResourceLock lock;
        lock.point_id = lock_node["point_id"] ? lock_node["point_id"].as<std::string>() : "";
        if (lock_node["holders"] && lock_node["holders"].IsSequence()) {
          for (const auto holder_node : lock_node["holders"]) {
            LockHolder holder;
            holder.task_id = holder_node["task_id"] ? holder_node["task_id"].as<int>() : 0;
            holder.task_type = holder_node["task_type"]
              ? taskTypeFromPersisted(holder_node["task_type"].as<std::string>())
              : TaskType::DELIVERY;
            holder.robot_id = holder_node["robot_id"] ? holder_node["robot_id"].as<std::string>() : "";
            holder.parent_task_id = holder_node["parent_task_id"]
              ? holder_node["parent_task_id"].as<int>()
              : 0;
            holder.abnormal_point_id = holder_node["abnormal_point_id"]
              ? holder_node["abnormal_point_id"].as<std::string>()
              : "";
            lock.holders.push_back(holder);
          }
        }
        restored_locks.push_back(lock);
      }
    }
    core_.setResourceLocks(restored_locks);
    RCLCPP_WARN(
      get_logger(), "restored persisted safety state %s from %s",
      state.c_str(), safety_state_file_.c_str());
    return true;
  } catch (const std::exception & exc) {
    safety_state_file_warning_ = true;
    safety_state_file_warning_message_ =
      std::string("safety state file ignored: ") + exc.what();
    system_mode_ = SystemMode::STANDBY;
    RCLCPP_ERROR(get_logger(), "%s", safety_state_file_warning_message_.c_str());
    return false;
  }
}

void RobotDispatchNode::persistSafetyStateLocked(const std::string & reason)
{
  if (safety_state_file_.empty()) {
    return;
  }
  try {
    const std::filesystem::path path{safety_state_file_};
    if (path.has_parent_path()) {
      std::filesystem::create_directories(path.parent_path());
    }
    const auto temp_path = path.string() + ".tmp";
    std::ofstream out(temp_path, std::ios::trunc);
    out << "{\n";
    out << "  \"version\": 1,\n";
    out << "  \"system_state\": \"" << toSystemStateString(system_mode_) << "\",\n";
    out << "  \"reason\": \"" << jsonEscape(reason) << "\",\n";
    out << "  \"triggered_at_sec\": " << now().seconds() << ",\n";
    out << "  \"requires_operator_recovery\": true,\n";
    out << "  \"frozen_locks\": [\n";
    const auto locks = core_.resourceLocks();
    for (std::size_t i = 0; i < locks.size(); ++i) {
      const auto & lock = locks[i];
      out << "    {\"point_id\": \"" << jsonEscape(lock.point_id) << "\", \"holders\": [";
      for (std::size_t j = 0; j < lock.holders.size(); ++j) {
        const auto & holder = lock.holders[j];
        if (j != 0) {
          out << ", ";
        }
        out << "{\"task_id\": " << holder.task_id
            << ", \"task_type\": \"" << toString(holder.task_type)
            << "\", \"robot_id\": \"" << jsonEscape(holder.robot_id)
            << "\", \"parent_task_id\": " << holder.parent_task_id
            << ", \"abnormal_point_id\": \"" << jsonEscape(holder.abnormal_point_id)
            << "\"}";
      }
      out << "]}";
      if (i + 1 < locks.size()) {
        out << ",";
      }
      out << "\n";
    }
    out << "  ]\n";
    out << "}\n";
    out.close();
    std::filesystem::rename(temp_path, path);
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(get_logger(), "failed to persist safety state: %s", exc.what());
  }
}

void RobotDispatchNode::clearPersistedSafetyStateLocked()
{
  if (safety_state_file_.empty()) {
    return;
  }
  std::error_code ec;
  std::filesystem::remove(safety_state_file_, ec);
  if (ec) {
    RCLCPP_WARN(
      get_logger(), "failed to remove safety state file %s: %s",
      safety_state_file_.c_str(), ec.message().c_str());
  }
}

uint8_t RobotDispatchNode::toInterfaceSystemState(SystemMode mode)
{
  switch (mode) {
    case SystemMode::WAITING_ROBOTS:
      return robot_interfaces::msg::SystemState::WAITING_ROBOTS;
    case SystemMode::STANDBY:
      return robot_interfaces::msg::SystemState::STANDBY;
    case SystemMode::READY:
      return robot_interfaces::msg::SystemState::READY;
    case SystemMode::ESTOPPED:
      return robot_interfaces::msg::SystemState::ESTOPPED;
    case SystemMode::INTERLOCKED:
      return robot_interfaces::msg::SystemState::INTERLOCKED;
  }
  return robot_interfaces::msg::SystemState::INTERLOCKED;
}

const char * RobotDispatchNode::toSystemStateString(SystemMode mode)
{
  switch (mode) {
    case SystemMode::WAITING_ROBOTS:
      return "WAITING_ROBOTS";
    case SystemMode::STANDBY:
      return "STANDBY";
    case SystemMode::READY:
      return "READY";
    case SystemMode::ESTOPPED:
      return "ESTOPPED";
    case SystemMode::INTERLOCKED:
      return "INTERLOCKED";
  }
  return "INTERLOCKED";
}

void RobotDispatchNode::publishSummaryLocked()
{
  std::vector<robot_interfaces::msg::Task> tasks;
  std::vector<robot_interfaces::msg::RobotState> robots;
  std::vector<robot_interfaces::msg::ResourceLock> locks;
  fillState(&tasks, &robots, &locks);

  std_msgs::msg::String task_msg;
  task_msg.data = taskSummary(tasks);
  tasks_pub_->publish(task_msg);

  std_msgs::msg::String robot_msg;
  robot_msg.data = robotSummary(robots);
  robot_states_pub_->publish(robot_msg);

  std_msgs::msg::String lock_msg;
  lock_msg.data = lockSummary(locks);
  resource_locks_pub_->publish(lock_msg);

  system_state_pub_->publish(systemStateMessageLocked());
}

void RobotDispatchNode::publishMarkersLocked()
{
  MarkerSceneState state;
  for (const auto & lock : core_.resourceLocks()) {
    state.locked_point_ids.insert(lock.point_id);
  }
  for (const auto & task : core_.tasks()) {
    if (task.current_step_index < task.target_point_ids.size() &&
        (task.state == TaskState::ASSIGNED ||
        task.state == TaskState::RUNNING ||
        task.state == TaskState::WAITING_CONFIRMATION ||
        task.state == TaskState::WAITING_RESOURCE ||
        task.state == TaskState::RESUMING)) {
      state.active_target_ids.insert(task.target_point_ids[task.current_step_index]);
    }
    if (task.type == TaskType::RECHECK && !task.abnormal_point_id.empty()) {
      state.abnormal_point_ids.insert(task.abnormal_point_id);
      state.recheck_target_ids.insert(task.abnormal_point_id);
    }
  }
  markers_pub_->publish(buildTaskPointMarkers(task_points_, state));
}

void RobotDispatchNode::onMissionResult(
  int task_id,
  const GoalHandleExecuteMission::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto task = core_.task(task_id);
  if (task == nullptr ||
      task->state == TaskState::CANCELED ||
      task->state == TaskState::PAUSED ||
      task->state == TaskState::FAILED) {
    publishStateLocked();
    return;
  }

  if (result.code == rclcpp_action::ResultCode::CANCELED) {
    publishStateLocked();
    return;
  }

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
      result.result && result.result->success) {
    active_task_goals_.erase(task_id);
    active_task_goal_robots_.erase(task_id);
    core_.setTaskWaitingConfirmation(task_id);
  } else {
    active_task_goals_.erase(task_id);
    active_task_goal_robots_.erase(task_id);
    core_.failTask(task_id, result.result ? result.result->message : "mission failed");
  }
  publishStateLocked();
}

void RobotDispatchNode::onReturnHomeResult(
  const std::string & robot_id,
  const GoalHandleExecuteMission::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (result.code == rclcpp_action::ResultCode::CANCELED) {
    active_return_home_goals_.erase(robot_id);
    return_home_cancel_requested_.erase(robot_id);
    publishStateLocked();
    return;
  }
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
      result.result && result.result->success) {
    active_return_home_goals_.erase(robot_id);
    core_.completeReturn(robot_id);
    dispatchAndSendLocked();
  } else {
    active_return_home_goals_.erase(robot_id);
    core_.setRobotState(robot_id, RobotState::ERROR);
  }
  publishStateLocked();
}

std::string RobotDispatchNode::nextTemporaryPointId(PointKind kind) const
{
  const std::string prefix = temporaryPointPrefix(kind);
  for (int index = 1; index < 100000; ++index) {
    const auto candidate = prefix + std::to_string(index);
    if (task_points_.points.count(candidate) == 0) {
      return candidate;
    }
  }
  throw std::runtime_error("unable to allocate rviz temporary task point id");
}

std::vector<std::string> RobotDispatchNode::activeTemporaryPointReferencesLocked() const
{
  std::vector<std::string> blocked;
  for (const auto & task : core_.tasks()) {
    if (isTerminalTaskState(task.state)) {
      continue;
    }
    for (const auto & point_id : task.target_point_ids) {
      const auto point_it = task_points_.points.find(point_id);
      if (point_it != task_points_.points.end() && point_it->second.temporary) {
        blocked.push_back(point_id);
      }
    }
  }
  return blocked;
}

int RobotDispatchNode::parseTaskId(const std::string & task_id)
{
  if (task_id.rfind("task_", 0) == 0) {
    return std::stoi(task_id.substr(5));
  }
  return std::stoi(task_id);
}

std::string RobotDispatchNode::formatTaskId(int task_id)
{
  return "task_" + std::to_string(task_id);
}

}  // namespace robot_dispatch
