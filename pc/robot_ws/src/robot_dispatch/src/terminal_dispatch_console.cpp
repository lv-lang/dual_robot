#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_interfaces/msg/task.hpp"
#include "robot_interfaces/msg/task_point.hpp"
#include "robot_interfaces/srv/cancel_task.hpp"
#include "robot_interfaces/srv/clear_temporary_task_points.hpp"
#include "robot_interfaces/srv/confirm_task_step.hpp"
#include "robot_interfaces/srv/create_task.hpp"
#include "robot_interfaces/srv/emergency_stop.hpp"
#include "robot_interfaces/srv/get_dispatch_state.hpp"
#include "robot_interfaces/srv/get_task_points.hpp"
#include "robot_interfaces/srv/pause_task.hpp"
#include "robot_interfaces/srv/resume_task.hpp"

#include "robot_dispatch/command_parser.hpp"
#include "robot_dispatch/task_points.hpp"

namespace robot_dispatch
{

namespace
{

using namespace std::chrono_literals;

uint8_t taskTypeFromConsole(const std::string & task_type)
{
  const auto normalized = normalize_token(task_type);
  if (normalized == "INSPECTION") {
    return robot_interfaces::msg::Task::TYPE_INSPECTION;
  }
  if (normalized == "RECHECK") {
    return robot_interfaces::msg::Task::TYPE_RECHECK;
  }
  return robot_interfaces::msg::Task::TYPE_DELIVERY;
}

uint8_t confirmationFromConsole(const std::string & confirmation)
{
  const auto normalized = normalize_token(confirmation);
  if (normalized == "ABNORMAL") {
    return robot_interfaces::srv::ConfirmTaskStep::Request::RESULT_ABNORMAL;
  }
  if (normalized == "REJECT") {
    return robot_interfaces::srv::ConfirmTaskStep::Request::RESULT_REJECT;
  }
  return robot_interfaces::srv::ConfirmTaskStep::Request::RESULT_OK;
}

PointKind pointKindFromInterface(uint8_t kind)
{
  if (kind == robot_interfaces::msg::TaskPoint::KIND_WAITING_AREA) {
    return PointKind::WAITING_AREA;
  }
  if (kind == robot_interfaces::msg::TaskPoint::KIND_PICKUP) {
    return PointKind::PICKUP;
  }
  if (kind == robot_interfaces::msg::TaskPoint::KIND_DELIVERY) {
    return PointKind::DELIVERY;
  }
  return PointKind::INSPECTION;
}

const char * pointKindName(uint8_t kind)
{
  if (kind == robot_interfaces::msg::TaskPoint::KIND_WAITING_AREA) {
    return "waiting_area";
  }
  if (kind == robot_interfaces::msg::TaskPoint::KIND_PICKUP) {
    return "pickup";
  }
  if (kind == robot_interfaces::msg::TaskPoint::KIND_DELIVERY) {
    return "delivery";
  }
  if (kind == robot_interfaces::msg::TaskPoint::KIND_INSPECTION) {
    return "inspection";
  }
  return "unknown";
}

TaskPointConfig taskPointConfigFromMessages(
  const std::vector<robot_interfaces::msg::TaskPoint> & points)
{
  TaskPointConfig config;
  for (const auto & msg : points) {
    TaskPoint point;
    point.id = msg.point_id;
    point.kind = pointKindFromInterface(msg.kind);
    point.label = msg.label;
    point.pose.x = msg.pose.pose.position.x;
    point.pose.y = msg.pose.pose.position.y;
    point.pose.yaw = 0.0;
    point.temporary = msg.temporary;
    config.points[point.id] = point;
    if (!msg.pose.header.frame_id.empty()) {
      config.frame_id = msg.pose.header.frame_id == "map" ? "/map" : msg.pose.header.frame_id;
    }
  }
  return config;
}

robot_interfaces::msg::MissionStep stepFromPoint(
  const TaskPointConfig & config,
  const std::string & point_id,
  std::size_t sequence)
{
  robot_interfaces::msg::MissionStep step;
  step.sequence = static_cast<uint32_t>(sequence);
  step.step_type = robot_interfaces::msg::MissionStep::STEP_NAVIGATE;
  step.step_id = point_id;
  step.point_id = point_id;
  step.resource_id = point_id;
  step.requires_confirmation = true;
  step.target_pose.header.frame_id = config.frame_id == "/map" ? "map" : config.frame_id;
  const auto point_it = config.points.find(point_id);
  if (point_it != config.points.end()) {
    step.label = point_it->second.label;
    step.target_pose.pose.position.x = point_it->second.pose.x;
    step.target_pose.pose.position.y = point_it->second.pose.y;
    const double yaw = point_it->second.pose.yaw;
    step.target_pose.pose.orientation.z = std::sin(yaw * 0.5);
    step.target_pose.pose.orientation.w = std::cos(yaw * 0.5);
  }
  return step;
}

template<typename ServiceT>
bool waitAndCall(
  rclcpp::Node & node,
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const std::shared_ptr<typename ServiceT::Request> & request,
  typename ServiceT::Response::SharedPtr * response)
{
  if (!client->wait_for_service(2s)) {
    std::cerr << "service not available: " << client->get_service_name() << "\n";
    return false;
  }
  auto future = client->async_send_request(request);
  const auto status = rclcpp::spin_until_future_complete(
    node.get_node_base_interface(), future, 5s);
  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    std::cerr << "service call timed out: " << client->get_service_name() << "\n";
    return false;
  }
  *response = future.get();
  return true;
}

}  // namespace

class TerminalDispatchConsole : public rclcpp::Node
{
public:
  TerminalDispatchConsole()
  : Node("terminal_dispatch_console")
  {
    std::string default_task_points;
    try {
      default_task_points =
        ament_index_cpp::get_package_share_directory("robot_dispatch") +
        "/config/task_points.yaml";
    } catch (const std::exception &) {
      default_task_points = "src/robot_dispatch/config/task_points.yaml";
    }
    const auto task_points_file =
      declare_parameter<std::string>("task_points_file", default_task_points);
    task_points_ = loadTaskPointsFromYaml(task_points_file);

    create_client_ = create_client<robot_interfaces::srv::CreateTask>(
      "/robot_dispatch/create_task");
    cancel_client_ = create_client<robot_interfaces::srv::CancelTask>(
      "/robot_dispatch/cancel_task");
    pause_client_ = create_client<robot_interfaces::srv::PauseTask>(
      "/robot_dispatch/pause_task");
    resume_client_ = create_client<robot_interfaces::srv::ResumeTask>(
      "/robot_dispatch/resume_task");
    confirm_client_ = create_client<robot_interfaces::srv::ConfirmTaskStep>(
      "/robot_dispatch/confirm_task_step");
    estop_client_ = create_client<robot_interfaces::srv::EmergencyStop>(
      "/robot_dispatch/emergency_stop");
    state_client_ = create_client<robot_interfaces::srv::GetDispatchState>(
      "/robot_dispatch/get_state");
    points_client_ = create_client<robot_interfaces::srv::GetTaskPoints>(
      "/robot_dispatch/get_task_points");
    clear_points_client_ =
      create_client<robot_interfaces::srv::ClearTemporaryTaskPoints>(
      "/robot_dispatch/clear_temporary_task_points");
  }

  void run()
  {
    std::cout << console_help_text();
    std::string line;
    while (rclcpp::ok()) {
      std::cout << "dispatch> " << std::flush;
      if (!std::getline(std::cin, line)) {
        break;
      }
      const auto parsed = parse_console_command(line);
      if (!parsed.ok) {
        std::cout << "error: " << parsed.error << "\n";
        continue;
      }
      if (parsed.spec.action == ConsoleAction::Quit) {
        break;
      }
      handle(parsed.spec);
    }
  }

private:
  void handle(const ServiceCallSpec & spec)
  {
    switch (spec.action) {
      case ConsoleAction::Help:
        std::cout << console_help_text();
        break;
      case ConsoleAction::CreateTask:
        createTask(spec);
        break;
      case ConsoleAction::ConfirmTaskStep:
        confirmTask(spec);
        break;
      case ConsoleAction::PauseTask:
        taskOnly<robot_interfaces::srv::PauseTask>(pause_client_, spec.task_id);
        break;
      case ConsoleAction::ResumeTask:
        taskOnly<robot_interfaces::srv::ResumeTask>(resume_client_, spec.task_id);
        break;
      case ConsoleAction::CancelTask:
        taskOnly<robot_interfaces::srv::CancelTask>(cancel_client_, spec.task_id);
        break;
      case ConsoleAction::EmergencyStop:
        emergencyStop();
        break;
      case ConsoleAction::PrintTasks:
      case ConsoleAction::PrintRobots:
      case ConsoleAction::PrintLocks:
        printState(spec.action);
        break;
      case ConsoleAction::PrintPoints:
        printPoints();
        break;
      case ConsoleAction::ClearTemporaryPoints:
        clearTemporaryPoints();
        break;
      case ConsoleAction::Quit:
        break;
    }
  }

  void createTask(const ServiceCallSpec & spec)
  {
    refreshTaskPoints();
    auto request = std::make_shared<robot_interfaces::srv::CreateTask::Request>();
    request->requester = "terminal";
    request->task_type = taskTypeFromConsole(spec.task_type);
    for (std::size_t i = 0; i < spec.point_ids.size(); ++i) {
      if (task_points_.points.count(spec.point_ids[i]) == 0) {
        std::cout << "unknown point: " << spec.point_ids[i]
                  << " (run 11 to list current task points)\n";
        return;
      }
      request->steps.push_back(stepFromPoint(task_points_, spec.point_ids[i], i));
    }
    robot_interfaces::srv::CreateTask::Response::SharedPtr response;
    if (waitAndCall<robot_interfaces::srv::CreateTask>(*this, create_client_, request, &response)) {
      std::cout << response->message << ": " << response->task_id << "\n";
    }
  }

  void confirmTask(const ServiceCallSpec & spec)
  {
    auto request = std::make_shared<robot_interfaces::srv::ConfirmTaskStep::Request>();
    request->requester = "terminal";
    request->task_id = spec.task_id;
    request->result = confirmationFromConsole(spec.confirmation);
    robot_interfaces::srv::ConfirmTaskStep::Response::SharedPtr response;
    if (waitAndCall<robot_interfaces::srv::ConfirmTaskStep>(
        *this, confirm_client_, request, &response))
    {
      std::cout << response->message;
      if (!response->derived_task_id.empty()) {
        std::cout << ", derived " << response->derived_task_id;
      }
      std::cout << "\n";
    }
  }

  template<typename ServiceT>
  void taskOnly(const typename rclcpp::Client<ServiceT>::SharedPtr & client, const std::string & task_id)
  {
    auto request = std::make_shared<typename ServiceT::Request>();
    request->requester = "terminal";
    request->task_id = task_id;
    typename ServiceT::Response::SharedPtr response;
    if (waitAndCall<ServiceT>(*this, client, request, &response)) {
      std::cout << response->message << "\n";
    }
  }

  void emergencyStop()
  {
    auto request = std::make_shared<robot_interfaces::srv::EmergencyStop::Request>();
    request->requester = "terminal";
    request->active = true;
    robot_interfaces::srv::EmergencyStop::Response::SharedPtr response;
    if (waitAndCall<robot_interfaces::srv::EmergencyStop>(*this, estop_client_, request, &response)) {
      std::cout << response->message << "\n";
    }
  }

  void printState(ConsoleAction action)
  {
    auto request = std::make_shared<robot_interfaces::srv::GetDispatchState::Request>();
    request->requester = "terminal";
    robot_interfaces::srv::GetDispatchState::Response::SharedPtr response;
    if (!waitAndCall<robot_interfaces::srv::GetDispatchState>(*this, state_client_, request, &response)) {
      return;
    }
    if (action == ConsoleAction::PrintTasks) {
      for (const auto & task : response->tasks) {
        std::cout << task.task_id << " type=" << int(task.task_type)
                  << " state=" << int(task.state.state)
                  << " robot=" << task.assigned_robot_id
                  << " step=" << task.current_step_index << "\n";
      }
    } else if (action == ConsoleAction::PrintRobots) {
      for (const auto & robot : response->robot_states) {
        std::cout << robot.robot_id << " state=" << int(robot.state)
                  << " task=" << robot.current_task_id << "\n";
      }
    } else {
      for (const auto & lock : response->resource_locks) {
        std::cout << lock.resource_id << " status=" << int(lock.status)
                  << " task=" << lock.locked_by_task_id << "\n";
      }
    }
  }

  bool refreshTaskPoints()
  {
    auto request = std::make_shared<robot_interfaces::srv::GetTaskPoints::Request>();
    request->requester = "terminal";
    robot_interfaces::srv::GetTaskPoints::Response::SharedPtr response;
    if (!waitAndCall<robot_interfaces::srv::GetTaskPoints>(
        *this, points_client_, request, &response))
    {
      return false;
    }
    task_points_ = taskPointConfigFromMessages(response->points);
    return true;
  }

  void printPoints()
  {
    if (!refreshTaskPoints()) {
      return;
    }
    for (const auto & [point_id, point] : task_points_.points) {
      const auto kind = point.kind == PointKind::WAITING_AREA
        ? robot_interfaces::msg::TaskPoint::KIND_WAITING_AREA
        : point.kind == PointKind::PICKUP
        ? robot_interfaces::msg::TaskPoint::KIND_PICKUP
        : point.kind == PointKind::DELIVERY
        ? robot_interfaces::msg::TaskPoint::KIND_DELIVERY
        : robot_interfaces::msg::TaskPoint::KIND_INSPECTION;
      std::cout << point_id << " kind=" << pointKindName(kind)
                << " x=" << point.pose.x << " y=" << point.pose.y
                << " temporary=" << (point.temporary ? "true" : "false")
                << " label=\"" << point.label << "\"\n";
    }
  }

  void clearTemporaryPoints()
  {
    auto request =
      std::make_shared<robot_interfaces::srv::ClearTemporaryTaskPoints::Request>();
    request->requester = "terminal";
    robot_interfaces::srv::ClearTemporaryTaskPoints::Response::SharedPtr response;
    if (!waitAndCall<robot_interfaces::srv::ClearTemporaryTaskPoints>(
        *this, clear_points_client_, request, &response))
    {
      return;
    }
    std::cout << response->message << ": cleared=" << response->cleared_count;
    if (!response->blocked_point_ids.empty()) {
      std::cout << " blocked=";
      for (std::size_t i = 0; i < response->blocked_point_ids.size(); ++i) {
        if (i != 0) {
          std::cout << ",";
        }
        std::cout << response->blocked_point_ids[i];
      }
    }
    std::cout << "\n";
  }

  TaskPointConfig task_points_;
  rclcpp::Client<robot_interfaces::srv::CreateTask>::SharedPtr create_client_;
  rclcpp::Client<robot_interfaces::srv::CancelTask>::SharedPtr cancel_client_;
  rclcpp::Client<robot_interfaces::srv::PauseTask>::SharedPtr pause_client_;
  rclcpp::Client<robot_interfaces::srv::ResumeTask>::SharedPtr resume_client_;
  rclcpp::Client<robot_interfaces::srv::ConfirmTaskStep>::SharedPtr confirm_client_;
  rclcpp::Client<robot_interfaces::srv::EmergencyStop>::SharedPtr estop_client_;
  rclcpp::Client<robot_interfaces::srv::GetDispatchState>::SharedPtr state_client_;
  rclcpp::Client<robot_interfaces::srv::GetTaskPoints>::SharedPtr points_client_;
  rclcpp::Client<robot_interfaces::srv::ClearTemporaryTaskPoints>::SharedPtr clear_points_client_;
};

}  // namespace robot_dispatch

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_dispatch::TerminalDispatchConsole>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
