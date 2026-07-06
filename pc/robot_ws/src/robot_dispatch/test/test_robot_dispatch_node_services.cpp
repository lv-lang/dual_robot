#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_dispatch/robot_dispatch_node.hpp"
#include "robot_interfaces/action/execute_mission.hpp"
#include "robot_interfaces/msg/task_point.hpp"
#include "robot_interfaces/srv/add_task_point.hpp"
#include "robot_interfaces/srv/cancel_task.hpp"
#include "robot_interfaces/srv/clear_temporary_task_points.hpp"
#include "robot_interfaces/srv/confirm_task_step.hpp"
#include "robot_interfaces/srv/create_task.hpp"
#include "robot_interfaces/srv/emergency_stop.hpp"
#include "robot_interfaces/srv/get_dispatch_state.hpp"
#include "robot_interfaces/srv/get_task_points.hpp"
#include "robot_interfaces/srv/pause_task.hpp"
#include "robot_interfaces/srv/resume_task.hpp"

using namespace std::chrono_literals;

namespace robot_dispatch
{

namespace
{

using ExecuteMission = robot_interfaces::action::ExecuteMission;
using GoalHandleExecuteMission = rclcpp_action::ServerGoalHandle<ExecuteMission>;

class FakeExecuteMissionServer
{
public:
  FakeExecuteMissionServer(
    const rclcpp::Node::SharedPtr & node,
    const std::string & action_name)
  {
    server_ = rclcpp_action::create_server<ExecuteMission>(
      node,
      action_name,
      [this](
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ExecuteMission::Goal> goal)
      {
        goals_.push_back(goal->command.task_id);
        if (!goal->command.steps.empty()) {
          point_ids_.push_back(goal->command.steps.front().point_id);
          task_point_ids_.push_back(
            std::make_pair(goal->command.task_id, goal->command.steps.front().point_id));
          step_types_.push_back(goal->command.steps.front().step_type);
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](
        const std::shared_ptr<GoalHandleExecuteMission>)
      {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [](
        const std::shared_ptr<GoalHandleExecuteMission> goal_handle)
      {
        auto result = std::make_shared<ExecuteMission::Result>();
        result->status = ExecuteMission::Result::STATUS_SUCCEEDED;
        result->task_id = goal_handle->get_goal()->command.task_id;
        result->success = true;
        result->message = "fake mission success";
        goal_handle->succeed(result);
      });
  }

  bool sawPoint(const std::string & point_id) const
  {
    return std::find(point_ids_.begin(), point_ids_.end(), point_id) != point_ids_.end();
  }

  bool sawTaskPoint(const std::string & task_id, const std::string & point_id) const
  {
    return std::find(
      task_point_ids_.begin(),
      task_point_ids_.end(),
      std::make_pair(task_id, point_id)) != task_point_ids_.end();
  }

  bool sawReturnHome() const
  {
    return std::find(goals_.begin(), goals_.end(), "return_home") != goals_.end();
  }

private:
  rclcpp_action::Server<ExecuteMission>::SharedPtr server_;
  std::vector<std::string> goals_;
  std::vector<std::string> point_ids_;
  std::vector<std::pair<std::string, std::string>> task_point_ids_;
  std::vector<uint8_t> step_types_;
};

class ControllableExecuteMissionServer
{
public:
  ControllableExecuteMissionServer(
    const rclcpp::Node::SharedPtr & node,
    const std::string & action_name)
  {
    server_ = rclcpp_action::create_server<ExecuteMission>(
      node,
      action_name,
      [this](
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ExecuteMission::Goal> goal)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_counts_[goal->command.task_id] += 1;
        if (!goal->command.steps.empty()) {
          point_ids_.push_back(goal->command.steps.front().point_id);
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](
        const std::shared_ptr<GoalHandleExecuteMission> goal_handle)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        cancel_counts_[goal_handle->get_goal()->command.task_id] += 1;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](
        const std::shared_ptr<GoalHandleExecuteMission> goal_handle)
      {
        std::lock_guard<std::mutex> lock(threads_mutex_);
        threads_.emplace_back([this, goal_handle]() {
          while (rclcpp::ok() && !stop_.load()) {
            if (goal_handle->is_canceling()) {
              auto result = std::make_shared<ExecuteMission::Result>();
              result->status = ExecuteMission::Result::STATUS_CANCELED;
              result->task_id = goal_handle->get_goal()->command.task_id;
              result->success = false;
              result->message = "fake mission canceled";
              goal_handle->canceled(result);
              return;
            }
            std::this_thread::sleep_for(10ms);
          }
        });
      });
  }

  ~ControllableExecuteMissionServer()
  {
    stop_.store(true);
    std::lock_guard<std::mutex> lock(threads_mutex_);
    for (auto & thread : threads_) {
      if (thread.joinable()) {
        thread.join();
      }
    }
  }

  int goalCount(const std::string & task_id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = goal_counts_.find(task_id);
    return it == goal_counts_.end() ? 0 : it->second;
  }

  int cancelCount(const std::string & task_id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = cancel_counts_.find(task_id);
    return it == cancel_counts_.end() ? 0 : it->second;
  }

  bool sawPoint(const std::string & point_id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::find(point_ids_.begin(), point_ids_.end(), point_id) != point_ids_.end();
  }

private:
  rclcpp_action::Server<ExecuteMission>::SharedPtr server_;
  mutable std::mutex mutex_;
  mutable std::mutex threads_mutex_;
  std::atomic_bool stop_{false};
  std::map<std::string, int> goal_counts_;
  std::map<std::string, int> cancel_counts_;
  std::vector<std::string> point_ids_;
  std::vector<std::thread> threads_;
};

template<typename PredicateT>
void spinUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  PredicateT predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return;
    }
    std::this_thread::sleep_for(10ms);
  }
}

const robot_interfaces::msg::Task * findTask(
  const std::vector<robot_interfaces::msg::Task> & tasks,
  const std::string & task_id)
{
  const auto it = std::find_if(tasks.begin(), tasks.end(), [&](const auto & task) {
    return task.task_id == task_id;
  });
  return it == tasks.end() ? nullptr : &(*it);
}

const robot_interfaces::msg::RobotState * findRobot(
  const std::vector<robot_interfaces::msg::RobotState> & robots,
  const std::string & robot_id)
{
  const auto it = std::find_if(robots.begin(), robots.end(), [&](const auto & robot) {
    return robot.robot_id == robot_id;
  });
  return it == robots.end() ? nullptr : &(*it);
}

}  // namespace

TEST(RobotDispatchNodeServices, CreateTaskPublishesQueryableState)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter(
      "task_points_file",
      std::string(ROBOT_DISPATCH_SOURCE_DIR) + "/config/task_points.yaml"),
    rclcpp::Parameter("enable_mission_execution", false),
  });
  auto dispatch = std::make_shared<RobotDispatchNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("dispatch_node_service_test");
  auto create_client =
    client_node->create_client<robot_interfaces::srv::CreateTask>(
    "/robot_dispatch/create_task");
  auto state_client =
    client_node->create_client<robot_interfaces::srv::GetDispatchState>(
    "/robot_dispatch/get_state");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dispatch);
  executor.add_node(client_node);
  ASSERT_TRUE(create_client->wait_for_service(2s));
  auto request = std::make_shared<robot_interfaces::srv::CreateTask::Request>();
  request->task_type = robot_interfaces::msg::Task::TYPE_DELIVERY;
  robot_interfaces::msg::MissionStep pickup;
  pickup.point_id = "PICKUP_A";
  robot_interfaces::msg::MissionStep delivery;
  delivery.point_id = "DELIVERY_C";
  request->steps = {pickup, delivery};

  auto create_future = create_client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(create_future, 2s));
  auto create_response = create_future.get();
  ASSERT_TRUE(create_response->accepted);
  EXPECT_EQ("task_1", create_response->task_id);
  EXPECT_EQ("auto", create_response->task.preferred_robot_id);

  ASSERT_TRUE(state_client->wait_for_service(2s));
  auto state_future = state_client->async_send_request(
    std::make_shared<robot_interfaces::srv::GetDispatchState::Request>());
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(state_future, 2s));
  const auto state = state_future.get();
  ASSERT_EQ(1u, state->tasks.size());
  EXPECT_EQ("task_1", state->tasks.front().task_id);
  EXPECT_EQ(robot_interfaces::msg::TaskState::ASSIGNED, state->tasks.front().state.state);
  ASSERT_EQ(2u, state->robot_states.size());
  ASSERT_EQ(1u, state->resource_locks.size());
  EXPECT_EQ("PICKUP_A", state->resource_locks.front().point_id);
  ASSERT_EQ(1u, state->tasks.front().locked_resource_ids.size());
  EXPECT_EQ("PICKUP_A", state->tasks.front().locked_resource_ids.front());

  executor.cancel();
  executor.remove_node(dispatch);
  executor.remove_node(client_node);
}

TEST(RobotDispatchNodeServices, DeliveryPreferenceAssignsRobot2WhenAvailable)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter(
      "task_points_file",
      std::string(ROBOT_DISPATCH_SOURCE_DIR) + "/config/task_points.yaml"),
    rclcpp::Parameter("enable_mission_execution", false),
  });
  auto dispatch = std::make_shared<RobotDispatchNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("dispatch_preferred_robot2_test");
  auto create_client =
    client_node->create_client<robot_interfaces::srv::CreateTask>(
    "/robot_dispatch/create_task");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dispatch);
  executor.add_node(client_node);
  ASSERT_TRUE(create_client->wait_for_service(2s));
  auto request = std::make_shared<robot_interfaces::srv::CreateTask::Request>();
  request->task_type = robot_interfaces::msg::Task::TYPE_DELIVERY;
  request->preferred_robot_id = "robot2";
  robot_interfaces::msg::MissionStep pickup;
  pickup.point_id = "PICKUP_A";
  robot_interfaces::msg::MissionStep delivery;
  delivery.point_id = "DELIVERY_C";
  request->steps = {pickup, delivery};

  auto create_future = create_client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(create_future, 2s));
  const auto response = create_future.get();
  ASSERT_TRUE(response->accepted);
  EXPECT_EQ("robot2", response->task.assigned_robot_id);
  EXPECT_EQ("robot2", response->task.preferred_robot_id);

  executor.cancel();
  executor.remove_node(dispatch);
  executor.remove_node(client_node);
}

TEST(RobotDispatchNodeServices, OppositeRobotPreferencesKeepTaskPointsOnTheirTaskType)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter(
      "task_points_file",
      std::string(ROBOT_DISPATCH_SOURCE_DIR) + "/config/task_points.yaml"),
    rclcpp::Parameter("enable_mission_execution", true),
  });
  auto dispatch = std::make_shared<RobotDispatchNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("dispatch_opposite_preferences_test");
  auto robot1_action_node = std::make_shared<rclcpp::Node>("fake_robot1_opposite_preferences");
  auto robot2_action_node = std::make_shared<rclcpp::Node>("fake_robot2_opposite_preferences");
  FakeExecuteMissionServer robot1_server(robot1_action_node, "/robot1/execute_mission");
  FakeExecuteMissionServer robot2_server(robot2_action_node, "/robot2/execute_mission");

  auto create_client =
    client_node->create_client<robot_interfaces::srv::CreateTask>(
    "/robot_dispatch/create_task");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dispatch);
  executor.add_node(client_node);
  executor.add_node(robot1_action_node);
  executor.add_node(robot2_action_node);
  ASSERT_TRUE(create_client->wait_for_service(2s));

  auto delivery_request = std::make_shared<robot_interfaces::srv::CreateTask::Request>();
  delivery_request->task_type = robot_interfaces::msg::Task::TYPE_DELIVERY;
  delivery_request->preferred_robot_id = "robot2";
  robot_interfaces::msg::MissionStep pickup;
  pickup.point_id = "PICKUP_A";
  robot_interfaces::msg::MissionStep delivery;
  delivery.point_id = "DELIVERY_C";
  delivery_request->steps = {pickup, delivery};

  auto delivery_future = create_client->async_send_request(delivery_request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(delivery_future, 2s));
  const auto delivery_response = delivery_future.get();
  ASSERT_TRUE(delivery_response->accepted);
  EXPECT_EQ("robot2", delivery_response->task.assigned_robot_id);

  spinUntil(executor, [&]() {
    return robot2_server.sawTaskPoint(delivery_response->task_id, "PICKUP_A");
  });
  EXPECT_TRUE(robot2_server.sawTaskPoint(delivery_response->task_id, "PICKUP_A"));
  EXPECT_FALSE(robot2_server.sawTaskPoint(delivery_response->task_id, "P1"));

  auto inspection_request = std::make_shared<robot_interfaces::srv::CreateTask::Request>();
  inspection_request->task_type = robot_interfaces::msg::Task::TYPE_INSPECTION;
  inspection_request->preferred_robot_id = "robot1";
  robot_interfaces::msg::MissionStep p1;
  p1.point_id = "P1";
  robot_interfaces::msg::MissionStep p2;
  p2.point_id = "P2";
  robot_interfaces::msg::MissionStep p3;
  p3.point_id = "P3";
  inspection_request->steps = {p1, p2, p3};

  auto inspection_future = create_client->async_send_request(inspection_request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(inspection_future, 2s));
  const auto inspection_response = inspection_future.get();
  ASSERT_TRUE(inspection_response->accepted);
  EXPECT_EQ("robot1", inspection_response->task.assigned_robot_id);

  spinUntil(executor, [&]() {
    return robot1_server.sawTaskPoint(inspection_response->task_id, "P1");
  });
  EXPECT_TRUE(robot1_server.sawTaskPoint(inspection_response->task_id, "P1"));
  EXPECT_FALSE(robot1_server.sawTaskPoint(inspection_response->task_id, "PICKUP_A"));

  executor.cancel();
  executor.remove_node(dispatch);
  executor.remove_node(client_node);
  executor.remove_node(robot1_action_node);
  executor.remove_node(robot2_action_node);
}

TEST(RobotDispatchNodeServices, RejectsInvalidPreferredRobotId)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter(
      "task_points_file",
      std::string(ROBOT_DISPATCH_SOURCE_DIR) + "/config/task_points.yaml"),
    rclcpp::Parameter("enable_mission_execution", false),
  });
  auto dispatch = std::make_shared<RobotDispatchNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("dispatch_invalid_preference_test");
  auto create_client =
    client_node->create_client<robot_interfaces::srv::CreateTask>(
    "/robot_dispatch/create_task");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dispatch);
  executor.add_node(client_node);
  ASSERT_TRUE(create_client->wait_for_service(2s));
  auto request = std::make_shared<robot_interfaces::srv::CreateTask::Request>();
  request->task_type = robot_interfaces::msg::Task::TYPE_DELIVERY;
  request->preferred_robot_id = "robot3";
  robot_interfaces::msg::MissionStep pickup;
  pickup.point_id = "PICKUP_A";
  request->steps = {pickup};

  auto create_future = create_client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(create_future, 2s));
  const auto response = create_future.get();
  EXPECT_FALSE(response->accepted);
  EXPECT_EQ("invalid preferred_robot_id: robot3", response->message);
  EXPECT_TRUE(response->task_id.empty());

  executor.cancel();
  executor.remove_node(dispatch);
  executor.remove_node(client_node);
}

TEST(RobotDispatchNodeServices, CompletedDeliverySendsRobotBackToWaitingArea)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter(
      "task_points_file",
      std::string(ROBOT_DISPATCH_SOURCE_DIR) + "/config/task_points.yaml"),
    rclcpp::Parameter("enable_mission_execution", true),
  });
  auto dispatch = std::make_shared<RobotDispatchNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("dispatch_return_home_test");
  auto action_node = std::make_shared<rclcpp::Node>("fake_robot1_execute_mission");
  FakeExecuteMissionServer robot1_server(action_node, "/robot1/execute_mission");

  auto create_client =
    client_node->create_client<robot_interfaces::srv::CreateTask>(
    "/robot_dispatch/create_task");
  auto confirm_client =
    client_node->create_client<robot_interfaces::srv::ConfirmTaskStep>(
    "/robot_dispatch/confirm_task_step");
  auto state_client =
    client_node->create_client<robot_interfaces::srv::GetDispatchState>(
    "/robot_dispatch/get_state");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dispatch);
  executor.add_node(client_node);
  executor.add_node(action_node);

  ASSERT_TRUE(create_client->wait_for_service(2s));
  auto request = std::make_shared<robot_interfaces::srv::CreateTask::Request>();
  request->task_type = robot_interfaces::msg::Task::TYPE_DELIVERY;
  robot_interfaces::msg::MissionStep pickup;
  pickup.point_id = "PICKUP_A";
  robot_interfaces::msg::MissionStep delivery;
  delivery.point_id = "DELIVERY_C";
  request->steps = {pickup, delivery};

  auto create_future = create_client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(create_future, 2s));
  ASSERT_TRUE(create_future.get()->accepted);
  spinUntil(executor, [&]() {return robot1_server.sawPoint("PICKUP_A");});
  ASSERT_TRUE(robot1_server.sawPoint("PICKUP_A"));

  ASSERT_TRUE(confirm_client->wait_for_service(2s));
  auto pickup_confirm =
    std::make_shared<robot_interfaces::srv::ConfirmTaskStep::Request>();
  pickup_confirm->task_id = "task_1";
  pickup_confirm->result =
    robot_interfaces::srv::ConfirmTaskStep::Request::RESULT_OK;
  auto pickup_future = confirm_client->async_send_request(pickup_confirm);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(pickup_future, 2s));
  ASSERT_TRUE(pickup_future.get()->accepted);
  spinUntil(executor, [&]() {return robot1_server.sawPoint("DELIVERY_C");});
  ASSERT_TRUE(robot1_server.sawPoint("DELIVERY_C"));

  auto delivery_confirm =
    std::make_shared<robot_interfaces::srv::ConfirmTaskStep::Request>();
  delivery_confirm->task_id = "task_1";
  delivery_confirm->result =
    robot_interfaces::srv::ConfirmTaskStep::Request::RESULT_OK;
  auto delivery_future = confirm_client->async_send_request(delivery_confirm);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(delivery_future, 2s));
  ASSERT_TRUE(delivery_future.get()->accepted);
  spinUntil(executor, [&]() {return robot1_server.sawReturnHome();});
  EXPECT_TRUE(robot1_server.sawReturnHome());

  ASSERT_TRUE(state_client->wait_for_service(2s));
  auto state_future = state_client->async_send_request(
    std::make_shared<robot_interfaces::srv::GetDispatchState::Request>());
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(state_future, 2s));
  const auto state = state_future.get();
  ASSERT_FALSE(state->tasks.empty());
  EXPECT_EQ(robot_interfaces::msg::TaskState::SUCCEEDED, state->tasks.front().state.state);

  executor.cancel();
  executor.remove_node(dispatch);
  executor.remove_node(client_node);
  executor.remove_node(action_node);
}

TEST(RobotDispatchNodeServices, RedispatchToReturningHomeRobotCancelsReturnHomeGoal)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter(
      "task_points_file",
      std::string(ROBOT_DISPATCH_SOURCE_DIR) + "/config/task_points.yaml"),
    rclcpp::Parameter("enable_mission_execution", true),
  });
  auto dispatch = std::make_shared<RobotDispatchNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("dispatch_return_home_cancel_test");
  auto action_node = std::make_shared<rclcpp::Node>("controllable_robot1_return_home_test");
  ControllableExecuteMissionServer robot1_server(action_node, "/robot1/execute_mission");

  auto create_client =
    client_node->create_client<robot_interfaces::srv::CreateTask>(
    "/robot_dispatch/create_task");
  auto cancel_client =
    client_node->create_client<robot_interfaces::srv::CancelTask>(
    "/robot_dispatch/cancel_task");
  auto state_client =
    client_node->create_client<robot_interfaces::srv::GetDispatchState>(
    "/robot_dispatch/get_state");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dispatch);
  executor.add_node(client_node);
  executor.add_node(action_node);

  ASSERT_TRUE(create_client->wait_for_service(2s));
  auto first_request = std::make_shared<robot_interfaces::srv::CreateTask::Request>();
  first_request->task_type = robot_interfaces::msg::Task::TYPE_DELIVERY;
  robot_interfaces::msg::MissionStep first_pickup;
  first_pickup.point_id = "PICKUP_A";
  first_request->steps = {first_pickup};
  auto first_future = create_client->async_send_request(first_request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(first_future, 2s));
  ASSERT_TRUE(first_future.get()->accepted);
  spinUntil(executor, [&]() {
    return robot1_server.goalCount("task_1") == 1 && robot1_server.sawPoint("PICKUP_A");
  });
  ASSERT_EQ(1, robot1_server.goalCount("task_1"));

  ASSERT_TRUE(cancel_client->wait_for_service(2s));
  auto cancel_request = std::make_shared<robot_interfaces::srv::CancelTask::Request>();
  cancel_request->task_id = "task_1";
  auto cancel_future = cancel_client->async_send_request(cancel_request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(cancel_future, 2s));
  ASSERT_TRUE(cancel_future.get()->accepted);
  spinUntil(executor, [&]() {
    return robot1_server.goalCount("return_home") == 1 && robot1_server.sawPoint("W1");
  });
  ASSERT_EQ(1, robot1_server.goalCount("return_home"));

  auto second_request = std::make_shared<robot_interfaces::srv::CreateTask::Request>();
  second_request->task_type = robot_interfaces::msg::Task::TYPE_DELIVERY;
  second_request->preferred_robot_id = "robot1";
  robot_interfaces::msg::MissionStep second_pickup;
  second_pickup.point_id = "PICKUP_A";
  second_request->steps = {second_pickup};
  auto second_future = create_client->async_send_request(second_request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(second_future, 2s));
  const auto second_response = second_future.get();
  ASSERT_TRUE(second_response->accepted);
  EXPECT_EQ("robot1", second_response->task.assigned_robot_id);

  spinUntil(executor, [&]() {
    return robot1_server.cancelCount("return_home") >= 1 &&
      robot1_server.goalCount("task_2") >= 1 &&
      robot1_server.sawPoint("PICKUP_A");
  });
  EXPECT_GE(robot1_server.cancelCount("return_home"), 1);
  EXPECT_GE(robot1_server.goalCount("task_2"), 1);
  EXPECT_TRUE(robot1_server.sawPoint("PICKUP_A"));

  ASSERT_TRUE(state_client->wait_for_service(2s));
  auto state_future = state_client->async_send_request(
    std::make_shared<robot_interfaces::srv::GetDispatchState::Request>());
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(state_future, 2s));
  const auto state = state_future.get();
  const auto * first_task = findTask(state->tasks, "task_1");
  const auto * second_task = findTask(state->tasks, "task_2");
  ASSERT_NE(first_task, nullptr);
  ASSERT_NE(second_task, nullptr);
  EXPECT_EQ(robot_interfaces::msg::TaskState::CANCELED, first_task->state.state);
  EXPECT_EQ("robot1", second_task->assigned_robot_id);

  executor.cancel();
  executor.remove_node(dispatch);
  executor.remove_node(client_node);
  executor.remove_node(action_node);
}

TEST(RobotDispatchNodeServices, AddsListsAndClearsTemporaryTaskPoints)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter(
      "task_points_file",
      std::string(ROBOT_DISPATCH_SOURCE_DIR) + "/config/task_points.yaml"),
    rclcpp::Parameter("enable_mission_execution", false),
  });
  auto dispatch = std::make_shared<RobotDispatchNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("dispatch_task_points_test");
  auto add_client =
    client_node->create_client<robot_interfaces::srv::AddTaskPoint>(
    "/robot_dispatch/add_task_point");
  auto get_client =
    client_node->create_client<robot_interfaces::srv::GetTaskPoints>(
    "/robot_dispatch/get_task_points");
  auto clear_client =
    client_node->create_client<robot_interfaces::srv::ClearTemporaryTaskPoints>(
    "/robot_dispatch/clear_temporary_task_points");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dispatch);
  executor.add_node(client_node);
  ASSERT_TRUE(add_client->wait_for_service(2s));
  auto add_request = std::make_shared<robot_interfaces::srv::AddTaskPoint::Request>();
  add_request->kind = robot_interfaces::msg::TaskPoint::KIND_PICKUP;
  add_request->pose.header.frame_id = "map";
  add_request->pose.pose.position.x = -6.0;
  add_request->pose.pose.position.y = -2.0;

  auto add_future = add_client->async_send_request(add_request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(add_future, 2s));
  const auto add_response = add_future.get();
  ASSERT_TRUE(add_response->accepted);
  EXPECT_EQ("RVIZ_PICKUP_1", add_response->point.point_id);
  EXPECT_TRUE(add_response->point.temporary);

  ASSERT_TRUE(get_client->wait_for_service(2s));
  auto get_future = get_client->async_send_request(
    std::make_shared<robot_interfaces::srv::GetTaskPoints::Request>());
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(get_future, 2s));
  const auto points = get_future.get();
  const auto found = std::find_if(
    points->points.begin(), points->points.end(), [](const auto & point) {
      return point.point_id == "RVIZ_PICKUP_1" && point.temporary;
    });
  ASSERT_NE(found, points->points.end());

  ASSERT_TRUE(clear_client->wait_for_service(2s));
  auto clear_future = clear_client->async_send_request(
    std::make_shared<robot_interfaces::srv::ClearTemporaryTaskPoints::Request>());
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(clear_future, 2s));
  const auto clear_response = clear_future.get();
  ASSERT_TRUE(clear_response->accepted);
  EXPECT_EQ(1u, clear_response->cleared_count);

  executor.cancel();
  executor.remove_node(dispatch);
  executor.remove_node(client_node);
}

TEST(RobotDispatchNodeServices, TerminalTaskControlServicesDriveStateMachine)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter(
      "task_points_file",
      std::string(ROBOT_DISPATCH_SOURCE_DIR) + "/config/task_points.yaml"),
    rclcpp::Parameter("enable_mission_execution", false),
  });
  auto dispatch = std::make_shared<RobotDispatchNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("dispatch_terminal_controls_test");
  auto create_client =
    client_node->create_client<robot_interfaces::srv::CreateTask>(
    "/robot_dispatch/create_task");
  auto pause_client =
    client_node->create_client<robot_interfaces::srv::PauseTask>(
    "/robot_dispatch/pause_task");
  auto resume_client =
    client_node->create_client<robot_interfaces::srv::ResumeTask>(
    "/robot_dispatch/resume_task");
  auto cancel_client =
    client_node->create_client<robot_interfaces::srv::CancelTask>(
    "/robot_dispatch/cancel_task");
  auto estop_client =
    client_node->create_client<robot_interfaces::srv::EmergencyStop>(
    "/robot_dispatch/emergency_stop");
  auto state_client =
    client_node->create_client<robot_interfaces::srv::GetDispatchState>(
    "/robot_dispatch/get_state");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dispatch);
  executor.add_node(client_node);

  ASSERT_TRUE(create_client->wait_for_service(2s));
  auto create_request = std::make_shared<robot_interfaces::srv::CreateTask::Request>();
  create_request->task_type = robot_interfaces::msg::Task::TYPE_DELIVERY;
  robot_interfaces::msg::MissionStep pickup;
  pickup.point_id = "PICKUP_A";
  robot_interfaces::msg::MissionStep delivery;
  delivery.point_id = "DELIVERY_C";
  create_request->steps = {pickup, delivery};
  auto create_future = create_client->async_send_request(create_request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(create_future, 2s));
  ASSERT_TRUE(create_future.get()->accepted);

  ASSERT_TRUE(pause_client->wait_for_service(2s));
  auto pause_request = std::make_shared<robot_interfaces::srv::PauseTask::Request>();
  pause_request->task_id = "task_1";
  auto pause_future = pause_client->async_send_request(pause_request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(pause_future, 2s));
  const auto pause_response = pause_future.get();
  ASSERT_TRUE(pause_response->accepted);
  EXPECT_EQ("paused", pause_response->message);

  ASSERT_TRUE(resume_client->wait_for_service(2s));
  auto resume_request = std::make_shared<robot_interfaces::srv::ResumeTask::Request>();
  resume_request->task_id = "task_1";
  auto resume_future = resume_client->async_send_request(resume_request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(resume_future, 2s));
  const auto resume_response = resume_future.get();
  ASSERT_TRUE(resume_response->accepted);
  EXPECT_EQ("resumed", resume_response->message);

  ASSERT_TRUE(cancel_client->wait_for_service(2s));
  auto cancel_request = std::make_shared<robot_interfaces::srv::CancelTask::Request>();
  cancel_request->task_id = "task_1";
  auto cancel_future = cancel_client->async_send_request(cancel_request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(cancel_future, 2s));
  const auto cancel_response = cancel_future.get();
  ASSERT_TRUE(cancel_response->accepted);
  EXPECT_EQ("canceled; robot returning to waiting area", cancel_response->message);

  ASSERT_TRUE(estop_client->wait_for_service(2s));
  auto estop_request = std::make_shared<robot_interfaces::srv::EmergencyStop::Request>();
  estop_request->active = true;
  auto estop_future = estop_client->async_send_request(estop_request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(estop_future, 2s));
  const auto estop_response = estop_future.get();
  ASSERT_TRUE(estop_response->accepted);
  EXPECT_EQ("emergency stop active", estop_response->message);

  ASSERT_TRUE(state_client->wait_for_service(2s));
  auto state_future = state_client->async_send_request(
    std::make_shared<robot_interfaces::srv::GetDispatchState::Request>());
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(state_future, 2s));
  const auto state = state_future.get();
  const auto * task = findTask(state->tasks, "task_1");
  ASSERT_NE(task, nullptr);
  EXPECT_EQ(robot_interfaces::msg::TaskState::CANCELED, task->state.state);
  const auto * robot1 = findRobot(state->robot_states, "robot1");
  const auto * robot2 = findRobot(state->robot_states, "robot2");
  ASSERT_NE(robot1, nullptr);
  ASSERT_NE(robot2, nullptr);
  EXPECT_EQ(robot_interfaces::msg::RobotState::ESTOP, robot1->state);
  EXPECT_EQ(robot_interfaces::msg::RobotState::ESTOP, robot2->state);

  executor.cancel();
  executor.remove_node(dispatch);
  executor.remove_node(client_node);
}

TEST(RobotDispatchNodeServices, PauseResumeCancelControlActiveMissionGoal)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter(
      "task_points_file",
      std::string(ROBOT_DISPATCH_SOURCE_DIR) + "/config/task_points.yaml"),
    rclcpp::Parameter("enable_mission_execution", true),
  });
  auto dispatch = std::make_shared<RobotDispatchNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("dispatch_active_goal_control_test");
  auto action_node = std::make_shared<rclcpp::Node>("controllable_robot1_execute_mission");
  ControllableExecuteMissionServer robot1_server(action_node, "/robot1/execute_mission");

  auto create_client =
    client_node->create_client<robot_interfaces::srv::CreateTask>(
    "/robot_dispatch/create_task");
  auto pause_client =
    client_node->create_client<robot_interfaces::srv::PauseTask>(
    "/robot_dispatch/pause_task");
  auto resume_client =
    client_node->create_client<robot_interfaces::srv::ResumeTask>(
    "/robot_dispatch/resume_task");
  auto cancel_client =
    client_node->create_client<robot_interfaces::srv::CancelTask>(
    "/robot_dispatch/cancel_task");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dispatch);
  executor.add_node(client_node);
  executor.add_node(action_node);

  ASSERT_TRUE(create_client->wait_for_service(2s));
  auto create_request = std::make_shared<robot_interfaces::srv::CreateTask::Request>();
  create_request->task_type = robot_interfaces::msg::Task::TYPE_DELIVERY;
  robot_interfaces::msg::MissionStep pickup;
  pickup.point_id = "PICKUP_A";
  robot_interfaces::msg::MissionStep delivery;
  delivery.point_id = "DELIVERY_C";
  create_request->steps = {pickup, delivery};
  auto create_future = create_client->async_send_request(create_request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(create_future, 2s));
  ASSERT_TRUE(create_future.get()->accepted);
  spinUntil(executor, [&]() {
    return robot1_server.goalCount("task_1") == 1 && robot1_server.sawPoint("PICKUP_A");
  });
  ASSERT_EQ(1, robot1_server.goalCount("task_1"));

  ASSERT_TRUE(pause_client->wait_for_service(2s));
  auto pause_request = std::make_shared<robot_interfaces::srv::PauseTask::Request>();
  pause_request->task_id = "task_1";
  auto pause_future = pause_client->async_send_request(pause_request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(pause_future, 2s));
  ASSERT_TRUE(pause_future.get()->accepted);
  spinUntil(executor, [&]() {return robot1_server.cancelCount("task_1") >= 1;});
  EXPECT_GE(robot1_server.cancelCount("task_1"), 1);

  ASSERT_TRUE(resume_client->wait_for_service(2s));
  auto resume_request = std::make_shared<robot_interfaces::srv::ResumeTask::Request>();
  resume_request->task_id = "task_1";
  auto resume_future = resume_client->async_send_request(resume_request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(resume_future, 2s));
  ASSERT_TRUE(resume_future.get()->accepted);
  spinUntil(executor, [&]() {return robot1_server.goalCount("task_1") >= 2;});
  EXPECT_GE(robot1_server.goalCount("task_1"), 2);

  ASSERT_TRUE(cancel_client->wait_for_service(2s));
  auto cancel_request = std::make_shared<robot_interfaces::srv::CancelTask::Request>();
  cancel_request->task_id = "task_1";
  auto cancel_future = cancel_client->async_send_request(cancel_request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(cancel_future, 2s));
  ASSERT_TRUE(cancel_future.get()->accepted);
  spinUntil(executor, [&]() {
    return robot1_server.cancelCount("task_1") >= 2 &&
      robot1_server.goalCount("return_home") >= 1 &&
      robot1_server.sawPoint("W1");
  });
  EXPECT_GE(robot1_server.cancelCount("task_1"), 2);
  EXPECT_GE(robot1_server.goalCount("return_home"), 1);
  EXPECT_TRUE(robot1_server.sawPoint("W1"));

  executor.cancel();
  executor.remove_node(dispatch);
  executor.remove_node(client_node);
  executor.remove_node(action_node);
}

TEST(RobotDispatchNodeServices, RejectsClearingTemporaryPointsReferencedByActiveTasks)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter(
      "task_points_file",
      std::string(ROBOT_DISPATCH_SOURCE_DIR) + "/config/task_points.yaml"),
    rclcpp::Parameter("enable_mission_execution", false),
  });
  auto dispatch = std::make_shared<RobotDispatchNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("dispatch_blocked_points_test");
  auto add_client =
    client_node->create_client<robot_interfaces::srv::AddTaskPoint>(
    "/robot_dispatch/add_task_point");
  auto create_client =
    client_node->create_client<robot_interfaces::srv::CreateTask>(
    "/robot_dispatch/create_task");
  auto clear_client =
    client_node->create_client<robot_interfaces::srv::ClearTemporaryTaskPoints>(
    "/robot_dispatch/clear_temporary_task_points");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dispatch);
  executor.add_node(client_node);
  ASSERT_TRUE(add_client->wait_for_service(2s));
  auto add_request = std::make_shared<robot_interfaces::srv::AddTaskPoint::Request>();
  add_request->kind = robot_interfaces::msg::TaskPoint::KIND_INSPECTION;
  add_request->pose.header.frame_id = "map";
  add_request->pose.pose.position.x = -6.0;
  add_request->pose.pose.position.y = 2.0;
  auto add_future = add_client->async_send_request(add_request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(add_future, 2s));
  ASSERT_TRUE(add_future.get()->accepted);

  ASSERT_TRUE(create_client->wait_for_service(2s));
  auto create_request = std::make_shared<robot_interfaces::srv::CreateTask::Request>();
  create_request->task_type = robot_interfaces::msg::Task::TYPE_INSPECTION;
  robot_interfaces::msg::MissionStep step;
  step.point_id = "RVIZ_INSPECTION_1";
  create_request->steps = {step};
  auto create_future = create_client->async_send_request(create_request);
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(create_future, 2s));
  ASSERT_TRUE(create_future.get()->accepted);

  ASSERT_TRUE(clear_client->wait_for_service(2s));
  auto clear_future = clear_client->async_send_request(
    std::make_shared<robot_interfaces::srv::ClearTemporaryTaskPoints::Request>());
  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    executor.spin_until_future_complete(clear_future, 2s));
  const auto clear_response = clear_future.get();
  EXPECT_FALSE(clear_response->accepted);
  ASSERT_EQ(1u, clear_response->blocked_point_ids.size());
  EXPECT_EQ("RVIZ_INSPECTION_1", clear_response->blocked_point_ids.front());

  executor.cancel();
  executor.remove_node(dispatch);
  executor.remove_node(client_node);
}

}  // namespace robot_dispatch
