#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "robot_interfaces/action/execute_mission.hpp"
#include "robot_interfaces/msg/dispatch_lease.hpp"
#include "robot_interfaces/msg/mission_step.hpp"
#include "robot_interfaces/msg/resource_lock.hpp"
#include "robot_interfaces/msg/robot_heartbeat.hpp"
#include "robot_interfaces/msg/robot_health.hpp"
#include "robot_interfaces/msg/robot_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robot_interfaces/msg/system_state.hpp"
#include "robot_interfaces/msg/task.hpp"
#include "robot_interfaces/msg/task_point.hpp"
#include "robot_interfaces/srv/add_task_point.hpp"
#include "robot_interfaces/srv/cancel_task.hpp"
#include "robot_interfaces/srv/clear_temporary_task_points.hpp"
#include "robot_interfaces/srv/confirm_task_step.hpp"
#include "robot_interfaces/srv/create_task.hpp"
#include "robot_interfaces/srv/emergency_stop.hpp"
#include "robot_interfaces/srv/enable_system.hpp"
#include "robot_interfaces/srv/get_dispatch_state.hpp"
#include "robot_interfaces/srv/get_task_points.hpp"
#include "robot_interfaces/srv/pause_task.hpp"
#include "robot_interfaces/srv/recover_system.hpp"
#include "robot_interfaces/srv/resume_task.hpp"

#include "robot_dispatch/dispatch_core.hpp"
#include "robot_dispatch/marker_builder.hpp"
#include "robot_dispatch/task_points.hpp"

namespace robot_dispatch
{

class RobotDispatchNode : public rclcpp::Node
{
public:
  explicit RobotDispatchNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using ExecuteMission = robot_interfaces::action::ExecuteMission;
  using GoalHandleExecuteMission = rclcpp_action::ClientGoalHandle<ExecuteMission>;
  using GetLifecycleState = lifecycle_msgs::srv::GetState;

  enum class SystemMode
  {
    WAITING_ROBOTS,
    STANDBY,
    READY,
    ESTOPPED,
    INTERLOCKED
  };

  struct RobotRuntimeHealth
  {
    std::string robot_id;
    std::string robot_namespace;
    std::string map_version;
    uint8_t mission_state{0};
    std::string message;
    bool has_heartbeat{false};
    bool has_health{false};
    bool has_scan{false};
    bool has_odom{false};
    rclcpp::Time last_heartbeat;
    rclcpp::Time last_health;
    rclcpp::Time last_scan;
    rclcpp::Time last_odom;
    bool amcl_active{false};
    bool nav2_active{false};
    bool map_to_odom_ok{false};
    bool mission_ready{false};
    uint8_t health_state{0};
    float scan_age_sec{0.0F};
    float odom_age_sec{0.0F};
    float tf_age_sec{0.0F};
    std::string map_bundle_hash;
    std::vector<std::string> health_reasons;
    geometry_msgs::msg::PoseStamped map_pose;
    bool has_map_pose{false};
    rclcpp::Time map_pose_stamp;
  };

  void handleCreateTask(
    const std::shared_ptr<robot_interfaces::srv::CreateTask::Request> request,
    std::shared_ptr<robot_interfaces::srv::CreateTask::Response> response);
  void handleCancelTask(
    const std::shared_ptr<robot_interfaces::srv::CancelTask::Request> request,
    std::shared_ptr<robot_interfaces::srv::CancelTask::Response> response);
  void handlePauseTask(
    const std::shared_ptr<robot_interfaces::srv::PauseTask::Request> request,
    std::shared_ptr<robot_interfaces::srv::PauseTask::Response> response);
  void handleResumeTask(
    const std::shared_ptr<robot_interfaces::srv::ResumeTask::Request> request,
    std::shared_ptr<robot_interfaces::srv::ResumeTask::Response> response);
  void handleConfirmTaskStep(
    const std::shared_ptr<robot_interfaces::srv::ConfirmTaskStep::Request> request,
    std::shared_ptr<robot_interfaces::srv::ConfirmTaskStep::Response> response);
  void handleEmergencyStop(
    const std::shared_ptr<robot_interfaces::srv::EmergencyStop::Request> request,
    std::shared_ptr<robot_interfaces::srv::EmergencyStop::Response> response);
  void handleEnableSystem(
    const std::shared_ptr<robot_interfaces::srv::EnableSystem::Request> request,
    std::shared_ptr<robot_interfaces::srv::EnableSystem::Response> response);
  void handleRecoverSystem(
    const std::shared_ptr<robot_interfaces::srv::RecoverSystem::Request> request,
    std::shared_ptr<robot_interfaces::srv::RecoverSystem::Response> response);
  void handleGetDispatchState(
    const std::shared_ptr<robot_interfaces::srv::GetDispatchState::Request> request,
    std::shared_ptr<robot_interfaces::srv::GetDispatchState::Response> response);
  void handleAddTaskPoint(
    const std::shared_ptr<robot_interfaces::srv::AddTaskPoint::Request> request,
    std::shared_ptr<robot_interfaces::srv::AddTaskPoint::Response> response);
  void handleClearTemporaryTaskPoints(
    const std::shared_ptr<robot_interfaces::srv::ClearTemporaryTaskPoints::Request> request,
    std::shared_ptr<robot_interfaces::srv::ClearTemporaryTaskPoints::Response> response);
  void handleGetTaskPoints(
    const std::shared_ptr<robot_interfaces::srv::GetTaskPoints::Request> request,
    std::shared_ptr<robot_interfaces::srv::GetTaskPoints::Response> response);

  bool validatePointIds(const std::vector<std::string> & point_ids, std::string * error) const;
  std::vector<std::string> pointIdsFromSteps(
    const std::vector<robot_interfaces::msg::MissionStep> & steps) const;
  robot_interfaces::msg::Task taskMessage(const TaskRecord & task) const;
  robot_interfaces::msg::TaskPoint taskPointMessage(const TaskPoint & point) const;
  robot_interfaces::msg::RobotState robotStateMessage(const RobotRecord & robot) const;
  robot_interfaces::msg::ResourceLock resourceLockMessage(const ResourceLock & lock) const;
  robot_interfaces::msg::SystemState systemStateMessageLocked() const;
  robot_interfaces::msg::MissionStep missionStepForPoint(
    const TaskRecord & task,
    const std::string & point_id,
    std::size_t sequence) const;
  void fillState(
    std::vector<robot_interfaces::msg::Task> * tasks,
    std::vector<robot_interfaces::msg::RobotState> * robots,
    std::vector<robot_interfaces::msg::ResourceLock> * locks) const;

  void dispatchAndSendLocked();
  bool taskCreationAllowedLocked(std::string * reason) const;
  void sendCurrentStepLocked(const TaskRecord & task);
  void cancelActiveTaskMissionLocked(int task_id);
  void cancelReturnHomeGoalLocked(const std::string & robot_id);
  void cancelAllActiveMissionsLocked();
  void startReturnHomeLocked(const std::string & robot_id);
  bool sendReturnHomeLocked(const std::string & robot_id);
  void publishStateLocked();
  void publishState();
  void onHeartbeat(const robot_interfaces::msg::RobotHeartbeat::SharedPtr msg);
  void onHealth(const robot_interfaces::msg::RobotHealth::SharedPtr msg);
  void onScan(const std::string & robot_id, const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onOdom(const std::string & robot_id, const nav_msgs::msg::Odometry::SharedPtr msg);
  void evaluateSystemStateLocked();
  void pollLifecycleStates();
  void publishDispatchLeasesLocked();
  bool robotHealthOkLocked(const std::string & robot_id, std::string * reason) const;
  bool allRobotsHealthyLocked(std::string * reason) const;
  bool healthStableForEnableLocked() const;
  void triggerSystemStopLocked(SystemMode mode, const std::string & reason);
  bool loadSafetyStateLocked();
  void persistSafetyStateLocked(const std::string & reason);
  void clearPersistedSafetyStateLocked();
  static uint8_t toInterfaceSystemState(SystemMode mode);
  static const char * toSystemStateString(SystemMode mode);
  void publishSummaryLocked();
  void publishMarkersLocked();
  void onMissionResult(
    int task_id,
    const GoalHandleExecuteMission::WrappedResult & result);
  void onReturnHomeResult(
    const std::string & robot_id,
    const GoalHandleExecuteMission::WrappedResult & result);
  std::string nextTemporaryPointId(PointKind kind) const;
  std::vector<std::string> activeTemporaryPointReferencesLocked() const;

  static int parseTaskId(const std::string & task_id);
  static std::string formatTaskId(int task_id);

  mutable std::mutex mutex_;
  DispatchCore core_;
  TaskPointConfig task_points_;
  std::string task_points_file_;
  bool task_points_valid_{true};
  bool enforce_real_system_gates_{false};
  std::string map_version_;
  std::string map_bundle_hash_;
  std::string safety_state_file_;
  bool safety_state_file_warning_{false};
  std::string safety_state_file_warning_message_;
  SystemMode system_mode_{SystemMode::READY};
  std::optional<rclcpp::Time> health_ok_since_;
  bool enable_mission_execution_{true};
  uint32_t dispatch_lease_seq_{0};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tasks_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_states_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr resource_locks_pub_;
  rclcpp::Publisher<robot_interfaces::msg::SystemState>::SharedPtr system_state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  std::map<std::string, rclcpp::Publisher<robot_interfaces::msg::DispatchLease>::SharedPtr> dispatch_lease_pubs_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::TimerBase::SharedPtr dispatch_lease_timer_;

  rclcpp::Service<robot_interfaces::srv::CreateTask>::SharedPtr create_task_srv_;
  rclcpp::Service<robot_interfaces::srv::CancelTask>::SharedPtr cancel_task_srv_;
  rclcpp::Service<robot_interfaces::srv::PauseTask>::SharedPtr pause_task_srv_;
  rclcpp::Service<robot_interfaces::srv::ResumeTask>::SharedPtr resume_task_srv_;
  rclcpp::Service<robot_interfaces::srv::ConfirmTaskStep>::SharedPtr confirm_task_step_srv_;
  rclcpp::Service<robot_interfaces::srv::EmergencyStop>::SharedPtr emergency_stop_srv_;
  rclcpp::Service<robot_interfaces::srv::EnableSystem>::SharedPtr enable_system_srv_;
  rclcpp::Service<robot_interfaces::srv::RecoverSystem>::SharedPtr recover_system_srv_;
  rclcpp::Service<robot_interfaces::srv::GetDispatchState>::SharedPtr get_state_srv_;
  rclcpp::Service<robot_interfaces::srv::AddTaskPoint>::SharedPtr add_task_point_srv_;
  rclcpp::Service<robot_interfaces::srv::ClearTemporaryTaskPoints>::SharedPtr clear_temporary_points_srv_;
  rclcpp::Service<robot_interfaces::srv::GetTaskPoints>::SharedPtr get_task_points_srv_;

  std::map<std::string, rclcpp_action::Client<ExecuteMission>::SharedPtr> mission_clients_;
  std::map<std::string, RobotRuntimeHealth> robot_health_;
  std::vector<rclcpp::Subscription<robot_interfaces::msg::RobotHeartbeat>::SharedPtr> heartbeat_subs_;
  std::vector<rclcpp::Subscription<robot_interfaces::msg::RobotHealth>::SharedPtr> health_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subs_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  std::map<std::string, rclcpp::Client<GetLifecycleState>::SharedPtr> lifecycle_clients_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::map<int, GoalHandleExecuteMission::SharedPtr> active_task_goals_;
  std::map<int, std::string> active_task_goal_robots_;
  std::map<std::string, GoalHandleExecuteMission::SharedPtr> active_return_home_goals_;
  std::set<std::string> return_home_cancel_requested_;
};

}  // namespace robot_dispatch
