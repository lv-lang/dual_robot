#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "robot_dispatch/dispatch_types.hpp"

namespace robot_dispatch
{

class DispatchCore
{
public:
  void registerRobot(
    const std::string & robot_id,
    const std::string & waiting_area_id,
    RobotState initial_state = RobotState::IDLE);

  int createTask(
    TaskType type,
    const std::vector<std::string> & target_point_ids,
    int parent_task_id = 0,
    const std::string & originating_robot_id = "",
    const std::string & abnormal_point_id = "",
    const std::string & preferred_robot_id = "auto");

  std::optional<DispatchDecision> dispatchOnce();
  std::vector<DispatchDecision> dispatchAll();

  bool markTaskRunning(int task_id);
  bool setTaskWaitingConfirmation(int task_id);
  bool confirmTaskStep(int task_id, ConfirmationResult result);
  bool pauseTask(int task_id);
  bool resumeTask(int task_id);
  bool cancelTask(int task_id);
  bool failTask(int task_id, const std::string & reason);
  void emergencyStop();
  void failAllActiveTasks(const std::string & reason, bool release_locks);
  void clearResourceLocks();
  void setResourceLocks(const std::vector<ResourceLock> & locks);
  bool completeReturn(const std::string & robot_id);

  bool setRobotState(const std::string & robot_id, RobotState state);

  const TaskRecord * task(int task_id) const;
  const RobotRecord * robot(const std::string & robot_id) const;
  std::vector<TaskRecord> tasks() const;
  std::vector<RobotRecord> robots() const;
  std::vector<ResourceLock> resourceLocks() const;
  bool hasLock(const std::string & point_id, int task_id) const;

private:
  std::vector<int> sortedDispatchableTaskIds() const;
  std::vector<std::string> candidateRobotsForTask(const TaskRecord & task) const;
  std::vector<std::string> requiredLocksFor(
    const TaskRecord & task,
    const RobotRecord & robot) const;
  bool canAcquireLocks(
    const TaskRecord & task,
    const RobotRecord & robot,
    const std::vector<std::string> & point_ids) const;
  void acquireLocks(
    const TaskRecord & task,
    const RobotRecord & robot,
    const std::vector<std::string> & point_ids);
  void releaseLockForTaskPoint(int task_id, const std::string & point_id);
  void releaseLocksForTask(int task_id);
  bool finishTask(int task_id, const std::string & business_result = "");
  bool advanceAfterConfirmation(TaskRecord & task, ConfirmationResult result);
  bool canShareAbnormalLock(const TaskRecord & task, const ResourceLock & lock) const;

  std::map<std::string, RobotRecord> robots_;
  std::map<int, TaskRecord> tasks_;
  std::map<std::string, ResourceLock> locks_;
  int next_task_id_{1};
  std::uint64_t next_sequence_{1};
};

}  // namespace robot_dispatch
