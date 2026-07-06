#include "robot_dispatch/dispatch_core.hpp"

#include <algorithm>
#include <cctype>

namespace robot_dispatch
{

namespace
{

int priority(TaskType type)
{
  switch (type) {
    case TaskType::RECHECK:
      return 0;
    case TaskType::DELIVERY:
      return 1;
    case TaskType::INSPECTION:
      return 1;
  }
  return 9;
}

bool isTerminal(TaskState state)
{
  return state == TaskState::SUCCEEDED || state == TaskState::FAILED ||
         state == TaskState::CANCELED;
}

bool isDispatchable(RobotState state)
{
  return state == RobotState::IDLE || state == RobotState::RETURNING_HOME;
}

std::string normalizePreferredRobotId(std::string value)
{
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), [](unsigned char c) {
    return !std::isspace(c);
  }));
  value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char c) {
    return !std::isspace(c);
  }).base(), value.end());
  return value.empty() ? "auto" : value;
}

void appendUnique(std::vector<std::string> * ids, const std::string & robot_id)
{
  if (std::find(ids->begin(), ids->end(), robot_id) == ids->end()) {
    ids->push_back(robot_id);
  }
}

void appendIfRegistered(
  std::vector<std::string> * ids,
  const std::map<std::string, RobotRecord> & robots,
  const std::string & robot_id)
{
  if (robots.count(robot_id) > 0) {
    appendUnique(ids, robot_id);
  }
}

}  // namespace

void DispatchCore::registerRobot(
  const std::string & robot_id,
  const std::string & waiting_area_id,
  RobotState initial_state)
{
  robots_[robot_id] = RobotRecord{robot_id, waiting_area_id, initial_state, 0};
}

int DispatchCore::createTask(
  TaskType type,
  const std::vector<std::string> & target_point_ids,
  int parent_task_id,
  const std::string & originating_robot_id,
  const std::string & abnormal_point_id,
  const std::string & preferred_robot_id)
{
  const int id = next_task_id_++;
  TaskRecord task;
  task.id = id;
  task.type = type;
  task.state = TaskState::PENDING;
  task.target_point_ids = target_point_ids;
  task.created_sequence = next_sequence_++;
  task.parent_task_id = parent_task_id;
  task.originating_robot_id = originating_robot_id;
  task.preferred_robot_id = normalizePreferredRobotId(preferred_robot_id);
  task.abnormal_point_id =
    abnormal_point_id.empty() && !target_point_ids.empty()
    ? target_point_ids.front()
    : abnormal_point_id;
  tasks_[id] = task;
  return id;
}

std::optional<DispatchDecision> DispatchCore::dispatchOnce()
{
  for (const int task_id : sortedDispatchableTaskIds()) {
    auto task_it = tasks_.find(task_id);
    if (task_it == tasks_.end()) {
      continue;
    }
    auto & task = task_it->second;
    if (task.state == TaskState::WAITING_RESOURCE) {
      auto robot_it = robots_.find(task.assigned_robot_id);
      if (robot_it == robots_.end() ||
          robot_it->second.state != RobotState::WAITING_RESOURCE) {
        continue;
      }
      const auto required_locks = requiredLocksFor(task, robot_it->second);
      if (!canAcquireLocks(task, robot_it->second, required_locks)) {
        continue;
      }
      acquireLocks(task, robot_it->second, required_locks);
      task.state = TaskState::ASSIGNED;
      robot_it->second.state = RobotState::ASSIGNED;
      robot_it->second.active_task_id = task.id;
      return DispatchDecision{
        task.id, task.type, robot_it->second.id, required_locks, false};
    }

    for (const auto & robot_id : candidateRobotsForTask(task)) {
      auto robot_it = robots_.find(robot_id);
      if (robot_it == robots_.end() ||
          !isDispatchable(robot_it->second.state)) {
        continue;
      }
      const auto required_locks = requiredLocksFor(task, robot_it->second);
      if (!canAcquireLocks(task, robot_it->second, required_locks)) {
        continue;
      }

      const bool interrupted_return_home =
        robot_it->second.state == RobotState::RETURNING_HOME;
      acquireLocks(task, robot_it->second, required_locks);
      task.state = TaskState::ASSIGNED;
      task.assigned_robot_id = robot_id;
      robot_it->second.state = RobotState::ASSIGNED;
      robot_it->second.active_task_id = task.id;
      return DispatchDecision{
        task.id, task.type, robot_id, required_locks, interrupted_return_home};
    }
  }
  return std::nullopt;
}

std::vector<DispatchDecision> DispatchCore::dispatchAll()
{
  std::vector<DispatchDecision> decisions;
  while (true) {
    auto decision = dispatchOnce();
    if (!decision) {
      return decisions;
    }
    decisions.push_back(*decision);
  }
}

bool DispatchCore::markTaskRunning(int task_id)
{
  auto task_it = tasks_.find(task_id);
  if (task_it == tasks_.end()) {
    return false;
  }
  auto & task = task_it->second;
  if (isTerminal(task.state)) {
    return false;
  }
  task.state = TaskState::RUNNING;
  auto robot_it = robots_.find(task.assigned_robot_id);
  if (robot_it != robots_.end()) {
    robot_it->second.state = RobotState::EXECUTING;
    robot_it->second.active_task_id = task.id;
  }
  return true;
}

bool DispatchCore::setTaskWaitingConfirmation(int task_id)
{
  auto task_it = tasks_.find(task_id);
  if (task_it == tasks_.end() || isTerminal(task_it->second.state)) {
    return false;
  }
  task_it->second.state = TaskState::WAITING_CONFIRMATION;
  auto robot_it = robots_.find(task_it->second.assigned_robot_id);
  if (robot_it != robots_.end()) {
    robot_it->second.state = RobotState::WAITING_CONFIRMATION;
  }
  return true;
}

bool DispatchCore::confirmTaskStep(int task_id, ConfirmationResult result)
{
  auto task_it = tasks_.find(task_id);
  if (task_it == tasks_.end()) {
    return false;
  }
  return advanceAfterConfirmation(task_it->second, result);
}

bool DispatchCore::pauseTask(int task_id)
{
  auto task_it = tasks_.find(task_id);
  if (task_it == tasks_.end() || isTerminal(task_it->second.state)) {
    return false;
  }
  auto & task = task_it->second;
  task.state = TaskState::PAUSED;
  auto robot_it = robots_.find(task.assigned_robot_id);
  if (robot_it != robots_.end()) {
    robot_it->second.state = RobotState::PAUSED;
  }
  return true;
}

bool DispatchCore::resumeTask(int task_id)
{
  auto task_it = tasks_.find(task_id);
  if (task_it == tasks_.end() || task_it->second.state != TaskState::PAUSED) {
    return false;
  }
  auto & task = task_it->second;
  auto robot_it = robots_.find(task.assigned_robot_id);
  if (robot_it != robots_.end()) {
    const auto required_locks = requiredLocksFor(task, robot_it->second);
    if (!canAcquireLocks(task, robot_it->second, required_locks)) {
      task.state = TaskState::WAITING_RESOURCE;
      robot_it->second.state = RobotState::WAITING_RESOURCE;
      return true;
    }
    acquireLocks(task, robot_it->second, required_locks);
    task.state = TaskState::RESUMING;
    robot_it->second.state = RobotState::ASSIGNED;
  } else {
    task.state = TaskState::RESUMING;
  }
  return true;
}

bool DispatchCore::cancelTask(int task_id)
{
  auto task_it = tasks_.find(task_id);
  if (task_it == tasks_.end() || isTerminal(task_it->second.state)) {
    return false;
  }
  auto & task = task_it->second;
  task.state = TaskState::CANCELED;
  releaseLocksForTask(task_id);
  auto robot_it = robots_.find(task.assigned_robot_id);
  if (robot_it != robots_.end()) {
    robot_it->second.state = RobotState::RETURNING_HOME;
    robot_it->second.active_task_id = 0;
  }
  return true;
}

bool DispatchCore::failTask(int task_id, const std::string & reason)
{
  auto task_it = tasks_.find(task_id);
  if (task_it == tasks_.end() || isTerminal(task_it->second.state)) {
    return false;
  }
  auto & task = task_it->second;
  task.state = TaskState::FAILED;
  task.failure_reason = reason;
  releaseLocksForTask(task_id);
  auto robot_it = robots_.find(task.assigned_robot_id);
  if (robot_it != robots_.end()) {
    robot_it->second.state = RobotState::ERROR;
    robot_it->second.active_task_id = task.id;
  }
  return true;
}

void DispatchCore::emergencyStop()
{
  failAllActiveTasks("emergency_stop", true);
}

void DispatchCore::failAllActiveTasks(const std::string & reason, bool release_locks)
{
  for (auto & [_, robot] : robots_) {
    robot.state = RobotState::ESTOP;
  }
  for (auto & [_, task] : tasks_) {
    if (!isTerminal(task.state)) {
      task.state = TaskState::FAILED;
      task.failure_reason = reason;
    }
  }
  if (release_locks) {
    locks_.clear();
  }
}

void DispatchCore::clearResourceLocks()
{
  locks_.clear();
}

void DispatchCore::setResourceLocks(const std::vector<ResourceLock> & locks)
{
  locks_.clear();
  for (const auto & lock : locks) {
    if (!lock.point_id.empty() && !lock.holders.empty()) {
      locks_[lock.point_id] = lock;
    }
  }
}

bool DispatchCore::completeReturn(const std::string & robot_id)
{
  auto robot_it = robots_.find(robot_id);
  if (robot_it == robots_.end() ||
      robot_it->second.state != RobotState::RETURNING_HOME) {
    return false;
  }
  robot_it->second.state = RobotState::IDLE;
  robot_it->second.active_task_id = 0;
  return true;
}

bool DispatchCore::setRobotState(const std::string & robot_id, RobotState state)
{
  auto robot_it = robots_.find(robot_id);
  if (robot_it == robots_.end()) {
    return false;
  }
  robot_it->second.state = state;
  return true;
}

const TaskRecord * DispatchCore::task(int task_id) const
{
  auto it = tasks_.find(task_id);
  return it == tasks_.end() ? nullptr : &it->second;
}

const RobotRecord * DispatchCore::robot(const std::string & robot_id) const
{
  auto it = robots_.find(robot_id);
  return it == robots_.end() ? nullptr : &it->second;
}

std::vector<TaskRecord> DispatchCore::tasks() const
{
  std::vector<TaskRecord> values;
  for (const auto & [_, task] : tasks_) {
    values.push_back(task);
  }
  return values;
}

std::vector<RobotRecord> DispatchCore::robots() const
{
  std::vector<RobotRecord> values;
  for (const auto & [_, robot] : robots_) {
    values.push_back(robot);
  }
  return values;
}

std::vector<ResourceLock> DispatchCore::resourceLocks() const
{
  std::vector<ResourceLock> values;
  for (const auto & [_, lock] : locks_) {
    values.push_back(lock);
  }
  return values;
}

bool DispatchCore::hasLock(const std::string & point_id, int task_id) const
{
  auto lock_it = locks_.find(point_id);
  if (lock_it == locks_.end()) {
    return false;
  }
  return std::any_of(
    lock_it->second.holders.begin(), lock_it->second.holders.end(),
    [task_id](const LockHolder & holder) {return holder.task_id == task_id;});
}

std::vector<int> DispatchCore::sortedDispatchableTaskIds() const
{
  std::vector<int> ids;
  for (const auto & [id, task] : tasks_) {
    if (task.state == TaskState::WAITING_RESOURCE ||
        task.state == TaskState::PENDING) {
      ids.push_back(id);
    }
  }
  std::sort(ids.begin(), ids.end(), [this](int lhs, int rhs) {
    const auto & a = tasks_.at(lhs);
    const auto & b = tasks_.at(rhs);
    if (a.state != b.state) {
      return a.state == TaskState::WAITING_RESOURCE;
    }
    if (priority(a.type) != priority(b.type)) {
      return priority(a.type) < priority(b.type);
    }
    return a.created_sequence < b.created_sequence;
  });
  return ids;
}

std::vector<std::string> DispatchCore::candidateRobotsForTask(
  const TaskRecord & task) const
{
  std::vector<std::string> ids;
  const auto preferred_it = robots_.find(task.preferred_robot_id);
  if (preferred_it != robots_.end() &&
      task.preferred_robot_id != task.originating_robot_id &&
      isDispatchable(preferred_it->second.state)) {
    ids.push_back(task.preferred_robot_id);
  }

  if (task.type == TaskType::DELIVERY) {
    appendIfRegistered(&ids, robots_, "mecanum");
    appendIfRegistered(&ids, robots_, "robot1");
    appendIfRegistered(&ids, robots_, "ackermann");
    appendIfRegistered(&ids, robots_, "robot2");
  } else if (task.type == TaskType::INSPECTION) {
    appendIfRegistered(&ids, robots_, "ackermann");
    appendIfRegistered(&ids, robots_, "robot2");
    appendIfRegistered(&ids, robots_, "mecanum");
    appendIfRegistered(&ids, robots_, "robot1");
  } else {
    for (const auto & [robot_id, robot] : robots_) {
      if (robot_id != task.originating_robot_id &&
          isDispatchable(robot.state)) {
        appendUnique(&ids, robot_id);
      }
    }
    const auto preferred = task.preferred_robot_id;
    std::stable_sort(ids.begin(), ids.end(), [&](const auto & lhs, const auto & rhs) {
      if (lhs == preferred) {
        return true;
      }
      if (rhs == preferred) {
        return false;
      }
      return lhs < rhs;
    });
    return ids;
  }

  for (const auto & [robot_id, _] : robots_) {
    if (robot_id != task.originating_robot_id) {
      appendUnique(&ids, robot_id);
    }
  }
  return ids;
}

std::vector<std::string> DispatchCore::requiredLocksFor(
  const TaskRecord & task,
  const RobotRecord &) const
{
  if (task.type != TaskType::INSPECTION) {
    if (task.current_step_index >= task.target_point_ids.size()) {
      return {};
    }
    return {task.target_point_ids[task.current_step_index]};
  }

  std::vector<std::string> point_ids = task.target_point_ids;
  std::sort(point_ids.begin(), point_ids.end());
  point_ids.erase(std::unique(point_ids.begin(), point_ids.end()), point_ids.end());
  return point_ids;
}

bool DispatchCore::canAcquireLocks(
  const TaskRecord & task,
  const RobotRecord &,
  const std::vector<std::string> & point_ids) const
{
  for (const auto & point_id : point_ids) {
    auto lock_it = locks_.find(point_id);
    if (lock_it == locks_.end() || lock_it->second.holders.empty()) {
      continue;
    }
    const bool already_held_by_task = std::all_of(
      lock_it->second.holders.begin(), lock_it->second.holders.end(),
      [&task](const LockHolder & holder) {return holder.task_id == task.id;});
    if (already_held_by_task) {
      continue;
    }
    if (!canShareAbnormalLock(task, lock_it->second)) {
      return false;
    }
  }
  return true;
}

void DispatchCore::acquireLocks(
  const TaskRecord & task,
  const RobotRecord & robot,
  const std::vector<std::string> & point_ids)
{
  for (const auto & point_id : point_ids) {
    auto & lock = locks_[point_id];
    lock.point_id = point_id;
    const bool already_held = std::any_of(
      lock.holders.begin(), lock.holders.end(),
      [&task](const LockHolder & holder) {return holder.task_id == task.id;});
    if (already_held) {
      continue;
    }
    lock.holders.push_back(
      LockHolder{
        task.id,
        task.type,
        robot.id,
        task.parent_task_id,
        task.abnormal_point_id});
  }
}

void DispatchCore::releaseLockForTaskPoint(int task_id, const std::string & point_id)
{
  auto lock_it = locks_.find(point_id);
  if (lock_it == locks_.end()) {
    return;
  }
  auto & holders = lock_it->second.holders;
  holders.erase(
    std::remove_if(
      holders.begin(), holders.end(),
      [task_id](const LockHolder & holder) {
        return holder.task_id == task_id;
      }),
    holders.end());
  if (holders.empty()) {
    locks_.erase(lock_it);
  }
}

void DispatchCore::releaseLocksForTask(int task_id)
{
  for (auto it = locks_.begin(); it != locks_.end();) {
    auto & holders = it->second.holders;
    holders.erase(
      std::remove_if(
        holders.begin(), holders.end(),
        [task_id](const LockHolder & holder) {
          return holder.task_id == task_id;
        }),
      holders.end());
    if (holders.empty()) {
      it = locks_.erase(it);
    } else {
      ++it;
    }
  }
}

bool DispatchCore::finishTask(int task_id, const std::string & business_result)
{
  auto task_it = tasks_.find(task_id);
  if (task_it == tasks_.end()) {
    return false;
  }
  auto & task = task_it->second;
  task.state = TaskState::SUCCEEDED;
  task.business_result = business_result;
  releaseLocksForTask(task.id);
  auto robot_it = robots_.find(task.assigned_robot_id);
  if (robot_it != robots_.end()) {
    robot_it->second.state = RobotState::IDLE;
    robot_it->second.active_task_id = 0;
  }
  return true;
}

bool DispatchCore::advanceAfterConfirmation(
  TaskRecord & task,
  ConfirmationResult result)
{
  if (isTerminal(task.state) || task.state == TaskState::PAUSED ||
      task.state == TaskState::WAITING_RESOURCE) {
    return false;
  }

  const std::string current_point =
    task.current_step_index < task.target_point_ids.size()
    ? task.target_point_ids[task.current_step_index]
    : std::string{};

  if (task.type == TaskType::INSPECTION &&
      result == ConfirmationResult::ABNORMAL &&
      !current_point.empty()) {
    createTask(
      TaskType::RECHECK, {current_point}, task.id, task.assigned_robot_id,
      current_point);
  }

  if (task.type == TaskType::RECHECK) {
    return finishTask(task.id, toString(result));
  }

  if (task.type != TaskType::INSPECTION && !current_point.empty()) {
    releaseLockForTaskPoint(task.id, current_point);
  }

  task.current_step_index += 1;
  if (task.current_step_index >= task.target_point_ids.size()) {
    return finishTask(task.id, toString(result));
  }

  auto robot_it = robots_.find(task.assigned_robot_id);
  if (robot_it != robots_.end()) {
    const auto required_locks = requiredLocksFor(task, robot_it->second);
    if (!canAcquireLocks(task, robot_it->second, required_locks)) {
      task.state = TaskState::WAITING_RESOURCE;
      robot_it->second.state = RobotState::WAITING_RESOURCE;
      return true;
    }
    acquireLocks(task, robot_it->second, required_locks);
    task.state = TaskState::RUNNING;
    robot_it->second.state = RobotState::EXECUTING;
  } else {
    task.state = TaskState::RUNNING;
  }
  return true;
}

bool DispatchCore::canShareAbnormalLock(
  const TaskRecord & task,
  const ResourceLock & lock) const
{
  if (task.type != TaskType::RECHECK ||
      task.abnormal_point_id.empty() ||
      lock.point_id != task.abnormal_point_id) {
    return false;
  }
  return std::any_of(
    lock.holders.begin(), lock.holders.end(),
    [&task](const LockHolder & holder) {
      return holder.task_id == task.parent_task_id &&
             holder.task_type == TaskType::INSPECTION;
    });
}

}  // namespace robot_dispatch
