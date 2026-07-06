#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "robot_dispatch/dispatch_core.hpp"

namespace
{

robot_dispatch::DispatchCore makeCore()
{
  robot_dispatch::DispatchCore core;
  core.registerRobot("robot1", "W1");
  core.registerRobot("robot2", "W2");
  return core;
}

const robot_dispatch::TaskRecord & task(
  const robot_dispatch::DispatchCore & core,
  int task_id)
{
  const auto * record = core.task(task_id);
  EXPECT_NE(record, nullptr);
  return *record;
}

const robot_dispatch::RobotRecord & robot(
  const robot_dispatch::DispatchCore & core,
  const std::string & robot_id)
{
  const auto * record = core.robot(robot_id);
  EXPECT_NE(record, nullptr);
  return *record;
}

}  // namespace

TEST(DispatchCore, OnlyIdleAndReturningHomeRobotsAreDispatchable)
{
  const std::vector<robot_dispatch::RobotState> unavailable_states = {
    robot_dispatch::RobotState::ASSIGNED,
    robot_dispatch::RobotState::EXECUTING,
    robot_dispatch::RobotState::WAITING_CONFIRMATION,
    robot_dispatch::RobotState::WAITING_RESOURCE,
    robot_dispatch::RobotState::PAUSED,
    robot_dispatch::RobotState::ESTOP,
    robot_dispatch::RobotState::ERROR,
  };

  for (const auto unavailable_state : unavailable_states) {
    auto core = makeCore();
    ASSERT_TRUE(core.setRobotState("robot1", unavailable_state));
    ASSERT_TRUE(core.setRobotState("robot2", unavailable_state));

    const auto task_id = core.createTask(
      robot_dispatch::TaskType::DELIVERY,
      {"PICKUP_A", "DELIVERY_C"});

    EXPECT_FALSE(core.dispatchOnce().has_value()) << robot_dispatch::toString(unavailable_state);
    EXPECT_EQ(task(core, task_id).state, robot_dispatch::TaskState::PENDING);
  }

  auto returning_core = makeCore();
  ASSERT_TRUE(returning_core.setRobotState(
    "robot1", robot_dispatch::RobotState::RETURNING_HOME));
  ASSERT_TRUE(returning_core.setRobotState("robot2", robot_dispatch::RobotState::ERROR));
  const auto task_id = returning_core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_A", "DELIVERY_C"});
  const auto decision = returning_core.dispatchOnce();
  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->task_id, task_id);
  EXPECT_EQ(decision->robot_id, "robot1");
  EXPECT_TRUE(decision->interrupted_return_home);
}

TEST(DispatchCore, DeliveryPrefersRobot1AndFallsBackToRobot2)
{
  auto core = makeCore();

  const auto first = core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_A", "DELIVERY_C"});
  const auto first_decision = core.dispatchOnce();
  ASSERT_TRUE(first_decision.has_value());
  EXPECT_EQ(first_decision->task_id, first);
  EXPECT_EQ(first_decision->robot_id, "robot1");

  const auto second = core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_B", "DELIVERY_D"});
  const auto second_decision = core.dispatchOnce();
  ASSERT_TRUE(second_decision.has_value());
  EXPECT_EQ(second_decision->task_id, second);
  EXPECT_EQ(second_decision->robot_id, "robot2");
}

TEST(DispatchCore, PreferredRobotOverridesTaskTypeDefaultWhenAvailable)
{
  auto delivery_core = makeCore();
  const auto delivery = delivery_core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_A", "DELIVERY_C"},
    0,
    "",
    "",
    "robot2");
  const auto delivery_decision = delivery_core.dispatchOnce();
  ASSERT_TRUE(delivery_decision.has_value());
  EXPECT_EQ(delivery_decision->task_id, delivery);
  EXPECT_EQ(delivery_decision->robot_id, "robot2");
  EXPECT_EQ(task(delivery_core, delivery).preferred_robot_id, "robot2");

  auto inspection_core = makeCore();
  const auto inspection = inspection_core.createTask(
    robot_dispatch::TaskType::INSPECTION,
    {"P1", "P2", "P3"},
    0,
    "",
    "",
    "robot1");
  const auto inspection_decision = inspection_core.dispatchOnce();
  ASSERT_TRUE(inspection_decision.has_value());
  EXPECT_EQ(inspection_decision->task_id, inspection);
  EXPECT_EQ(inspection_decision->robot_id, "robot1");
}

TEST(DispatchCore, UnavailablePreferredRobotFallsBackByTaskTypeOrder)
{
  auto core = makeCore();
  ASSERT_TRUE(core.setRobotState("robot2", robot_dispatch::RobotState::EXECUTING));

  const auto delivery = core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_A", "DELIVERY_C"},
    0,
    "",
    "",
    "robot2");

  const auto decision = core.dispatchOnce();
  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->task_id, delivery);
  EXPECT_EQ(decision->robot_id, "robot1");
}

TEST(DispatchCore, SharedFutureDeliveryDoesNotBlockSecondPickup)
{
  auto core = makeCore();

  const auto first = core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_A", "DELIVERY_C"});
  const auto first_decision = core.dispatchOnce();
  ASSERT_TRUE(first_decision.has_value());
  EXPECT_EQ(first_decision->task_id, first);
  EXPECT_EQ(first_decision->robot_id, "robot1");
  EXPECT_TRUE(core.hasLock("PICKUP_A", first));
  EXPECT_FALSE(core.hasLock("DELIVERY_C", first));

  const auto second = core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_B", "DELIVERY_C"});
  const auto second_decision = core.dispatchOnce();
  ASSERT_TRUE(second_decision.has_value());
  EXPECT_EQ(second_decision->task_id, second);
  EXPECT_EQ(second_decision->robot_id, "robot2");
  EXPECT_TRUE(core.hasLock("PICKUP_B", second));
  EXPECT_FALSE(core.hasLock("DELIVERY_C", second));
}

TEST(DispatchCore, DeliveryStepTransitionReleasesCompletedPointAndWaitsForNextLock)
{
  auto core = makeCore();

  const auto first = core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_A", "DELIVERY_C"});
  ASSERT_TRUE(core.dispatchOnce().has_value());
  const auto second = core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_B", "DELIVERY_C"});
  ASSERT_TRUE(core.dispatchOnce().has_value());

  ASSERT_TRUE(core.markTaskRunning(first));
  ASSERT_TRUE(core.setTaskWaitingConfirmation(first));
  ASSERT_TRUE(core.confirmTaskStep(first, robot_dispatch::ConfirmationResult::OK));
  EXPECT_EQ(task(core, first).current_step_index, 1u);
  EXPECT_EQ(task(core, first).state, robot_dispatch::TaskState::RUNNING);
  EXPECT_FALSE(core.hasLock("PICKUP_A", first));
  EXPECT_TRUE(core.hasLock("DELIVERY_C", first));

  ASSERT_TRUE(core.markTaskRunning(second));
  ASSERT_TRUE(core.setTaskWaitingConfirmation(second));
  ASSERT_TRUE(core.confirmTaskStep(second, robot_dispatch::ConfirmationResult::OK));
  EXPECT_EQ(task(core, second).current_step_index, 1u);
  EXPECT_EQ(task(core, second).state, robot_dispatch::TaskState::WAITING_RESOURCE);
  EXPECT_EQ(robot(core, "robot2").state, robot_dispatch::RobotState::WAITING_RESOURCE);
  EXPECT_FALSE(core.hasLock("PICKUP_B", second));
  EXPECT_FALSE(core.hasLock("DELIVERY_C", second));

  ASSERT_TRUE(core.setTaskWaitingConfirmation(first));
  ASSERT_TRUE(core.confirmTaskStep(first, robot_dispatch::ConfirmationResult::OK));
  EXPECT_EQ(task(core, first).state, robot_dispatch::TaskState::SUCCEEDED);
  EXPECT_FALSE(core.hasLock("DELIVERY_C", first));

  const auto resumed = core.dispatchOnce();
  ASSERT_TRUE(resumed.has_value());
  EXPECT_EQ(resumed->task_id, second);
  EXPECT_EQ(resumed->robot_id, "robot2");
  EXPECT_TRUE(core.hasLock("DELIVERY_C", second));
}

TEST(DispatchCore, ResumeWaitingResourceTaskDoesNotBypassLockedPoint)
{
  auto core = makeCore();

  const auto first = core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_A", "DELIVERY_C"});
  ASSERT_TRUE(core.dispatchOnce().has_value());
  const auto second = core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_B", "DELIVERY_C"});
  ASSERT_TRUE(core.dispatchOnce().has_value());

  ASSERT_TRUE(core.markTaskRunning(first));
  ASSERT_TRUE(core.setTaskWaitingConfirmation(first));
  ASSERT_TRUE(core.confirmTaskStep(first, robot_dispatch::ConfirmationResult::OK));
  ASSERT_TRUE(core.markTaskRunning(second));
  ASSERT_TRUE(core.setTaskWaitingConfirmation(second));
  ASSERT_TRUE(core.confirmTaskStep(second, robot_dispatch::ConfirmationResult::OK));
  ASSERT_EQ(task(core, second).state, robot_dispatch::TaskState::WAITING_RESOURCE);

  ASSERT_TRUE(core.pauseTask(second));
  EXPECT_EQ(task(core, second).state, robot_dispatch::TaskState::PAUSED);
  ASSERT_TRUE(core.resumeTask(second));
  EXPECT_EQ(task(core, second).state, robot_dispatch::TaskState::WAITING_RESOURCE);
  EXPECT_EQ(robot(core, "robot2").state, robot_dispatch::RobotState::WAITING_RESOURCE);
  EXPECT_FALSE(core.hasLock("DELIVERY_C", second));
}

TEST(DispatchCore, InspectionPrefersRobot2AndFallsBackToRobot1)
{
  auto core = makeCore();

  const auto inspection = core.createTask(
    robot_dispatch::TaskType::INSPECTION,
    {"P1", "P2", "P3"});
  const auto decision = core.dispatchOnce();
  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->task_id, inspection);
  EXPECT_EQ(decision->robot_id, "robot2");

  auto fallback_core = makeCore();
  ASSERT_TRUE(fallback_core.setRobotState("robot2", robot_dispatch::RobotState::ERROR));
  const auto fallback = fallback_core.createTask(
    robot_dispatch::TaskType::INSPECTION,
    {"P1", "P2", "P3"});
  const auto fallback_decision = fallback_core.dispatchOnce();
  ASSERT_TRUE(fallback_decision.has_value());
  EXPECT_EQ(fallback_decision->task_id, fallback);
  EXPECT_EQ(fallback_decision->robot_id, "robot1");
}

TEST(DispatchCore, RecheckPriorityBeatsOrdinaryTasksWithoutPreemption)
{
  auto core = makeCore();

  const auto delivery = core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_A", "DELIVERY_C"});
  const auto recheck = core.createTask(
    robot_dispatch::TaskType::RECHECK,
    {"P1"},
    42,
    "robot1",
    "P1");

  const auto decisions = core.dispatchAll();
  ASSERT_EQ(decisions.size(), 2u);
  EXPECT_EQ(decisions[0].task_id, recheck);
  EXPECT_EQ(decisions[0].robot_id, "robot2");
  EXPECT_EQ(decisions[1].task_id, delivery);
  EXPECT_EQ(decisions[1].robot_id, "robot1");

  auto no_preempt_core = makeCore();
  ASSERT_TRUE(no_preempt_core.setRobotState("robot2", robot_dispatch::RobotState::EXECUTING));
  const auto pending_recheck = no_preempt_core.createTask(
    robot_dispatch::TaskType::RECHECK,
    {"P2"},
    7,
    "robot1",
    "P2");
  const auto ordinary = no_preempt_core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_A", "DELIVERY_C"});

  const auto fallback_decision = no_preempt_core.dispatchOnce();
  ASSERT_TRUE(fallback_decision.has_value());
  EXPECT_EQ(fallback_decision->task_id, ordinary);
  EXPECT_EQ(fallback_decision->robot_id, "robot1");
  EXPECT_EQ(task(no_preempt_core, pending_recheck).state, robot_dispatch::TaskState::PENDING);
}

TEST(DispatchCore, RecheckExcludesOriginatingRobotEvenWhenPreferred)
{
  auto core = makeCore();
  const auto recheck = core.createTask(
    robot_dispatch::TaskType::RECHECK,
    {"P1"},
    42,
    "robot1",
    "P1",
    "robot1");

  const auto decision = core.dispatchOnce();
  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->task_id, recheck);
  EXPECT_EQ(decision->robot_id, "robot2");
}

TEST(DispatchCore, OrdinaryTasksAreFifoWithinPriorityBand)
{
  auto core = makeCore();
  const auto delivery = core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_A", "DELIVERY_C"});
  const auto inspection = core.createTask(
    robot_dispatch::TaskType::INSPECTION,
    {"P1", "P2", "P3"});

  const auto decisions = core.dispatchAll();
  ASSERT_EQ(decisions.size(), 2u);
  EXPECT_EQ(decisions[0].task_id, delivery);
  EXPECT_EQ(decisions[0].robot_id, "robot1");
  EXPECT_EQ(decisions[1].task_id, inspection);
  EXPECT_EQ(decisions[1].robot_id, "robot2");
}

TEST(DispatchCore, InspectionLocksRouteAndRecheckMayShareOnlyAbnormalPoint)
{
  auto core = makeCore();
  const auto inspection = core.createTask(
    robot_dispatch::TaskType::INSPECTION,
    {"P1", "P2", "P3"});
  ASSERT_TRUE(core.dispatchOnce().has_value());

  EXPECT_TRUE(core.hasLock("P1", inspection));
  EXPECT_TRUE(core.hasLock("P2", inspection));
  EXPECT_TRUE(core.hasLock("P3", inspection));

  const auto blocked_delivery = core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"P1", "DELIVERY_C"});
  EXPECT_FALSE(core.dispatchOnce().has_value());
  EXPECT_EQ(task(core, blocked_delivery).state, robot_dispatch::TaskState::PENDING);

  ASSERT_TRUE(core.markTaskRunning(inspection));
  ASSERT_TRUE(core.setTaskWaitingConfirmation(inspection));
  ASSERT_TRUE(core.confirmTaskStep(inspection, robot_dispatch::ConfirmationResult::ABNORMAL));

  const auto tasks = core.tasks();
  const auto recheck_it = std::find_if(tasks.begin(), tasks.end(), [&](const auto & record) {
    return record.type == robot_dispatch::TaskType::RECHECK &&
           record.parent_task_id == inspection;
  });
  ASSERT_NE(recheck_it, tasks.end());
  EXPECT_EQ(recheck_it->originating_robot_id, "robot2");
  EXPECT_EQ(recheck_it->abnormal_point_id, "P1");
  EXPECT_EQ(task(core, inspection).current_step_index, 1u);
  EXPECT_EQ(task(core, inspection).state, robot_dispatch::TaskState::RUNNING);

  const auto recheck_decision = core.dispatchOnce();
  ASSERT_TRUE(recheck_decision.has_value());
  EXPECT_EQ(recheck_decision->task_id, recheck_it->id);
  EXPECT_EQ(recheck_decision->robot_id, "robot1");

  const auto locks = core.resourceLocks();
  const auto p1_lock_it = std::find_if(locks.begin(), locks.end(), [](const auto & lock) {
    return lock.point_id == "P1";
  });
  ASSERT_NE(p1_lock_it, locks.end());
  ASSERT_EQ(p1_lock_it->holders.size(), 2u);
  EXPECT_TRUE(core.hasLock("P1", inspection));
  EXPECT_TRUE(core.hasLock("P1", recheck_it->id));
  EXPECT_EQ(task(core, blocked_delivery).state, robot_dispatch::TaskState::PENDING);
}

TEST(DispatchCore, CancelReleasesOriginalTaskAndStartsSeparateReturn)
{
  auto core = makeCore();
  const auto delivery = core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_A", "DELIVERY_C"});
  ASSERT_TRUE(core.dispatchOnce().has_value());

  ASSERT_TRUE(core.cancelTask(delivery));
  EXPECT_EQ(task(core, delivery).state, robot_dispatch::TaskState::CANCELED);
  EXPECT_EQ(robot(core, "robot1").state, robot_dispatch::RobotState::RETURNING_HOME);
  EXPECT_EQ(robot(core, "robot1").active_task_id, 0);
  EXPECT_TRUE(core.resourceLocks().empty());

  ASSERT_TRUE(core.completeReturn("robot1"));
  EXPECT_EQ(robot(core, "robot1").state, robot_dispatch::RobotState::IDLE);
}

TEST(DispatchCore, PauseKeepsLocksAndResumeKeepsCurrentStep)
{
  auto core = makeCore();
  const auto delivery = core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_A", "DELIVERY_C"});
  ASSERT_TRUE(core.dispatchOnce().has_value());
  ASSERT_TRUE(core.markTaskRunning(delivery));
  ASSERT_TRUE(core.setTaskWaitingConfirmation(delivery));
  ASSERT_TRUE(core.confirmTaskStep(delivery, robot_dispatch::ConfirmationResult::OK));

  ASSERT_TRUE(core.pauseTask(delivery));
  EXPECT_EQ(task(core, delivery).state, robot_dispatch::TaskState::PAUSED);
  EXPECT_EQ(task(core, delivery).current_step_index, 1u);
  EXPECT_FALSE(core.hasLock("PICKUP_A", delivery));
  EXPECT_TRUE(core.hasLock("DELIVERY_C", delivery));

  ASSERT_TRUE(core.resumeTask(delivery));
  EXPECT_EQ(task(core, delivery).state, robot_dispatch::TaskState::RESUMING);
  EXPECT_EQ(task(core, delivery).current_step_index, 1u);
  EXPECT_FALSE(core.hasLock("PICKUP_A", delivery));
  EXPECT_TRUE(core.hasLock("DELIVERY_C", delivery));
}

TEST(DispatchCore, EmergencyStopFailsActiveTasksAndDoesNotReturnHome)
{
  auto core = makeCore();
  const auto delivery = core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_A", "DELIVERY_C"});
  ASSERT_TRUE(core.dispatchOnce().has_value());
  ASSERT_TRUE(core.markTaskRunning(delivery));

  core.emergencyStop();

  EXPECT_EQ(task(core, delivery).state, robot_dispatch::TaskState::FAILED);
  EXPECT_EQ(task(core, delivery).failure_reason, "emergency_stop");
  EXPECT_EQ(robot(core, "robot1").state, robot_dispatch::RobotState::ESTOP);
  EXPECT_EQ(robot(core, "robot2").state, robot_dispatch::RobotState::ESTOP);
  EXPECT_NE(robot(core, "robot1").state, robot_dispatch::RobotState::RETURNING_HOME);
  EXPECT_TRUE(core.resourceLocks().empty());
}

TEST(DispatchCore, FailureDoesNotStartReturnHome)
{
  auto core = makeCore();
  const auto delivery = core.createTask(
    robot_dispatch::TaskType::DELIVERY,
    {"PICKUP_A", "DELIVERY_C"});
  ASSERT_TRUE(core.dispatchOnce().has_value());

  ASSERT_TRUE(core.failTask(delivery, "navigation_failed"));
  EXPECT_EQ(task(core, delivery).state, robot_dispatch::TaskState::FAILED);
  EXPECT_EQ(robot(core, "robot1").state, robot_dispatch::RobotState::ERROR);
  EXPECT_NE(robot(core, "robot1").state, robot_dispatch::RobotState::RETURNING_HOME);
  EXPECT_TRUE(core.resourceLocks().empty());
}
