#include <gtest/gtest.h>

#include "robot_dispatch/command_parser.hpp"

namespace robot_dispatch
{

TEST(CommandParser, ParsesDeliveryCommand)
{
  const auto parsed = parse_console_command("1 PICKUP_A DELIVERY_C");

  ASSERT_TRUE(parsed.ok) << parsed.error;
  EXPECT_EQ(ConsoleAction::CreateTask, parsed.spec.action);
  EXPECT_EQ("DELIVERY", parsed.spec.task_type);
  ASSERT_EQ(2u, parsed.spec.point_ids.size());
  EXPECT_EQ("PICKUP_A", parsed.spec.point_ids[0]);
  EXPECT_EQ("DELIVERY_C", parsed.spec.point_ids[1]);
  EXPECT_EQ("/robot_dispatch/create_task", parsed.spec.service_name);
}

TEST(CommandParser, ParsesInspectionConfirmAndPrints)
{
  EXPECT_TRUE(parse_console_command("2 P1 P2 P3").ok);

  const auto confirm = parse_console_command("3 task_1 abnormal");
  ASSERT_TRUE(confirm.ok) << confirm.error;
  EXPECT_EQ(ConsoleAction::ConfirmTaskStep, confirm.spec.action);
  EXPECT_EQ("ABNORMAL", confirm.spec.confirmation);

  EXPECT_EQ(ConsoleAction::PrintTasks, parse_console_command("8").spec.action);
  EXPECT_EQ(ConsoleAction::PrintRobots, parse_console_command("9").spec.action);
  EXPECT_EQ(ConsoleAction::PrintLocks, parse_console_command("10").spec.action);
  EXPECT_EQ(ConsoleAction::PrintPoints, parse_console_command("11").spec.action);
  EXPECT_EQ(ConsoleAction::ClearTemporaryPoints, parse_console_command("12").spec.action);
}

TEST(CommandParser, ParsesTaskControlCommands)
{
  const auto pause = parse_console_command("4 task_1");
  ASSERT_TRUE(pause.ok) << pause.error;
  EXPECT_EQ(ConsoleAction::PauseTask, pause.spec.action);
  EXPECT_EQ("task_1", pause.spec.task_id);
  EXPECT_EQ("/robot_dispatch/pause_task", pause.spec.service_name);

  const auto resume = parse_console_command("5 task_1");
  ASSERT_TRUE(resume.ok) << resume.error;
  EXPECT_EQ(ConsoleAction::ResumeTask, resume.spec.action);
  EXPECT_EQ("task_1", resume.spec.task_id);
  EXPECT_EQ("/robot_dispatch/resume_task", resume.spec.service_name);

  const auto cancel = parse_console_command("6 task_1");
  ASSERT_TRUE(cancel.ok) << cancel.error;
  EXPECT_EQ(ConsoleAction::CancelTask, cancel.spec.action);
  EXPECT_EQ("task_1", cancel.spec.task_id);
  EXPECT_EQ("/robot_dispatch/cancel_task", cancel.spec.service_name);
}

TEST(CommandParser, ParsesGlobalEmergencyStopOnly)
{
  const auto implicit_all = parse_console_command("7");
  ASSERT_TRUE(implicit_all.ok) << implicit_all.error;
  EXPECT_EQ(ConsoleAction::EmergencyStop, implicit_all.spec.action);
  EXPECT_EQ("all", implicit_all.spec.robot_id);
  EXPECT_EQ("/robot_dispatch/emergency_stop", implicit_all.spec.service_name);

  const auto explicit_all = parse_console_command("7 all");
  ASSERT_TRUE(explicit_all.ok) << explicit_all.error;
  EXPECT_EQ("all", explicit_all.spec.robot_id);

  EXPECT_FALSE(parse_console_command("7 robot1").ok);
  EXPECT_FALSE(parse_console_command("7 robot2").ok);
  EXPECT_FALSE(parse_console_command("7 banana").ok);
}

TEST(CommandParser, RejectsInvalidCommands)
{
  EXPECT_FALSE(parse_console_command("").ok);
  EXPECT_FALSE(parse_console_command("1 ONLY_PICKUP").ok);
  EXPECT_FALSE(parse_console_command("3 task_1 MAYBE").ok);
}

}  // namespace robot_dispatch
