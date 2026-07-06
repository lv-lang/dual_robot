#pragma once

#include <string>
#include <vector>
#include <optional>

namespace robot_dispatch
{

enum class ConsoleAction
{
  CreateTask,
  ConfirmTaskStep,
  PauseTask,
  ResumeTask,
  CancelTask,
  EmergencyStop,
  PrintTasks,
  PrintRobots,
  PrintLocks,
  PrintPoints,
  ClearTemporaryPoints,
  Help,
  Quit,
};

struct ServiceCallSpec
{
  ConsoleAction action{ConsoleAction::Help};
  std::string service_name;
  std::string task_id;
  std::string task_type;
  std::vector<std::string> point_ids;
  std::string confirmation;
  std::string robot_id;
  bool requires_service{false};
};

struct ParseResult
{
  bool ok{false};
  std::string error;
  ServiceCallSpec spec;
};

ParseResult parse_console_command(const std::string & line);
std::string normalize_token(const std::string & token);
std::optional<std::string> parse_confirmation(const std::string & token);
std::string console_help_text();

}  // namespace robot_dispatch
