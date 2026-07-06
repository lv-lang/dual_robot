#include "robot_dispatch/command_parser.hpp"

#include <algorithm>
#include <cctype>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace robot_dispatch
{
namespace
{

std::vector<std::string> split_words(const std::string & line)
{
  std::istringstream input(line);
  std::vector<std::string> words;
  std::string word;
  while (input >> word) {
    words.push_back(word);
  }
  return words;
}

ParseResult error(const std::string & message)
{
  ParseResult result;
  result.ok = false;
  result.error = message;
  return result;
}

ParseResult ok(ServiceCallSpec spec)
{
  ParseResult result;
  result.ok = true;
  result.spec = std::move(spec);
  return result;
}

bool is_token(const std::string & token, const std::string & alpha, const std::string & numeric)
{
  const auto normalized = normalize_token(token);
  return normalized == normalize_token(alpha) || normalized == numeric;
}

ServiceCallSpec service_spec(ConsoleAction action, const std::string & service_name)
{
  ServiceCallSpec spec;
  spec.action = action;
  spec.service_name = service_name;
  spec.requires_service = true;
  return spec;
}

}  // namespace

std::string normalize_token(const std::string & token)
{
  std::string normalized;
  normalized.reserve(token.size());
  for (const auto c : token) {
    normalized.push_back(
      static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
  }
  return normalized;
}

std::optional<std::string> parse_confirmation(const std::string & token)
{
  const auto normalized = normalize_token(token);
  if (normalized == "OK" || normalized == "ABNORMAL" ||
      normalized == "REJECT") {
    return normalized;
  }
  return std::nullopt;
}

ParseResult parse_console_command(const std::string & line)
{
  const auto words = split_words(line);
  if (words.empty()) {
    return error("empty command");
  }

  const auto command = normalize_token(words.front());
  if (is_token(words.front(), "HELP", "H") || command == "?") {
    ServiceCallSpec spec;
    spec.action = ConsoleAction::Help;
    return ok(spec);
  }
  if (is_token(words.front(), "QUIT", "0") || command == "EXIT") {
    ServiceCallSpec spec;
    spec.action = ConsoleAction::Quit;
    return ok(spec);
  }
  if (is_token(words.front(), "TASKS", "8")) {
    ServiceCallSpec spec;
    spec.action = ConsoleAction::PrintTasks;
    return ok(spec);
  }
  if (is_token(words.front(), "ROBOTS", "9")) {
    ServiceCallSpec spec;
    spec.action = ConsoleAction::PrintRobots;
    return ok(spec);
  }
  if (is_token(words.front(), "LOCKS", "10")) {
    ServiceCallSpec spec;
    spec.action = ConsoleAction::PrintLocks;
    return ok(spec);
  }
  if (is_token(words.front(), "POINTS", "11")) {
    ServiceCallSpec spec;
    spec.action = ConsoleAction::PrintPoints;
    return ok(spec);
  }
  if (is_token(words.front(), "CLEAR_POINTS", "12") || command == "CLEAR_TEMP_POINTS") {
    auto spec = service_spec(
      ConsoleAction::ClearTemporaryPoints,
      "/robot_dispatch/clear_temporary_task_points");
    return ok(spec);
  }

  if (is_token(words.front(), "DELIVERY", "1") || command == "CREATE_DELIVERY") {
    if (words.size() != 3) {
      return error("delivery expects pickup and delivery point ids");
    }
    auto spec = service_spec(ConsoleAction::CreateTask, "/robot_dispatch/create_task");
    spec.task_type = "DELIVERY";
    spec.point_ids = {words[1], words[2]};
    return ok(spec);
  }

  if (is_token(words.front(), "INSPECTION", "2") || command == "CREATE_INSPECTION") {
    if (words.size() < 2) {
      return error("inspection expects one or more point ids");
    }
    auto spec = service_spec(ConsoleAction::CreateTask, "/robot_dispatch/create_task");
    spec.task_type = "INSPECTION";
    spec.point_ids.assign(words.begin() + 1, words.end());
    return ok(spec);
  }

  if (is_token(words.front(), "CONFIRM", "3")) {
    if (words.size() != 3) {
      return error("confirm expects task_id and OK, ABNORMAL, or REJECT");
    }
    if (!parse_confirmation(words[2]).has_value()) {
      return error("unknown confirmation result: " + words[2]);
    }
    auto spec = service_spec(ConsoleAction::ConfirmTaskStep, "/robot_dispatch/confirm_task_step");
    spec.task_id = words[1];
    spec.confirmation = normalize_token(words[2]);
    return ok(spec);
  }

  if (is_token(words.front(), "PAUSE", "4")) {
    if (words.size() != 2) {
      return error("pause expects task_id");
    }
    auto spec = service_spec(ConsoleAction::PauseTask, "/robot_dispatch/pause_task");
    spec.task_id = words[1];
    return ok(spec);
  }

  if (is_token(words.front(), "RESUME", "5")) {
    if (words.size() != 2) {
      return error("resume expects task_id");
    }
    auto spec = service_spec(ConsoleAction::ResumeTask, "/robot_dispatch/resume_task");
    spec.task_id = words[1];
    return ok(spec);
  }

  if (is_token(words.front(), "CANCEL", "6")) {
    if (words.size() != 2) {
      return error("cancel expects task_id");
    }
    auto spec = service_spec(ConsoleAction::CancelTask, "/robot_dispatch/cancel_task");
    spec.task_id = words[1];
    return ok(spec);
  }

  if (is_token(words.front(), "ESTOP", "7") || command == "EMERGENCY_STOP") {
    if (words.size() > 2) {
      return error("emergency stop expects no target or all");
    }
    if (words.size() == 2 && normalize_token(words[1]) != "ALL") {
      return error("emergency stop currently supports all robots only");
    }
    auto spec = service_spec(ConsoleAction::EmergencyStop, "/robot_dispatch/emergency_stop");
    spec.robot_id = "all";
    return ok(spec);
  }

  return error("unknown command: " + words.front());
}

std::string console_help_text()
{
  return
    "Commands:\n"
    "  1 PICKUP DELIVERY        create DELIVERY task\n"
    "  2 P1 [P2 ...]           create INSPECTION task\n"
    "  3 TASK_ID OK|ABNORMAL|REJECT\n"
    "  4 TASK_ID                pause task\n"
    "  5 TASK_ID                resume task\n"
    "  6 TASK_ID                cancel task\n"
    "  7 [all]                  emergency stop all robots\n"
    "  8                        print task list\n"
    "  9                        print robot states\n"
    "  10                       print resource locks\n"
    "  11                       print task points\n"
    "  12                       clear RViz temporary points\n"
    "  0                        quit\n";
}

}  // namespace robot_dispatch
