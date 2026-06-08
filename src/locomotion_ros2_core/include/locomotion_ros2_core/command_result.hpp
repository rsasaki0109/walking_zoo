#ifndef LOCOMOTION_ROS2_CORE__COMMAND_RESULT_HPP_
#define LOCOMOTION_ROS2_CORE__COMMAND_RESULT_HPP_

#include <string>

#include "locomotion_ros2_core/types.hpp"

namespace locomotion_ros2_core
{

struct CommandResult
{
  CommandStatus status{CommandStatus::ACCEPTED};
  bool accepted{true};
  std::string message{"accepted"};

  static CommandResult success(const std::string & message = "accepted")
  {
    return {CommandStatus::ACCEPTED, true, message};
  }

  static CommandResult limited(const std::string & message = "limited")
  {
    return {CommandStatus::LIMITED, true, message};
  }

  static CommandResult blocked(const std::string & message = "blocked")
  {
    return {CommandStatus::BLOCKED, false, message};
  }

  static CommandResult rejected(const std::string & message = "rejected")
  {
    return {CommandStatus::REJECTED, false, message};
  }

  static CommandResult error(const std::string & message = "error")
  {
    return {CommandStatus::ERROR, false, message};
  }
};

}  // namespace locomotion_ros2_core

#endif  // LOCOMOTION_ROS2_CORE__COMMAND_RESULT_HPP_
