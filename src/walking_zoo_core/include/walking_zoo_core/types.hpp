#ifndef WALKING_ZOO_CORE__TYPES_HPP_
#define WALKING_ZOO_CORE__TYPES_HPP_

#include <cstdint>
#include <string>

namespace walking_zoo_core
{

enum class CallbackReturn : std::uint8_t
{
  SUCCESS = 0,
  FAILURE = 1,
  ERROR = 2
};

enum class StopMode : std::uint8_t
{
  NORMAL = 0,
  QUICK = 1,
  EMERGENCY = 2
};

enum class CommandStatus : std::uint8_t
{
  ACCEPTED = 0,
  REJECTED = 1,
  LIMITED = 2,
  BLOCKED = 3,
  ERROR = 4
};

enum class CommandSourcePriority : std::uint8_t
{
  BACKGROUND = 0,
  VLA_SEMANTIC_ACTION = 1,
  NAV2 = 2,
  OPERATOR_OVERRIDE = 3,
  SAFETY_SUPERVISOR = 4,
  EMERGENCY_STOP = 5
};

std::string to_string(CallbackReturn value);
std::string to_string(StopMode value);
std::string to_string(CommandStatus value);

}  // namespace walking_zoo_core

#endif  // WALKING_ZOO_CORE__TYPES_HPP_
