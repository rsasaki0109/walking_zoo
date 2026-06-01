#include "walking_zoo_runtime/command_arbiter.hpp"

namespace walking_zoo_runtime
{

std::uint8_t CommandArbiter::priority_for_source(const std::string & source) const
{
  if (source == "estop" || source == "emergency_stop") {
    return static_cast<std::uint8_t>(walking_zoo_core::CommandSourcePriority::EMERGENCY_STOP);
  }
  if (source == "safety" || source == "fall_recovery") {
    return static_cast<std::uint8_t>(walking_zoo_core::CommandSourcePriority::SAFETY_SUPERVISOR);
  }
  if (source == "operator" || source == "teleop" || source == "manual") {
    return static_cast<std::uint8_t>(walking_zoo_core::CommandSourcePriority::OPERATOR_OVERRIDE);
  }
  if (source == "nav2" || source == "/cmd_vel") {
    return static_cast<std::uint8_t>(walking_zoo_core::CommandSourcePriority::NAV2);
  }
  if (source == "vla" || source == "semantic_action") {
    return static_cast<std::uint8_t>(walking_zoo_core::CommandSourcePriority::VLA_SEMANTIC_ACTION);
  }
  return static_cast<std::uint8_t>(walking_zoo_core::CommandSourcePriority::BACKGROUND);
}

bool CommandArbiter::should_replace(
  const std::string & current_source,
  const std::string & candidate_source) const
{
  return priority_for_source(candidate_source) >= priority_for_source(current_source);
}

}  // namespace walking_zoo_runtime
