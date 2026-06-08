#ifndef LOCOMOTION_ROS2_RUNTIME__COMMAND_ARBITER_HPP_
#define LOCOMOTION_ROS2_RUNTIME__COMMAND_ARBITER_HPP_

#include <cstdint>
#include <string>

#include "locomotion_ros2_core/types.hpp"

namespace locomotion_ros2_runtime
{

class CommandArbiter
{
public:
  std::uint8_t priority_for_source(const std::string & source) const;
  bool should_replace(const std::string & current_source, const std::string & candidate_source) const;
};

}  // namespace locomotion_ros2_runtime

#endif  // LOCOMOTION_ROS2_RUNTIME__COMMAND_ARBITER_HPP_
