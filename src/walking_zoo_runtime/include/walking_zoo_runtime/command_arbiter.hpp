#ifndef WALKING_ZOO_RUNTIME__COMMAND_ARBITER_HPP_
#define WALKING_ZOO_RUNTIME__COMMAND_ARBITER_HPP_

#include <cstdint>
#include <string>

#include "walking_zoo_core/types.hpp"

namespace walking_zoo_runtime
{

class CommandArbiter
{
public:
  std::uint8_t priority_for_source(const std::string & source) const;
  bool should_replace(const std::string & current_source, const std::string & candidate_source) const;
};

}  // namespace walking_zoo_runtime

#endif  // WALKING_ZOO_RUNTIME__COMMAND_ARBITER_HPP_
