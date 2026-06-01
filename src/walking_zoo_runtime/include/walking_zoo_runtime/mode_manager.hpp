#ifndef WALKING_ZOO_RUNTIME__MODE_MANAGER_HPP_
#define WALKING_ZOO_RUNTIME__MODE_MANAGER_HPP_

#include <cstdint>

#include "walking_zoo_msgs/msg/walking_state.hpp"

namespace walking_zoo_runtime
{

class ModeManager
{
public:
  bool set_mode(std::uint8_t mode);
  std::uint8_t mode() const;

private:
  std::uint8_t mode_{walking_zoo_msgs::msg::WalkingState::MODE_IDLE};
};

}  // namespace walking_zoo_runtime

#endif  // WALKING_ZOO_RUNTIME__MODE_MANAGER_HPP_
