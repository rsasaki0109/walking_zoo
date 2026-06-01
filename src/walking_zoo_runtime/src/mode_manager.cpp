#include "walking_zoo_runtime/mode_manager.hpp"

namespace walking_zoo_runtime
{

bool ModeManager::set_mode(std::uint8_t mode)
{
  switch (mode) {
    case walking_zoo_msgs::msg::WalkingState::MODE_IDLE:
    case walking_zoo_msgs::msg::WalkingState::MODE_STAND:
    case walking_zoo_msgs::msg::WalkingState::MODE_WALK:
    case walking_zoo_msgs::msg::WalkingState::MODE_BODY_POSE:
    case walking_zoo_msgs::msg::WalkingState::MODE_FOOTSTEP:
    case walking_zoo_msgs::msg::WalkingState::MODE_SEMANTIC:
      mode_ = mode;
      return true;
    default:
      return false;
  }
}

std::uint8_t ModeManager::mode() const
{
  return mode_;
}

}  // namespace walking_zoo_runtime
