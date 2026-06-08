#include "locomotion_ros2_runtime/mode_manager.hpp"

namespace locomotion_ros2_runtime
{

bool ModeManager::set_mode(std::uint8_t mode)
{
  switch (mode) {
    case locomotion_ros2_msgs::msg::WalkingState::MODE_IDLE:
    case locomotion_ros2_msgs::msg::WalkingState::MODE_STAND:
    case locomotion_ros2_msgs::msg::WalkingState::MODE_WALK:
    case locomotion_ros2_msgs::msg::WalkingState::MODE_BODY_POSE:
    case locomotion_ros2_msgs::msg::WalkingState::MODE_FOOTSTEP:
    case locomotion_ros2_msgs::msg::WalkingState::MODE_SEMANTIC:
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

}  // namespace locomotion_ros2_runtime
