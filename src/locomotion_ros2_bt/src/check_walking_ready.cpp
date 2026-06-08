#include "locomotion_ros2_bt/check_walking_ready.hpp"

namespace locomotion_ros2_bt
{

bool CheckWalkingReady::tick(const locomotion_ros2_msgs::msg::WalkingState & state) const
{
  return state.adapter_connected &&
         state.is_balanced &&
         !state.is_fallen &&
         !state.estop_active &&
         (state.locomotion_state == locomotion_ros2_msgs::msg::WalkingState::STATE_STANDING ||
         state.locomotion_state == locomotion_ros2_msgs::msg::WalkingState::STATE_IDLE);
}

}  // namespace locomotion_ros2_bt
