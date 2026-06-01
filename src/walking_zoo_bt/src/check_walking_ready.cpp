#include "walking_zoo_bt/check_walking_ready.hpp"

namespace walking_zoo_bt
{

bool CheckWalkingReady::tick(const walking_zoo_msgs::msg::WalkingState & state) const
{
  return state.adapter_connected &&
         state.is_balanced &&
         !state.is_fallen &&
         !state.estop_active &&
         (state.locomotion_state == walking_zoo_msgs::msg::WalkingState::STATE_STANDING ||
         state.locomotion_state == walking_zoo_msgs::msg::WalkingState::STATE_IDLE);
}

}  // namespace walking_zoo_bt
