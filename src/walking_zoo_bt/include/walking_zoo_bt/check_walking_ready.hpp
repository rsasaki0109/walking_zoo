#ifndef WALKING_ZOO_BT__CHECK_WALKING_READY_HPP_
#define WALKING_ZOO_BT__CHECK_WALKING_READY_HPP_

#include "walking_zoo_msgs/msg/walking_state.hpp"

namespace walking_zoo_bt
{

class CheckWalkingReady
{
public:
  bool tick(const walking_zoo_msgs::msg::WalkingState & state) const;
};

}  // namespace walking_zoo_bt

#endif  // WALKING_ZOO_BT__CHECK_WALKING_READY_HPP_
