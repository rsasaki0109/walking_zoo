#ifndef LOCOMOTION_ROS2_BT__CHECK_WALKING_READY_HPP_
#define LOCOMOTION_ROS2_BT__CHECK_WALKING_READY_HPP_

#include "locomotion_ros2_msgs/msg/walking_state.hpp"

namespace locomotion_ros2_bt
{

class CheckWalkingReady
{
public:
  bool tick(const locomotion_ros2_msgs::msg::WalkingState & state) const;
};

}  // namespace locomotion_ros2_bt

#endif  // LOCOMOTION_ROS2_BT__CHECK_WALKING_READY_HPP_
