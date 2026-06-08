#ifndef LOCOMOTION_ROS2_BT__CLEAR_WALKING_FAULT_HPP_
#define LOCOMOTION_ROS2_BT__CLEAR_WALKING_FAULT_HPP_

namespace locomotion_ros2_bt
{

class ClearWalkingFault
{
public:
  bool tick(bool clear_fault_service_succeeded) const;
};

}  // namespace locomotion_ros2_bt

#endif  // LOCOMOTION_ROS2_BT__CLEAR_WALKING_FAULT_HPP_
