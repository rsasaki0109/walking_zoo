#ifndef LOCOMOTION_ROS2_RUNTIME__MODE_MANAGER_HPP_
#define LOCOMOTION_ROS2_RUNTIME__MODE_MANAGER_HPP_

#include <cstdint>

#include "locomotion_ros2_msgs/msg/walking_state.hpp"

namespace locomotion_ros2_runtime
{

class ModeManager
{
public:
  bool set_mode(std::uint8_t mode);
  std::uint8_t mode() const;

private:
  std::uint8_t mode_{locomotion_ros2_msgs::msg::WalkingState::MODE_IDLE};
};

}  // namespace locomotion_ros2_runtime

#endif  // LOCOMOTION_ROS2_RUNTIME__MODE_MANAGER_HPP_
