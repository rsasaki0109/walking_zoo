#ifndef LOCOMOTION_ROS2_SAFETY__COMMAND_WATCHDOG_HPP_
#define LOCOMOTION_ROS2_SAFETY__COMMAND_WATCHDOG_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/time.hpp"

namespace locomotion_ros2_safety
{

class CommandWatchdog
{
public:
  explicit CommandWatchdog(double timeout_sec = 0.25);

  bool is_stale(const builtin_interfaces::msg::Time & stamp, const rclcpp::Time & now) const;
  double timeout_sec() const;
  void set_timeout_sec(double timeout_sec);

private:
  double timeout_sec_{0.25};
};

}  // namespace locomotion_ros2_safety

#endif  // LOCOMOTION_ROS2_SAFETY__COMMAND_WATCHDOG_HPP_
