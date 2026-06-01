#include "walking_zoo_safety/command_watchdog.hpp"

#include <algorithm>

namespace walking_zoo_safety
{

CommandWatchdog::CommandWatchdog(double timeout_sec)
: timeout_sec_(std::max(0.0, timeout_sec))
{
}

bool CommandWatchdog::is_stale(
  const builtin_interfaces::msg::Time & stamp,
  const rclcpp::Time & now) const
{
  if (timeout_sec_ <= 0.0) {
    return false;
  }
  const rclcpp::Time command_time(stamp);
  if (command_time.nanoseconds() == 0) {
    return false;
  }
  return (now - command_time).seconds() > timeout_sec_;
}

double CommandWatchdog::timeout_sec() const
{
  return timeout_sec_;
}

void CommandWatchdog::set_timeout_sec(double timeout_sec)
{
  timeout_sec_ = std::max(0.0, timeout_sec);
}

}  // namespace walking_zoo_safety
