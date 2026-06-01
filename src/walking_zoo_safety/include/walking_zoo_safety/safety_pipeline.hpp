#ifndef WALKING_ZOO_SAFETY__SAFETY_PIPELINE_HPP_
#define WALKING_ZOO_SAFETY__SAFETY_PIPELINE_HPP_

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/time.hpp"
#include "walking_zoo_core/command_result.hpp"
#include "walking_zoo_msgs/msg/safety_state.hpp"
#include "walking_zoo_safety/command_watchdog.hpp"
#include "walking_zoo_safety/estop_gate.hpp"
#include "walking_zoo_safety/velocity_limiter.hpp"

namespace walking_zoo_safety
{

struct SafetyResult
{
  walking_zoo_core::CommandResult result;
  geometry_msgs::msg::TwistStamped command;
};

class SafetyPipeline
{
public:
  SafetyPipeline();

  SafetyResult filter_velocity(
    const geometry_msgs::msg::TwistStamped & command,
    const rclcpp::Time & now) const;

  walking_zoo_msgs::msg::SafetyState make_state_msg() const;

  void set_limits(const VelocityLimits & limits);
  void set_estop_active(bool active);
  bool estop_active() const;
  void set_command_timeout_sec(double timeout_sec);

private:
  VelocityLimiter velocity_limiter_;
  EStopGate estop_gate_;
  CommandWatchdog watchdog_;
};

}  // namespace walking_zoo_safety

#endif  // WALKING_ZOO_SAFETY__SAFETY_PIPELINE_HPP_
