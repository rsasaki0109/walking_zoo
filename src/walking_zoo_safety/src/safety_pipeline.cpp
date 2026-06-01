#include "walking_zoo_safety/safety_pipeline.hpp"

namespace walking_zoo_safety
{

SafetyPipeline::SafetyPipeline() = default;

SafetyResult SafetyPipeline::filter_velocity(
  const geometry_msgs::msg::TwistStamped & command,
  const rclcpp::Time & now) const
{
  if (!estop_gate_.permits_motion()) {
    return {walking_zoo_core::CommandResult::blocked("estop active"), command};
  }

  if (watchdog_.is_stale(command.header.stamp, now)) {
    return {walking_zoo_core::CommandResult::blocked("command stale"), command};
  }

  const bool limited = velocity_limiter_.would_limit(command);
  const auto sanitized = velocity_limiter_.clamp(command);
  if (limited) {
    return {walking_zoo_core::CommandResult::limited("velocity limited"), sanitized};
  }
  return {walking_zoo_core::CommandResult::success("safety passed"), sanitized};
}

walking_zoo_msgs::msg::SafetyState SafetyPipeline::make_state_msg() const
{
  walking_zoo_msgs::msg::SafetyState state;
  state.state = estop_gate_.active() ?
    walking_zoo_msgs::msg::SafetyState::STATE_ESTOPPED :
    walking_zoo_msgs::msg::SafetyState::STATE_OK;
  state.estop_active = estop_gate_.active();
  state.command_stale = false;
  state.fall_detected = false;
  state.adapter_healthy = true;
  state.max_linear_x = static_cast<float>(velocity_limiter_.limits().max_linear_x);
  state.max_linear_y = static_cast<float>(velocity_limiter_.limits().max_linear_y);
  state.max_angular_z = static_cast<float>(velocity_limiter_.limits().max_angular_z);
  state.status_text = estop_gate_.active() ? "estop active" : "safety ok";
  return state;
}

void SafetyPipeline::set_limits(const VelocityLimits & limits)
{
  velocity_limiter_.set_limits(limits);
}

void SafetyPipeline::set_estop_active(bool active)
{
  estop_gate_.set_active(active);
}

bool SafetyPipeline::estop_active() const
{
  return estop_gate_.active();
}

void SafetyPipeline::set_command_timeout_sec(double timeout_sec)
{
  watchdog_.set_timeout_sec(timeout_sec);
}

}  // namespace walking_zoo_safety
