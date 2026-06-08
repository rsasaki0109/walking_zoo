#include "locomotion_ros2_safety/safety_pipeline.hpp"

#include <cmath>

namespace locomotion_ros2_safety
{

SafetyPipeline::SafetyPipeline() = default;

SafetyResult SafetyPipeline::filter_velocity(
  const geometry_msgs::msg::TwistStamped & command,
  const rclcpp::Time & now) const
{
  if (!estop_gate_.permits_motion()) {
    return {locomotion_ros2_core::CommandResult::blocked("estop active"), command};
  }

  if (watchdog_.is_stale(command.header.stamp, now)) {
    return {locomotion_ros2_core::CommandResult::blocked("command stale"), command};
  }

  const bool limited = velocity_limiter_.would_limit(command);
  const auto sanitized = velocity_limiter_.clamp(command);
  if (limited) {
    return {locomotion_ros2_core::CommandResult::limited("velocity limited"), sanitized};
  }
  return {locomotion_ros2_core::CommandResult::success("safety passed"), sanitized};
}

BodyPoseSafetyResult SafetyPipeline::filter_body_pose(
  const locomotion_ros2_msgs::msg::BodyPoseCommand & command) const
{
  if (!estop_gate_.permits_motion()) {
    return {locomotion_ros2_core::CommandResult::blocked("estop active"), command};
  }

  // Reject gross over-tilt on the raw request: a pose that would put the torso
  // into the fall band is unsafe no matter how the axes are clamped.
  if (fall_detector_.classify(command.roll, command.pitch) == FallState::FALLEN) {
    return {
      locomotion_ros2_core::CommandResult::rejected("body pose tilt exceeds fall threshold"),
      command};
  }

  auto clamped = command;
  bool limited = false;
  if (std::abs(command.roll) > max_body_roll_) {
    clamped.roll = std::copysign(static_cast<float>(max_body_roll_), command.roll);
    limited = true;
  }
  if (std::abs(command.pitch) > max_body_pitch_) {
    clamped.pitch = std::copysign(static_cast<float>(max_body_pitch_), command.pitch);
    limited = true;
  }
  if (limited) {
    return {locomotion_ros2_core::CommandResult::limited("body pose clamped to safe tilt"), clamped};
  }
  return {locomotion_ros2_core::CommandResult::success("safety passed"), clamped};
}

locomotion_ros2_msgs::msg::SafetyState SafetyPipeline::make_state_msg() const
{
  locomotion_ros2_msgs::msg::SafetyState state;
  state.state = estop_gate_.active() ?
    locomotion_ros2_msgs::msg::SafetyState::STATE_ESTOPPED :
    locomotion_ros2_msgs::msg::SafetyState::STATE_OK;
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

void SafetyPipeline::set_body_pose_limits(double max_roll_rad, double max_pitch_rad)
{
  max_body_roll_ = std::abs(max_roll_rad);
  max_body_pitch_ = std::abs(max_pitch_rad);
}

void SafetyPipeline::set_fall_thresholds(double tilt_warn_rad, double tilt_fall_rad)
{
  fall_detector_ = FallDetector(tilt_warn_rad, tilt_fall_rad);
}

FallState SafetyPipeline::classify_tilt(double roll, double pitch) const
{
  return fall_detector_.classify(roll, pitch);
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

}  // namespace locomotion_ros2_safety
