#include "walking_zoo_unitree_sdk2/unitree_loco_command.hpp"

#include <algorithm>
#include <cmath>

namespace walking_zoo_unitree_sdk2
{

namespace
{

// Clamp `value` into [lo, hi], flagging whether the clamp changed it.
double clamp_flag(double value, double lo, double hi, bool & clamped)
{
  const double out = std::clamp(value, lo, hi);
  if (std::abs(out - value) > 1e-9) {
    clamped = true;
  }
  return out;
}

}  // namespace

LocoVelocityCommand translate_velocity(
  const geometry_msgs::msg::TwistStamped & cmd,
  const G1VelocityLimits & limits)
{
  LocoVelocityCommand out;
  out.vx = clamp_flag(
    cmd.twist.linear.x, -std::abs(limits.max_backward), std::abs(limits.max_forward), out.clamped);
  out.vy = clamp_flag(
    cmd.twist.linear.y, -std::abs(limits.max_lateral), std::abs(limits.max_lateral), out.clamped);
  out.vyaw = clamp_flag(
    cmd.twist.angular.z, -std::abs(limits.max_yaw_rate), std::abs(limits.max_yaw_rate),
    out.clamped);
  return out;
}

LocoPostureCommand translate_body_pose(
  const walking_zoo_msgs::msg::BodyPoseCommand & cmd,
  const G1PostureLimits & limits)
{
  LocoPostureCommand out;
  out.roll = clamp_flag(cmd.roll, -std::abs(limits.max_roll), std::abs(limits.max_roll), out.clamped);
  out.pitch =
    clamp_flag(cmd.pitch, -std::abs(limits.max_pitch), std::abs(limits.max_pitch), out.clamped);
  out.yaw = clamp_flag(cmd.yaw, -std::abs(limits.max_yaw), std::abs(limits.max_yaw), out.clamped);
  out.height = clamp_flag(cmd.body_height, limits.min_height, limits.max_height, out.clamped);
  return out;
}

bool loco_transition_allowed(LocoMode current, LocoMode target)
{
  switch (target) {
    case LocoMode::ZERO_TORQUE:
    case LocoMode::DAMP:
      // Safety states are always reachable.
      return true;
    case LocoMode::BALANCE_STAND:
      // Can stand up from any non-locomoting state, or settle from locomotion.
      return true;
    case LocoMode::LOCOMOTION:
      // The G1 FSM requires balance-stand before it will track velocities.
      return current == LocoMode::BALANCE_STAND || current == LocoMode::LOCOMOTION;
  }
  return false;
}

const char * to_string(LocoMode mode)
{
  switch (mode) {
    case LocoMode::ZERO_TORQUE:
      return "zero_torque";
    case LocoMode::DAMP:
      return "damp";
    case LocoMode::BALANCE_STAND:
      return "balance_stand";
    case LocoMode::LOCOMOTION:
      return "locomotion";
  }
  return "unknown";
}

}  // namespace walking_zoo_unitree_sdk2
