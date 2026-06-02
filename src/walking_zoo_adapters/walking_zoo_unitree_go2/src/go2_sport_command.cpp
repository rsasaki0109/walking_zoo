#include "walking_zoo_unitree_go2/go2_sport_command.hpp"

#include <algorithm>
#include <cmath>

namespace walking_zoo_unitree_go2
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

Go2VelocityCommand translate_velocity(
  const geometry_msgs::msg::TwistStamped & cmd,
  const Go2VelocityLimits & limits)
{
  Go2VelocityCommand out;
  out.vx = clamp_flag(
    cmd.twist.linear.x, -std::abs(limits.max_backward), std::abs(limits.max_forward), out.clamped);
  out.vy = clamp_flag(
    cmd.twist.linear.y, -std::abs(limits.max_lateral), std::abs(limits.max_lateral), out.clamped);
  out.vyaw = clamp_flag(
    cmd.twist.angular.z, -std::abs(limits.max_yaw_rate), std::abs(limits.max_yaw_rate),
    out.clamped);
  return out;
}

Go2PostureCommand translate_body_pose(
  const walking_zoo_msgs::msg::BodyPoseCommand & cmd,
  const Go2PostureLimits & limits)
{
  Go2PostureCommand out;
  out.roll = clamp_flag(cmd.roll, -std::abs(limits.max_roll), std::abs(limits.max_roll), out.clamped);
  out.pitch =
    clamp_flag(cmd.pitch, -std::abs(limits.max_pitch), std::abs(limits.max_pitch), out.clamped);
  out.yaw = clamp_flag(cmd.yaw, -std::abs(limits.max_yaw), std::abs(limits.max_yaw), out.clamped);
  out.height = clamp_flag(cmd.body_height, limits.min_height, limits.max_height, out.clamped);
  return out;
}

bool sport_transition_allowed(SportMode current, SportMode target)
{
  switch (target) {
    case SportMode::DAMP:
      // Always reachable: emergency damp must never be gated.
      return true;
    case SportMode::STAND_DOWN:
      // Lie back down from a stand or trot; from damp the robot must recover
      // (stand up) before it can be commanded to lie down cleanly.
      return current == SportMode::BALANCE_STAND || current == SportMode::LOCOMOTION ||
             current == SportMode::STAND_DOWN;
    case SportMode::BALANCE_STAND:
      // The Go2 can recovery-stand into balance-stand from any state, including
      // damp (self-righting) and a lying STAND_DOWN.
      return true;
    case SportMode::LOCOMOTION:
      // Trotting requires already standing and balanced.
      return current == SportMode::BALANCE_STAND || current == SportMode::LOCOMOTION;
  }
  return false;
}

const char * to_string(SportMode mode)
{
  switch (mode) {
    case SportMode::DAMP:
      return "damp";
    case SportMode::STAND_DOWN:
      return "stand_down";
    case SportMode::BALANCE_STAND:
      return "balance_stand";
    case SportMode::LOCOMOTION:
      return "locomotion";
  }
  return "unknown";
}

}  // namespace walking_zoo_unitree_go2
