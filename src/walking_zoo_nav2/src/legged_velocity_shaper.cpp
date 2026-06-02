#include "walking_zoo_nav2/legged_velocity_shaper.hpp"

#include <algorithm>
#include <cmath>

namespace walking_zoo_nav2
{

namespace
{

// Limit how far `value` may move from `previous` given a maximum rate and dt.
double rate_limit(double value, double previous, double max_rate, double dt)
{
  if (dt <= 0.0 || max_rate <= 0.0) {
    return value;
  }
  const double max_delta = max_rate * dt;
  const double delta = std::clamp(value - previous, -max_delta, max_delta);
  return previous + delta;
}

}  // namespace

LeggedVelocityShaper::LeggedVelocityShaper(const LeggedMotionLimits & limits)
: limits_(limits)
{
}

void LeggedVelocityShaper::set_limits(const LeggedMotionLimits & limits)
{
  limits_ = limits;
}

const LeggedMotionLimits & LeggedVelocityShaper::limits() const
{
  return limits_;
}

void LeggedVelocityShaper::reset()
{
  last_vx_ = 0.0;
  last_vy_ = 0.0;
  last_vyaw_ = 0.0;
  has_last_ = false;
}

ShapedVelocity LeggedVelocityShaper::shape(double vx, double vy, double vyaw, double dt)
{
  const double raw_vx = vx;
  const double raw_vy = vy;
  const double raw_vyaw = vyaw;

  // 1. Asymmetric per-axis clamp: forward and backward limits differ.
  vx = std::clamp(vx, -std::abs(limits_.max_backward), std::abs(limits_.max_forward));
  vy = std::clamp(vy, -std::abs(limits_.max_lateral), std::abs(limits_.max_lateral));
  vyaw = std::clamp(vyaw, -std::abs(limits_.max_yaw_rate), std::abs(limits_.max_yaw_rate));

  // 2. Lateral deadband: walkers cannot realise very small side-steps cleanly.
  if (std::abs(vy) < std::abs(limits_.lateral_deadband)) {
    vy = 0.0;
  }

  // 3. Turn/forward coupling: shed forward speed when turning hard so the gait
  //    can keep its footing instead of being asked to arc fast.
  if (limits_.max_yaw_rate > 0.0 && limits_.turn_speed_coupling > 0.0) {
    const double turn_ratio = std::min(1.0, std::abs(vyaw) / std::abs(limits_.max_yaw_rate));
    const double scale = std::max(0.0, 1.0 - limits_.turn_speed_coupling * turn_ratio);
    vx *= scale;
  }

  // 4. Acceleration (rate) limiting against the previously emitted command.
  if (has_last_ && dt > 0.0) {
    vx = rate_limit(vx, last_vx_, limits_.max_linear_accel, dt);
    vy = rate_limit(vy, last_vy_, limits_.max_linear_accel, dt);
    vyaw = rate_limit(vyaw, last_vyaw_, limits_.max_yaw_accel, dt);
  }

  last_vx_ = vx;
  last_vy_ = vy;
  last_vyaw_ = vyaw;
  has_last_ = true;

  ShapedVelocity out;
  out.vx = vx;
  out.vy = vy;
  out.vyaw = vyaw;
  out.modified =
    std::abs(vx - raw_vx) > 1e-9 || std::abs(vy - raw_vy) > 1e-9 ||
    std::abs(vyaw - raw_vyaw) > 1e-9;
  return out;
}

}  // namespace walking_zoo_nav2
