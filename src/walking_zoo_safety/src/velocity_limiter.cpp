#include "walking_zoo_safety/velocity_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace walking_zoo_safety
{

namespace
{
double clamp_symmetric(double value, double limit)
{
  const double safe_limit = std::max(0.0, limit);
  return std::clamp(value, -safe_limit, safe_limit);
}
}  // namespace

VelocityLimiter::VelocityLimiter(VelocityLimits limits)
: limits_(limits)
{
}

geometry_msgs::msg::TwistStamped VelocityLimiter::clamp(
  const geometry_msgs::msg::TwistStamped & command) const
{
  auto sanitized = command;
  sanitized.twist.linear.x = clamp_symmetric(command.twist.linear.x, limits_.max_linear_x);
  sanitized.twist.linear.y = clamp_symmetric(command.twist.linear.y, limits_.max_linear_y);
  sanitized.twist.angular.z = clamp_symmetric(command.twist.angular.z, limits_.max_angular_z);
  return sanitized;
}

bool VelocityLimiter::would_limit(const geometry_msgs::msg::TwistStamped & command) const
{
  const auto sanitized = clamp(command);
  return std::abs(sanitized.twist.linear.x - command.twist.linear.x) > 1e-9 ||
         std::abs(sanitized.twist.linear.y - command.twist.linear.y) > 1e-9 ||
         std::abs(sanitized.twist.angular.z - command.twist.angular.z) > 1e-9;
}

const VelocityLimits & VelocityLimiter::limits() const
{
  return limits_;
}

void VelocityLimiter::set_limits(const VelocityLimits & limits)
{
  limits_ = limits;
}

}  // namespace walking_zoo_safety
