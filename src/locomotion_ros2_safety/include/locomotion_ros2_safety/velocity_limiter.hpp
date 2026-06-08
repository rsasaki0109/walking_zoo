#ifndef LOCOMOTION_ROS2_SAFETY__VELOCITY_LIMITER_HPP_
#define LOCOMOTION_ROS2_SAFETY__VELOCITY_LIMITER_HPP_

#include "geometry_msgs/msg/twist_stamped.hpp"

namespace locomotion_ros2_safety
{

struct VelocityLimits
{
  double max_linear_x{0.3};
  double max_linear_y{0.2};
  double max_angular_z{0.5};
};

class VelocityLimiter
{
public:
  explicit VelocityLimiter(VelocityLimits limits = {});

  geometry_msgs::msg::TwistStamped clamp(
    const geometry_msgs::msg::TwistStamped & command) const;

  bool would_limit(const geometry_msgs::msg::TwistStamped & command) const;
  const VelocityLimits & limits() const;
  void set_limits(const VelocityLimits & limits);

private:
  VelocityLimits limits_;
};

}  // namespace locomotion_ros2_safety

#endif  // LOCOMOTION_ROS2_SAFETY__VELOCITY_LIMITER_HPP_
