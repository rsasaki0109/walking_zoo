#include <gtest/gtest.h>

#include "walking_zoo_safety/velocity_limiter.hpp"

TEST(VelocityLimiter, ClampsConservativeLimits)
{
  walking_zoo_safety::VelocityLimiter limiter({0.5, 0.3, 0.8});
  geometry_msgs::msg::TwistStamped command;
  command.twist.linear.x = 1.2;
  command.twist.linear.y = -0.9;
  command.twist.angular.z = 2.0;

  const auto sanitized = limiter.clamp(command);

  EXPECT_DOUBLE_EQ(sanitized.twist.linear.x, 0.5);
  EXPECT_DOUBLE_EQ(sanitized.twist.linear.y, -0.3);
  EXPECT_DOUBLE_EQ(sanitized.twist.angular.z, 0.8);
  EXPECT_TRUE(limiter.would_limit(command));
}
