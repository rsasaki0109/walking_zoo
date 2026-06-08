#include <gtest/gtest.h>

#include "locomotion_ros2_nav2/legged_velocity_shaper.hpp"

using locomotion_ros2_nav2::LeggedMotionLimits;
using locomotion_ros2_nav2::LeggedVelocityShaper;

namespace
{

LeggedMotionLimits test_limits()
{
  LeggedMotionLimits limits;
  limits.max_forward = 0.6;
  limits.max_backward = 0.3;
  limits.max_lateral = 0.3;
  limits.max_yaw_rate = 0.8;
  limits.max_linear_accel = 1.0;
  limits.max_yaw_accel = 2.0;
  limits.lateral_deadband = 0.05;
  limits.turn_speed_coupling = 0.7;
  return limits;
}

}  // namespace

TEST(LeggedVelocityShaper, PassesThroughFeasibleCommand)
{
  LeggedVelocityShaper shaper(test_limits());
  const auto out = shaper.shape(0.3, 0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(out.vx, 0.3);
  EXPECT_DOUBLE_EQ(out.vy, 0.0);
  EXPECT_DOUBLE_EQ(out.vyaw, 0.0);
  EXPECT_FALSE(out.modified);
}

TEST(LeggedVelocityShaper, ClampsForwardBackwardAsymmetrically)
{
  LeggedVelocityShaper shaper(test_limits());
  EXPECT_DOUBLE_EQ(shaper.shape(5.0, 0.0, 0.0, 0.0).vx, 0.6);
  shaper.reset();
  EXPECT_DOUBLE_EQ(shaper.shape(-5.0, 0.0, 0.0, 0.0).vx, -0.3);
}

TEST(LeggedVelocityShaper, SuppressesTinyLateralStep)
{
  LeggedVelocityShaper shaper(test_limits());
  const auto out = shaper.shape(0.0, 0.02, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(out.vy, 0.0);
  EXPECT_TRUE(out.modified);
}

TEST(LeggedVelocityShaper, KeepsLateralAboveDeadband)
{
  LeggedVelocityShaper shaper(test_limits());
  const auto out = shaper.shape(0.0, 0.2, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(out.vy, 0.2);
}

TEST(LeggedVelocityShaper, SuppressesTinyYawCommand)
{
  auto limits = test_limits();
  limits.yaw_deadband = 0.08;
  LeggedVelocityShaper shaper(limits);
  const auto out = shaper.shape(0.3, 0.0, 0.03, 0.0);
  EXPECT_DOUBLE_EQ(out.vyaw, 0.0);
  EXPECT_TRUE(out.modified);
}

TEST(LeggedVelocityShaper, CutsForwardSpeedWhenTurningHard)
{
  LeggedVelocityShaper shaper(test_limits());
  // Full yaw rate with coupling 0.7 should shed 70% of forward speed.
  const auto out = shaper.shape(0.5, 0.0, 0.8, 0.0);
  EXPECT_NEAR(out.vx, 0.5 * (1.0 - 0.7), 1e-9);
  EXPECT_DOUBLE_EQ(out.vyaw, 0.8);
  EXPECT_TRUE(out.modified);
}

TEST(LeggedVelocityShaper, RateLimitsAcceleration)
{
  LeggedVelocityShaper shaper(test_limits());
  shaper.shape(0.0, 0.0, 0.0, 0.0);  // establish zero baseline
  // Demand 0.6 m/s with only 0.1 s elapsed: max delta = 1.0 * 0.1 = 0.1.
  const auto out = shaper.shape(0.6, 0.0, 0.0, 0.1);
  EXPECT_NEAR(out.vx, 0.1, 1e-9);
  EXPECT_TRUE(out.modified);
}

TEST(LeggedVelocityShaper, NoAccelLimitWhenDtZero)
{
  LeggedVelocityShaper shaper(test_limits());
  shaper.shape(0.0, 0.0, 0.0, 0.0);
  const auto out = shaper.shape(0.6, 0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(out.vx, 0.6);
}

TEST(LeggedVelocityShaper, ResetClearsRateHistory)
{
  LeggedVelocityShaper shaper(test_limits());
  shaper.shape(0.5, 0.0, 0.0, 0.1);
  shaper.reset();
  // After reset, the next call has no previous command to rate-limit against.
  const auto out = shaper.shape(0.6, 0.0, 0.0, 0.1);
  EXPECT_DOUBLE_EQ(out.vx, 0.6);
}
