#include <gtest/gtest.h>

#include "walking_zoo_unitree_go2/go2_sport_command.hpp"

using walking_zoo_unitree_go2::Go2PostureLimits;
using walking_zoo_unitree_go2::Go2VelocityLimits;
using walking_zoo_unitree_go2::SportMode;
using walking_zoo_unitree_go2::sport_transition_allowed;
using walking_zoo_unitree_go2::translate_body_pose;
using walking_zoo_unitree_go2::translate_velocity;

namespace
{

geometry_msgs::msg::TwistStamped twist(double x, double y, double wz)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.twist.linear.x = x;
  cmd.twist.linear.y = y;
  cmd.twist.angular.z = wz;
  return cmd;
}

}  // namespace

TEST(Go2SportCommand, VelocityWithinEnvelopeIsUnchanged)
{
  Go2VelocityLimits limits;
  const auto out = translate_velocity(twist(0.3, 0.1, 0.2), limits);
  EXPECT_DOUBLE_EQ(out.vx, 0.3);
  EXPECT_DOUBLE_EQ(out.vy, 0.1);
  EXPECT_DOUBLE_EQ(out.vyaw, 0.2);
  EXPECT_FALSE(out.clamped);
}

TEST(Go2SportCommand, VelocityClampsAsymmetricForwardBackward)
{
  Go2VelocityLimits limits;  // forward 0.6, backward 0.4
  const auto fwd = translate_velocity(twist(5.0, 0.0, 0.0), limits);
  EXPECT_DOUBLE_EQ(fwd.vx, 0.6);
  EXPECT_TRUE(fwd.clamped);

  const auto back = translate_velocity(twist(-5.0, 0.0, 0.0), limits);
  EXPECT_DOUBLE_EQ(back.vx, -0.4);
  EXPECT_TRUE(back.clamped);
}

TEST(Go2SportCommand, BodyPoseClampsToEnvelope)
{
  Go2PostureLimits limits;
  walking_zoo_msgs::msg::BodyPoseCommand cmd;
  cmd.roll = 5.0F;
  cmd.pitch = -5.0F;
  cmd.yaw = 5.0F;
  cmd.body_height = -5.0F;
  const auto out = translate_body_pose(cmd, limits);
  EXPECT_DOUBLE_EQ(out.roll, limits.max_roll);
  EXPECT_DOUBLE_EQ(out.pitch, -limits.max_pitch);
  EXPECT_DOUBLE_EQ(out.yaw, limits.max_yaw);
  EXPECT_DOUBLE_EQ(out.height, limits.min_height);
  EXPECT_TRUE(out.clamped);
}

TEST(Go2SportCommand, TrotRequiresStanding)
{
  // Cannot trot straight from a lying rest pose; must stand up first.
  EXPECT_FALSE(sport_transition_allowed(SportMode::STAND_DOWN, SportMode::LOCOMOTION));
  EXPECT_FALSE(sport_transition_allowed(SportMode::DAMP, SportMode::LOCOMOTION));
  EXPECT_TRUE(sport_transition_allowed(SportMode::BALANCE_STAND, SportMode::LOCOMOTION));
  EXPECT_TRUE(sport_transition_allowed(SportMode::LOCOMOTION, SportMode::LOCOMOTION));
}

TEST(Go2SportCommand, QuadrupedCanRecoveryStandFromAnyState)
{
  // The Go2 self-rights into balance-stand from lying or damped states.
  EXPECT_TRUE(sport_transition_allowed(SportMode::STAND_DOWN, SportMode::BALANCE_STAND));
  EXPECT_TRUE(sport_transition_allowed(SportMode::DAMP, SportMode::BALANCE_STAND));
  EXPECT_TRUE(sport_transition_allowed(SportMode::LOCOMOTION, SportMode::BALANCE_STAND));
}

TEST(Go2SportCommand, DampAlwaysReachableSitNeedsAStand)
{
  EXPECT_TRUE(sport_transition_allowed(SportMode::LOCOMOTION, SportMode::DAMP));
  EXPECT_TRUE(sport_transition_allowed(SportMode::STAND_DOWN, SportMode::DAMP));
  // Lying down cleanly requires standing first; not directly from a damp.
  EXPECT_TRUE(sport_transition_allowed(SportMode::BALANCE_STAND, SportMode::STAND_DOWN));
  EXPECT_FALSE(sport_transition_allowed(SportMode::DAMP, SportMode::STAND_DOWN));
}
