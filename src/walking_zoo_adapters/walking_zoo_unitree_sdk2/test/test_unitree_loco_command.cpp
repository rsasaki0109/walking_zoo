#include <gtest/gtest.h>

#include "walking_zoo_unitree_sdk2/unitree_loco_command.hpp"

using walking_zoo_unitree_sdk2::G1PostureLimits;
using walking_zoo_unitree_sdk2::G1VelocityLimits;
using walking_zoo_unitree_sdk2::LocoMode;
using walking_zoo_unitree_sdk2::loco_transition_allowed;
using walking_zoo_unitree_sdk2::translate_body_pose;
using walking_zoo_unitree_sdk2::translate_velocity;

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

TEST(UnitreeLocoCommand, VelocityPassesThroughWithinEnvelope)
{
  G1VelocityLimits limits;
  const auto out = translate_velocity(twist(0.3, 0.1, 0.2), limits);
  EXPECT_FALSE(out.clamped);
  EXPECT_DOUBLE_EQ(out.vx, 0.3);
  EXPECT_DOUBLE_EQ(out.vy, 0.1);
  EXPECT_DOUBLE_EQ(out.vyaw, 0.2);
}

TEST(UnitreeLocoCommand, VelocityClampsForwardAndYaw)
{
  G1VelocityLimits limits;
  const auto out = translate_velocity(twist(5.0, 0.0, 5.0), limits);
  EXPECT_TRUE(out.clamped);
  EXPECT_DOUBLE_EQ(out.vx, limits.max_forward);
  EXPECT_DOUBLE_EQ(out.vyaw, limits.max_yaw_rate);
}

TEST(UnitreeLocoCommand, VelocityUsesAsymmetricBackwardLimit)
{
  G1VelocityLimits limits;
  limits.max_forward = 0.6;
  limits.max_backward = 0.2;
  const auto out = translate_velocity(twist(-1.0, 0.0, 0.0), limits);
  EXPECT_TRUE(out.clamped);
  EXPECT_DOUBLE_EQ(out.vx, -0.2);
}

TEST(UnitreeLocoCommand, BodyPoseClampsToPostureEnvelope)
{
  G1PostureLimits limits;
  walking_zoo_msgs::msg::BodyPoseCommand cmd;
  cmd.roll = 1.0F;
  cmd.pitch = -1.0F;
  cmd.yaw = 0.0F;
  cmd.body_height = -0.5F;
  const auto out = translate_body_pose(cmd, limits);
  EXPECT_TRUE(out.clamped);
  EXPECT_DOUBLE_EQ(out.roll, limits.max_roll);
  EXPECT_DOUBLE_EQ(out.pitch, -limits.max_pitch);
  EXPECT_DOUBLE_EQ(out.height, limits.min_height);
}

TEST(UnitreeLocoCommand, LocomotionRequiresBalanceStand)
{
  EXPECT_FALSE(loco_transition_allowed(LocoMode::ZERO_TORQUE, LocoMode::LOCOMOTION));
  EXPECT_FALSE(loco_transition_allowed(LocoMode::DAMP, LocoMode::LOCOMOTION));
  EXPECT_TRUE(loco_transition_allowed(LocoMode::BALANCE_STAND, LocoMode::LOCOMOTION));
  EXPECT_TRUE(loco_transition_allowed(LocoMode::LOCOMOTION, LocoMode::LOCOMOTION));
}

TEST(UnitreeLocoCommand, SafetyModesAlwaysReachable)
{
  EXPECT_TRUE(loco_transition_allowed(LocoMode::LOCOMOTION, LocoMode::DAMP));
  EXPECT_TRUE(loco_transition_allowed(LocoMode::ZERO_TORQUE, LocoMode::DAMP));
  EXPECT_TRUE(loco_transition_allowed(LocoMode::LOCOMOTION, LocoMode::BALANCE_STAND));
}
