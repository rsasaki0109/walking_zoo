#include <gtest/gtest.h>

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "walking_zoo_core/adapter_context.hpp"
#include "walking_zoo_unitree_sdk2/unitree_sdk2_adapter.hpp"

using walking_zoo_unitree_sdk2::UnitreeSdk2Adapter;
using walking_zoo_core::CommandStatus;
using WalkingState = walking_zoo_msgs::msg::WalkingState;

namespace
{

walking_zoo_core::AdapterContext make_context()
{
  walking_zoo_core::AdapterContext context(
    rclcpp::get_logger("test_unitree_adapter"), std::make_shared<rclcpp::Clock>());
  context.robot_profile.robot_model = "g1";
  context.robot_profile.max_linear_x = 0.6;
  context.robot_profile.max_linear_y = 0.4;
  context.robot_profile.max_angular_z = 0.8;
  context.allow_motion = false;
  return context;
}

geometry_msgs::msg::TwistStamped twist(double x, double y, double wz)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.twist.linear.x = x;
  cmd.twist.linear.y = y;
  cmd.twist.angular.z = wz;
  return cmd;
}

UnitreeSdk2Adapter make_active_adapter()
{
  UnitreeSdk2Adapter adapter;
  adapter.configure(make_context());
  adapter.activate();
  return adapter;
}

}  // namespace

TEST(UnitreeSdk2Adapter, RejectsVelocityWhenInactive)
{
  UnitreeSdk2Adapter adapter;
  adapter.configure(make_context());
  const auto result = adapter.command_velocity(twist(0.2, 0.0, 0.0));
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, CommandStatus::REJECTED);
}

TEST(UnitreeSdk2Adapter, StandsUpOnActivate)
{
  auto adapter = make_active_adapter();
  const auto state = adapter.read_state();
  EXPECT_EQ(state.locomotion_state, WalkingState::STATE_STANDING);
  EXPECT_EQ(state.locomotion_mode, WalkingState::MODE_STAND);
  EXPECT_TRUE(state.is_balanced);
}

TEST(UnitreeSdk2Adapter, VelocityEntersLocomotionAndReportsWalking)
{
  auto adapter = make_active_adapter();
  const auto result = adapter.command_velocity(twist(0.3, 0.0, 0.0));
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.status, CommandStatus::ACCEPTED);
  const auto state = adapter.read_state();
  EXPECT_EQ(state.locomotion_state, WalkingState::STATE_WALKING);
  EXPECT_EQ(state.locomotion_mode, WalkingState::MODE_WALK);
}

TEST(UnitreeSdk2Adapter, VelocityBeyondEnvelopeIsLimited)
{
  auto adapter = make_active_adapter();
  const auto result = adapter.command_velocity(twist(5.0, 0.0, 0.0));
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.status, CommandStatus::LIMITED);
}

TEST(UnitreeSdk2Adapter, FootstepPlanIsUnsupported)
{
  auto adapter = make_active_adapter();
  walking_zoo_msgs::msg::FootstepPlan plan;
  const auto result = adapter.execute_footstep_plan(plan);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, CommandStatus::REJECTED);
}

TEST(UnitreeSdk2Adapter, EmergencyStopDampsAndBlocksCommands)
{
  auto adapter = make_active_adapter();
  adapter.command_velocity(twist(0.3, 0.0, 0.0));
  adapter.emergency_stop();

  auto state = adapter.read_state();
  EXPECT_TRUE(state.estop_active);
  EXPECT_EQ(state.locomotion_state, WalkingState::STATE_ESTOPPED);

  const auto blocked = adapter.command_velocity(twist(0.3, 0.0, 0.0));
  EXPECT_FALSE(blocked.accepted);
  EXPECT_EQ(blocked.status, CommandStatus::BLOCKED);

  // Fault cannot clear while estop is latched.
  const auto fault = adapter.clear_fault();
  EXPECT_EQ(fault.status, CommandStatus::BLOCKED);
}

TEST(UnitreeSdk2Adapter, BodyPoseReturnsToBalanceStand)
{
  auto adapter = make_active_adapter();
  adapter.command_velocity(twist(0.3, 0.0, 0.0));  // enter locomotion first
  walking_zoo_msgs::msg::BodyPoseCommand cmd;
  cmd.roll = 0.1F;
  cmd.pitch = 0.1F;
  const auto result = adapter.command_body_pose(cmd);
  EXPECT_TRUE(result.accepted);
  const auto state = adapter.read_state();
  EXPECT_EQ(state.locomotion_state, WalkingState::STATE_STANDING);
  EXPECT_EQ(state.locomotion_mode, WalkingState::MODE_STAND);
}
