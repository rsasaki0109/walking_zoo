#include <gtest/gtest.h>

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "walking_zoo_core/adapter_context.hpp"
#include "walking_zoo_unitree_go2/sport_backend.hpp"
#include "walking_zoo_unitree_go2/unitree_go2_adapter.hpp"

using walking_zoo_core::CommandStatus;
using walking_zoo_unitree_go2::SilSportBackend;
using walking_zoo_unitree_go2::UnitreeGo2Adapter;
using WalkingState = walking_zoo_msgs::msg::WalkingState;

namespace
{

walking_zoo_core::AdapterContext make_context()
{
  walking_zoo_core::AdapterContext context(
    rclcpp::get_logger("test_unitree_go2_adapter"), std::make_shared<rclcpp::Clock>());
  context.robot_profile.robot_model = "go2";
  context.robot_profile.robot_family = "quadruped";
  context.robot_profile.max_linear_x = 0.5;
  context.robot_profile.max_linear_y = 0.3;
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

UnitreeGo2Adapter make_active_adapter()
{
  UnitreeGo2Adapter adapter;
  adapter.configure(make_context());
  adapter.activate();
  return adapter;
}

}  // namespace

TEST(UnitreeGo2Adapter, RejectsVelocityWhenInactive)
{
  UnitreeGo2Adapter adapter;
  adapter.configure(make_context());
  const auto result = adapter.command_velocity(twist(0.2, 0.0, 0.0));
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, CommandStatus::REJECTED);
}

TEST(UnitreeGo2Adapter, RestsLyingDownBeforeActivate)
{
  UnitreeGo2Adapter adapter;
  adapter.configure(make_context());
  const auto state = adapter.read_state();
  // A powered-on quadruped rests on the ground: a quadruped-specific sit state.
  EXPECT_EQ(state.locomotion_state, WalkingState::STATE_SITTING);
  EXPECT_FALSE(state.is_balanced);
}

TEST(UnitreeGo2Adapter, StandsUpOnActivate)
{
  auto adapter = make_active_adapter();
  const auto state = adapter.read_state();
  EXPECT_EQ(state.locomotion_state, WalkingState::STATE_STANDING);
  EXPECT_EQ(state.locomotion_mode, WalkingState::MODE_STAND);
  EXPECT_EQ(state.support_phase, WalkingState::SUPPORT_QUADRUPED);
  EXPECT_TRUE(state.is_balanced);
}

TEST(UnitreeGo2Adapter, SitsBackDownOnDeactivate)
{
  auto adapter = make_active_adapter();
  adapter.deactivate();
  const auto state = adapter.read_state();
  EXPECT_EQ(state.locomotion_state, WalkingState::STATE_SITTING);
  EXPECT_FALSE(state.is_balanced);
}

TEST(UnitreeGo2Adapter, VelocityEntersLocomotionAndReportsWalking)
{
  auto adapter = make_active_adapter();
  const auto result = adapter.command_velocity(twist(0.3, 0.0, 0.0));
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.status, CommandStatus::ACCEPTED);
  const auto state = adapter.read_state();
  EXPECT_EQ(state.locomotion_state, WalkingState::STATE_WALKING);
  EXPECT_EQ(state.locomotion_mode, WalkingState::MODE_WALK);
  EXPECT_EQ(state.support_phase, WalkingState::SUPPORT_QUADRUPED);
}

TEST(UnitreeGo2Adapter, DefaultBuildUsesSilBackend)
{
  auto adapter = make_active_adapter();
  ASSERT_NE(adapter.backend(), nullptr);
#ifdef WALKING_ZOO_WITH_UNITREE_SDK2
  EXPECT_TRUE(adapter.backend()->dispatches_to_hardware());
#else
  EXPECT_FALSE(adapter.backend()->dispatches_to_hardware());
#endif
}

TEST(UnitreeGo2Adapter, ForwardsTranslatedVelocityToBackend)
{
  // The SIL backend records what *would* be sent to hardware; verify the
  // adapter actually forwards the clamped command through the backend boundary.
  auto adapter = make_active_adapter();
  adapter.command_velocity(twist(5.0, 0.0, 0.0));  // beyond envelope -> clamped

  const auto * sil = dynamic_cast<const SilSportBackend *>(adapter.backend());
  ASSERT_NE(sil, nullptr);
  EXPECT_GT(sil->last_velocity().vx, 0.0);
  EXPECT_LE(sil->last_velocity().vx, 0.5 + 1e-9);  // clamped to forward envelope
}

TEST(UnitreeGo2Adapter, VelocityBeyondEnvelopeIsLimited)
{
  auto adapter = make_active_adapter();
  const auto result = adapter.command_velocity(twist(5.0, 0.0, 0.0));
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.status, CommandStatus::LIMITED);
}

TEST(UnitreeGo2Adapter, FootstepPlanIsUnsupported)
{
  auto adapter = make_active_adapter();
  walking_zoo_msgs::msg::FootstepPlan plan;
  const auto result = adapter.execute_footstep_plan(plan);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, CommandStatus::REJECTED);
}

TEST(UnitreeGo2Adapter, QuickStopSitsTheQuadrupedDown)
{
  auto adapter = make_active_adapter();
  adapter.command_velocity(twist(0.3, 0.0, 0.0));  // trotting
  const auto result = adapter.stop(walking_zoo_core::StopMode::QUICK);
  EXPECT_TRUE(result.accepted);
  const auto state = adapter.read_state();
  EXPECT_EQ(state.locomotion_state, WalkingState::STATE_SITTING);
}

TEST(UnitreeGo2Adapter, EmergencyStopDampsAndBlocksCommands)
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

TEST(UnitreeGo2Adapter, BodyPoseReturnsToBalanceStand)
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
