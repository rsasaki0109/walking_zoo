#include <gtest/gtest.h>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "walking_zoo_core/robot_profile.hpp"
#include "walking_zoo_gait_lab_sil/gait_lab_sil_model.hpp"
#include "walking_zoo_msgs/msg/walking_state.hpp"

using walking_zoo_gait_lab_sil::GaitLabSilModel;
using walking_zoo_core::StopMode;
using WalkingState = walking_zoo_msgs::msg::WalkingState;

namespace
{
GaitLabSilModel make_configured()
{
  GaitLabSilModel m;
  m.configure(walking_zoo_core::RobotProfile{});
  return m;
}

geometry_msgs::msg::TwistStamped forward(double vx)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.twist.linear.x = vx;
  return cmd;
}
}  // namespace

TEST(GaitLabSilModel, ConfigureSetsPluginNameAndInactive)
{
  auto m = make_configured();
  EXPECT_TRUE(m.configured());
  EXPECT_FALSE(m.active());
  EXPECT_EQ(m.profile().adapter_plugin, GaitLabSilModel::PLUGIN_NAME);
}

TEST(GaitLabSilModel, VelocityRejectedUntilActive)
{
  auto m = make_configured();
  EXPECT_FALSE(m.command_velocity_gate(forward(0.2)).accepted);
  ASSERT_EQ(m.activate(), walking_zoo_core::CallbackReturn::SUCCESS);
  EXPECT_TRUE(m.command_velocity_gate(forward(0.2)).accepted);
}

TEST(GaitLabSilModel, EstopBlocksThenClearFaultRecovers)
{
  auto m = make_configured();
  m.activate();
  EXPECT_TRUE(m.emergency_stop_gate().accepted);
  EXPECT_TRUE(m.estop_active());
  // While estopped, velocity is blocked (not merely rejected).
  auto blocked = m.command_velocity_gate(forward(0.2));
  EXPECT_FALSE(blocked.accepted);
  EXPECT_EQ(blocked.status, walking_zoo_core::CommandStatus::BLOCKED);
  EXPECT_TRUE(m.clear_fault_gate().accepted);
  EXPECT_FALSE(m.estop_active());
  EXPECT_TRUE(m.command_velocity_gate(forward(0.2)).accepted);
}

TEST(GaitLabSilModel, BodyPoseAndFootstepsUnsupported)
{
  auto m = make_configured();
  m.activate();
  EXPECT_FALSE(m.body_pose_gate().accepted);
  EXPECT_FALSE(m.footstep_gate().accepted);
}

TEST(GaitLabSilModel, ControlSignalForStopMode)
{
  auto m = make_configured();
  EXPECT_EQ(m.control_for_stop(StopMode::EMERGENCY), GaitLabSilModel::CTRL_ESTOP);
  EXPECT_EQ(m.control_for_stop(StopMode::QUICK), GaitLabSilModel::CTRL_STOP_QUICK);
  EXPECT_EQ(m.control_for_stop(StopMode::NORMAL), GaitLabSilModel::CTRL_STOP_NORMAL);
}

TEST(GaitLabSilModel, SimStateFreshnessWindow)
{
  auto m = make_configured();
  m.set_freshness_timeout(0.5);
  EXPECT_FALSE(m.sim_connected(10.0));  // nothing ingested yet

  WalkingState s;
  s.is_balanced = true;
  s.locomotion_state = WalkingState::STATE_WALKING;
  m.ingest_sim_state(s, 10.0);
  EXPECT_TRUE(m.sim_connected(10.3));    // within window
  EXPECT_FALSE(m.sim_connected(10.9));   // stale
}

TEST(GaitLabSilModel, ReadStateUsesFreshSimStateAndStampsLifecycle)
{
  auto m = make_configured();
  m.activate();
  WalkingState s;
  s.locomotion_state = WalkingState::STATE_WALKING;
  s.is_balanced = true;
  s.is_fallen = false;
  m.ingest_sim_state(s, 10.0);

  auto fresh = m.read_state(10.1);
  EXPECT_TRUE(fresh.adapter_connected);
  EXPECT_EQ(fresh.locomotion_state, WalkingState::STATE_WALKING);
  EXPECT_EQ(fresh.lifecycle_state, WalkingState::LIFECYCLE_ACTIVE);
  EXPECT_EQ(fresh.active_adapter, GaitLabSilModel::PLUGIN_NAME);

  // Once the sim goes stale, read_state stops reporting its last (walking) claim
  // and synthesizes a standing state instead.
  auto stale = m.read_state(11.0);
  EXPECT_FALSE(stale.adapter_connected);
  EXPECT_EQ(stale.locomotion_state, WalkingState::STATE_STANDING);
}

TEST(GaitLabSilModel, EstopOverridesReportedState)
{
  auto m = make_configured();
  m.activate();
  WalkingState s;
  s.locomotion_state = WalkingState::STATE_WALKING;
  m.ingest_sim_state(s, 10.0);
  m.emergency_stop_gate();
  auto state = m.read_state(10.1);
  EXPECT_TRUE(state.estop_active);
  EXPECT_EQ(state.lifecycle_state, WalkingState::LIFECYCLE_ESTOPPED);
  EXPECT_EQ(state.locomotion_state, WalkingState::STATE_ESTOPPED);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
