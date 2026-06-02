#include <gtest/gtest.h>

#include "walking_zoo_unitree_sdk2/loco_backend.hpp"

using walking_zoo_unitree_sdk2::LocoMode;
using walking_zoo_unitree_sdk2::LocoPostureCommand;
using walking_zoo_unitree_sdk2::LocoVelocityCommand;
using walking_zoo_unitree_sdk2::make_loco_backend;
using walking_zoo_unitree_sdk2::SilLocoBackend;

TEST(LocoBackend, DefaultFactoryIsSilWithoutSdk)
{
  auto backend = make_loco_backend();
  ASSERT_NE(backend, nullptr);
#ifdef WALKING_ZOO_WITH_UNITREE_SDK2
  EXPECT_TRUE(backend->dispatches_to_hardware());
  EXPECT_EQ(backend->name(), "unitree_sdk2");
#else
  EXPECT_FALSE(backend->dispatches_to_hardware());
  EXPECT_EQ(backend->name(), "sil");
#endif
}

TEST(LocoBackend, SilConnectSucceeds)
{
  SilLocoBackend backend;
  EXPECT_FALSE(backend.connected());
  EXPECT_TRUE(backend.connect("lo"));
  EXPECT_TRUE(backend.connected());
}

TEST(LocoBackend, SilRecordsVelocityAndMode)
{
  SilLocoBackend backend;
  backend.set_mode(LocoMode::LOCOMOTION);
  EXPECT_EQ(backend.last_mode(), LocoMode::LOCOMOTION);

  LocoVelocityCommand cmd;
  cmd.vx = 0.3;
  cmd.vy = -0.1;
  cmd.vyaw = 0.2;
  backend.send_velocity(cmd);
  EXPECT_DOUBLE_EQ(backend.last_velocity().vx, 0.3);
  EXPECT_DOUBLE_EQ(backend.last_velocity().vy, -0.1);
  EXPECT_DOUBLE_EQ(backend.last_velocity().vyaw, 0.2);
}

TEST(LocoBackend, SilRecordsPosture)
{
  SilLocoBackend backend;
  LocoPostureCommand cmd;
  cmd.roll = 0.1;
  cmd.pitch = -0.05;
  cmd.height = -0.08;
  backend.send_posture(cmd);
  EXPECT_DOUBLE_EQ(backend.last_posture().roll, 0.1);
  EXPECT_DOUBLE_EQ(backend.last_posture().height, -0.08);
}

TEST(LocoBackend, SilEmergencyDampResetsVelocity)
{
  SilLocoBackend backend;
  LocoVelocityCommand cmd;
  cmd.vx = 0.5;
  backend.send_velocity(cmd);
  backend.emergency_damp();
  EXPECT_EQ(backend.last_mode(), LocoMode::DAMP);
  EXPECT_DOUBLE_EQ(backend.last_velocity().vx, 0.0);
}
