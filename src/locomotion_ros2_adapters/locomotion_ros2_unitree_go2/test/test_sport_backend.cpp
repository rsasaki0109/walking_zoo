#include <gtest/gtest.h>

#include "locomotion_ros2_unitree_go2/sport_backend.hpp"

using locomotion_ros2_unitree_go2::Go2PostureCommand;
using locomotion_ros2_unitree_go2::Go2VelocityCommand;
using locomotion_ros2_unitree_go2::make_sport_backend;
using locomotion_ros2_unitree_go2::SilSportBackend;
using locomotion_ros2_unitree_go2::SportMode;

TEST(SportBackend, DefaultFactoryIsSilWithoutSdk)
{
  auto backend = make_sport_backend();
  ASSERT_NE(backend, nullptr);
#ifdef LOCOMOTION_ROS2_WITH_UNITREE_SDK2
  EXPECT_TRUE(backend->dispatches_to_hardware());
  EXPECT_EQ(backend->name(), "unitree_sdk2");
#else
  EXPECT_FALSE(backend->dispatches_to_hardware());
  EXPECT_EQ(backend->name(), "sil");
#endif
}

TEST(SportBackend, SilConnectSucceeds)
{
  SilSportBackend backend;
  EXPECT_FALSE(backend.connected());
  EXPECT_TRUE(backend.connect("eth0"));
  EXPECT_TRUE(backend.connected());
}

TEST(SportBackend, SilRecordsVelocityAndMode)
{
  SilSportBackend backend;
  backend.set_mode(SportMode::LOCOMOTION);
  EXPECT_EQ(backend.last_mode(), SportMode::LOCOMOTION);

  Go2VelocityCommand cmd;
  cmd.vx = 0.4;
  cmd.vy = -0.1;
  cmd.vyaw = 0.3;
  backend.send_velocity(cmd);
  EXPECT_DOUBLE_EQ(backend.last_velocity().vx, 0.4);
  EXPECT_DOUBLE_EQ(backend.last_velocity().vy, -0.1);
  EXPECT_DOUBLE_EQ(backend.last_velocity().vyaw, 0.3);
}

TEST(SportBackend, SilRecordsPosture)
{
  SilSportBackend backend;
  Go2PostureCommand cmd;
  cmd.roll = 0.1;
  cmd.pitch = -0.05;
  cmd.yaw = 0.2;
  cmd.height = -0.06;
  backend.send_posture(cmd);
  EXPECT_DOUBLE_EQ(backend.last_posture().roll, 0.1);
  EXPECT_DOUBLE_EQ(backend.last_posture().yaw, 0.2);
  EXPECT_DOUBLE_EQ(backend.last_posture().height, -0.06);
}

TEST(SportBackend, SilEmergencyDampResetsVelocity)
{
  SilSportBackend backend;
  Go2VelocityCommand cmd;
  cmd.vx = 0.5;
  backend.send_velocity(cmd);
  backend.emergency_damp();
  EXPECT_EQ(backend.last_mode(), SportMode::DAMP);
  EXPECT_DOUBLE_EQ(backend.last_velocity().vx, 0.0);
}
