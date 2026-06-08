#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "locomotion_ros2_core/robot_profile.hpp"

TEST(RobotProfile, ConvertsToMessage)
{
  locomotion_ros2_core::RobotProfile profile;
  profile.robot_model = "unitree_go2";
  profile.max_linear_x = 0.5;

  const auto msg = profile.to_msg();

  EXPECT_EQ(msg.robot_model, "unitree_go2");
  EXPECT_FLOAT_EQ(msg.max_linear_x, 0.5F);
  EXPECT_FALSE(msg.real_robot_motion_allowed);
}

TEST(RobotProfile, LoadsYamlProfile)
{
  const auto path = std::filesystem::temp_directory_path() / "locomotion_ros2_robot_profile_test.yaml";
  std::ofstream file(path);
  file << R"(
robot_model: unitree_g1
robot_family: humanoid
adapter_plugin: locomotion_ros2_unitree_sdk2/UnitreeSdk2Adapter
capabilities:
  velocity_command: true
  body_pose_command: true
  footstep_plan: planned
  whole_body_goal: planned
  sit_stand: true
  estop: true
  lateral_step: true
  turn_in_place: true
limits:
  max_linear_x: 0.4
  max_linear_y: 0.2
  max_angular_z: 0.6
  command_timeout_sec: 0.25
frames:
  base_frame: g1_base
  odom_frame: odom
  map_frame: map
safety:
  allow_motion_default: false
)";
  file.close();

  const auto profile = locomotion_ros2_core::load_robot_profile_from_yaml(path.string());

  EXPECT_EQ(profile.robot_model, "unitree_g1");
  EXPECT_EQ(profile.robot_family, "humanoid");
  EXPECT_EQ(profile.adapter_plugin, "locomotion_ros2_unitree_sdk2/UnitreeSdk2Adapter");
  EXPECT_TRUE(profile.velocity_command);
  EXPECT_FALSE(profile.footstep_plan);
  EXPECT_DOUBLE_EQ(profile.max_linear_x, 0.4);
  EXPECT_EQ(profile.base_frame, "g1_base");
  EXPECT_FALSE(profile.real_robot_motion_allowed);

  std::filesystem::remove(path);
}
