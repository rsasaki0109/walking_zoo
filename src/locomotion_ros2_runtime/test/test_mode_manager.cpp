#include <gtest/gtest.h>

#include "locomotion_ros2_runtime/mode_manager.hpp"

using locomotion_ros2_msgs::msg::WalkingState;

TEST(ModeManager, DefaultsToIdle)
{
  locomotion_ros2_runtime::ModeManager manager;
  EXPECT_EQ(manager.mode(), WalkingState::MODE_IDLE);
}

TEST(ModeManager, AcceptsEveryValidMode)
{
  locomotion_ros2_runtime::ModeManager manager;

  for (const std::uint8_t mode : {
      WalkingState::MODE_IDLE,
      WalkingState::MODE_STAND,
      WalkingState::MODE_WALK,
      WalkingState::MODE_BODY_POSE,
      WalkingState::MODE_FOOTSTEP,
      WalkingState::MODE_SEMANTIC,
    })
  {
    EXPECT_TRUE(manager.set_mode(mode));
    EXPECT_EQ(manager.mode(), mode);
  }
}

TEST(ModeManager, RejectsUnknownMode)
{
  locomotion_ros2_runtime::ModeManager manager;
  manager.set_mode(WalkingState::MODE_WALK);

  EXPECT_FALSE(manager.set_mode(WalkingState::MODE_UNKNOWN));
  EXPECT_EQ(manager.mode(), WalkingState::MODE_WALK);
}

TEST(ModeManager, RejectsOutOfRangeMode)
{
  locomotion_ros2_runtime::ModeManager manager;
  manager.set_mode(WalkingState::MODE_STAND);

  EXPECT_FALSE(manager.set_mode(99));
  EXPECT_EQ(manager.mode(), WalkingState::MODE_STAND);
}

TEST(ModeManager, RejectedTransitionKeepsPreviousMode)
{
  locomotion_ros2_runtime::ModeManager manager;

  EXPECT_TRUE(manager.set_mode(WalkingState::MODE_SEMANTIC));
  EXPECT_FALSE(manager.set_mode(200));
  EXPECT_EQ(manager.mode(), WalkingState::MODE_SEMANTIC);
}
