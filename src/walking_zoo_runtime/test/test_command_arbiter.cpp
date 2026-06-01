#include <gtest/gtest.h>

#include "walking_zoo_runtime/command_arbiter.hpp"

TEST(CommandArbiter, SafetyOutranksVla)
{
  walking_zoo_runtime::CommandArbiter arbiter;

  EXPECT_GT(arbiter.priority_for_source("estop"), arbiter.priority_for_source("vla"));
  EXPECT_GT(arbiter.priority_for_source("teleop"), arbiter.priority_for_source("nav2"));
  EXPECT_TRUE(arbiter.should_replace("vla", "nav2"));
  EXPECT_FALSE(arbiter.should_replace("estop", "nav2"));
}
