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

TEST(CommandArbiter, PriorityOrderingIsStrict)
{
  walking_zoo_runtime::CommandArbiter arbiter;

  // estop > safety > operator/teleop > nav2 > vla > background
  EXPECT_GT(arbiter.priority_for_source("estop"), arbiter.priority_for_source("safety"));
  EXPECT_GT(arbiter.priority_for_source("safety"), arbiter.priority_for_source("teleop"));
  EXPECT_GT(arbiter.priority_for_source("teleop"), arbiter.priority_for_source("nav2"));
  EXPECT_GT(arbiter.priority_for_source("nav2"), arbiter.priority_for_source("vla"));
  EXPECT_GT(arbiter.priority_for_source("vla"), arbiter.priority_for_source("background"));
}

TEST(CommandArbiter, AliasesShareSamePriority)
{
  walking_zoo_runtime::CommandArbiter arbiter;

  EXPECT_EQ(arbiter.priority_for_source("estop"), arbiter.priority_for_source("emergency_stop"));
  EXPECT_EQ(arbiter.priority_for_source("safety"), arbiter.priority_for_source("fall_recovery"));
  EXPECT_EQ(arbiter.priority_for_source("operator"), arbiter.priority_for_source("teleop"));
  EXPECT_EQ(arbiter.priority_for_source("operator"), arbiter.priority_for_source("manual"));
  EXPECT_EQ(arbiter.priority_for_source("nav2"), arbiter.priority_for_source("/cmd_vel"));
  EXPECT_EQ(arbiter.priority_for_source("vla"), arbiter.priority_for_source("semantic_action"));
}

TEST(CommandArbiter, VlaNeverOutranksOperatorOrSafety)
{
  walking_zoo_runtime::CommandArbiter arbiter;

  // The core safety invariant: semantic intent cannot override an operator,
  // the safety supervisor, or the e-stop.
  EXPECT_FALSE(arbiter.should_replace("operator", "vla"));
  EXPECT_FALSE(arbiter.should_replace("safety", "vla"));
  EXPECT_FALSE(arbiter.should_replace("estop", "vla"));
}

TEST(CommandArbiter, HigherPriorityReplacesLower)
{
  walking_zoo_runtime::CommandArbiter arbiter;

  EXPECT_TRUE(arbiter.should_replace("vla", "teleop"));
  EXPECT_TRUE(arbiter.should_replace("nav2", "safety"));
  EXPECT_TRUE(arbiter.should_replace("teleop", "estop"));
}

TEST(CommandArbiter, LowerPriorityDoesNotReplaceHigher)
{
  walking_zoo_runtime::CommandArbiter arbiter;

  EXPECT_FALSE(arbiter.should_replace("teleop", "vla"));
  EXPECT_FALSE(arbiter.should_replace("safety", "nav2"));
  EXPECT_FALSE(arbiter.should_replace("estop", "teleop"));
}

TEST(CommandArbiter, EqualPriorityReplacesSoLatestWins)
{
  walking_zoo_runtime::CommandArbiter arbiter;

  // Two commands from the same priority tier: the newest one takes over.
  EXPECT_TRUE(arbiter.should_replace("nav2", "/cmd_vel"));
  EXPECT_TRUE(arbiter.should_replace("vla", "semantic_action"));
}

TEST(CommandArbiter, UnknownSourceIsBackgroundAndYields)
{
  walking_zoo_runtime::CommandArbiter arbiter;

  EXPECT_EQ(
    arbiter.priority_for_source("some_unknown_source"),
    arbiter.priority_for_source("background"));
  EXPECT_TRUE(arbiter.should_replace("some_unknown_source", "vla"));
}
