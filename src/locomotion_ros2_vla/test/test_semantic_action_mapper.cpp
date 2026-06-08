#include <gtest/gtest.h>

#include "locomotion_ros2_vla/semantic_action_mapper.hpp"

namespace
{

locomotion_ros2_msgs::msg::SemanticAction make_action(const std::string & name)
{
  locomotion_ros2_msgs::msg::SemanticAction action;
  action.action = name;
  return action;
}

}  // namespace

TEST(SemanticActionMapper, StopRequestsStop)
{
  locomotion_ros2_vla::SemanticActionMapper mapper;
  const auto mapping = mapper.map(make_action("stop"));

  EXPECT_TRUE(mapping.recognized);
  EXPECT_TRUE(mapping.stop);
}

TEST(SemanticActionMapper, MoveForwardIsConservativeAndPositive)
{
  locomotion_ros2_vla::SemanticActionMapper mapper;
  const auto mapping = mapper.map(make_action("move_forward"));

  EXPECT_TRUE(mapping.recognized);
  EXPECT_FALSE(mapping.stop);
  EXPECT_GT(mapping.velocity.twist.linear.x, 0.0);
}

TEST(SemanticActionMapper, RunForwardIsFasterThanWalkForward)
{
  locomotion_ros2_vla::SemanticActionMapper mapper;
  const auto walk = mapper.map(make_action("move_forward"));
  const auto run = mapper.map(make_action("run_forward"));

  EXPECT_TRUE(run.recognized);
  EXPECT_GT(run.velocity.twist.linear.x, walk.velocity.twist.linear.x);
}

TEST(SemanticActionMapper, SlowCarefulWalkIsSlowerThanWalkForward)
{
  locomotion_ros2_vla::SemanticActionMapper mapper;
  const auto walk = mapper.map(make_action("walk_forward"));
  const auto slow = mapper.map(make_action("slow_careful_walk"));

  EXPECT_TRUE(slow.recognized);
  EXPECT_GT(walk.velocity.twist.linear.x, slow.velocity.twist.linear.x);
}

TEST(SemanticActionMapper, SlowWalkAliasMatchesSlowCarefulWalk)
{
  locomotion_ros2_vla::SemanticActionMapper mapper;
  const auto slow = mapper.map(make_action("slow_careful_walk"));
  const auto alias = mapper.map(make_action("slow_walk"));

  EXPECT_TRUE(alias.recognized);
  EXPECT_DOUBLE_EQ(alias.velocity.twist.linear.x, slow.velocity.twist.linear.x);
}

TEST(SemanticActionMapper, WalkForwardAliasMatchesMoveForward)
{
  locomotion_ros2_vla::SemanticActionMapper mapper;
  const auto move = mapper.map(make_action("move_forward"));
  const auto walk = mapper.map(make_action("walk_forward"));

  EXPECT_TRUE(walk.recognized);
  EXPECT_DOUBLE_EQ(walk.velocity.twist.linear.x, move.velocity.twist.linear.x);
}

TEST(SemanticActionMapper, SidestepLeftAndRightHaveOppositeLateral)
{
  locomotion_ros2_vla::SemanticActionMapper mapper;
  const auto left = mapper.map(make_action("sidestep_left"));
  const auto right = mapper.map(make_action("sidestep_right"));

  EXPECT_TRUE(left.recognized);
  EXPECT_TRUE(right.recognized);
  EXPECT_GT(left.velocity.twist.linear.y, 0.0);
  EXPECT_LT(right.velocity.twist.linear.y, 0.0);
}

TEST(SemanticActionMapper, MoveBackwardIsConservativeAndNegative)
{
  locomotion_ros2_vla::SemanticActionMapper mapper;
  const auto mapping = mapper.map(make_action("move_backward"));

  EXPECT_TRUE(mapping.recognized);
  EXPECT_FALSE(mapping.stop);
  EXPECT_LT(mapping.velocity.twist.linear.x, 0.0);
}

TEST(SemanticActionMapper, WalkBackwardAliasMatchesMoveBackward)
{
  locomotion_ros2_vla::SemanticActionMapper mapper;
  const auto move = mapper.map(make_action("move_backward"));
  const auto walk = mapper.map(make_action("walk_backward"));

  EXPECT_TRUE(walk.recognized);
  EXPECT_DOUBLE_EQ(walk.velocity.twist.linear.x, move.velocity.twist.linear.x);
}

TEST(SemanticActionMapper, TurnLeftAndRightHaveOppositeYaw)
{
  locomotion_ros2_vla::SemanticActionMapper mapper;
  const auto left = mapper.map(make_action("turn_left"));
  const auto right = mapper.map(make_action("turn_right"));

  EXPECT_TRUE(left.recognized);
  EXPECT_TRUE(right.recognized);
  EXPECT_GT(left.velocity.twist.angular.z, 0.0);
  EXPECT_LT(right.velocity.twist.angular.z, 0.0);
}

TEST(SemanticActionMapper, UnknownActionIsNotRecognized)
{
  locomotion_ros2_vla::SemanticActionMapper mapper;
  const auto mapping = mapper.map(make_action("do_a_backflip"));

  EXPECT_FALSE(mapping.recognized);
  EXPECT_FALSE(mapping.stop);
}
