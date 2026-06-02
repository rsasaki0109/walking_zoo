#include <gtest/gtest.h>

#include "walking_zoo_vla/semantic_action_mapper.hpp"

namespace
{

walking_zoo_msgs::msg::SemanticAction make_action(const std::string & name)
{
  walking_zoo_msgs::msg::SemanticAction action;
  action.action = name;
  return action;
}

}  // namespace

TEST(SemanticActionMapper, StopRequestsStop)
{
  walking_zoo_vla::SemanticActionMapper mapper;
  const auto mapping = mapper.map(make_action("stop"));

  EXPECT_TRUE(mapping.recognized);
  EXPECT_TRUE(mapping.stop);
}

TEST(SemanticActionMapper, MoveForwardIsConservativeAndPositive)
{
  walking_zoo_vla::SemanticActionMapper mapper;
  const auto mapping = mapper.map(make_action("move_forward"));

  EXPECT_TRUE(mapping.recognized);
  EXPECT_FALSE(mapping.stop);
  EXPECT_GT(mapping.velocity.twist.linear.x, 0.0);
}

TEST(SemanticActionMapper, MoveBackwardIsConservativeAndNegative)
{
  walking_zoo_vla::SemanticActionMapper mapper;
  const auto mapping = mapper.map(make_action("move_backward"));

  EXPECT_TRUE(mapping.recognized);
  EXPECT_FALSE(mapping.stop);
  EXPECT_LT(mapping.velocity.twist.linear.x, 0.0);
}

TEST(SemanticActionMapper, WalkBackwardAliasMatchesMoveBackward)
{
  walking_zoo_vla::SemanticActionMapper mapper;
  const auto move = mapper.map(make_action("move_backward"));
  const auto walk = mapper.map(make_action("walk_backward"));

  EXPECT_TRUE(walk.recognized);
  EXPECT_DOUBLE_EQ(walk.velocity.twist.linear.x, move.velocity.twist.linear.x);
}

TEST(SemanticActionMapper, TurnLeftAndRightHaveOppositeYaw)
{
  walking_zoo_vla::SemanticActionMapper mapper;
  const auto left = mapper.map(make_action("turn_left"));
  const auto right = mapper.map(make_action("turn_right"));

  EXPECT_TRUE(left.recognized);
  EXPECT_TRUE(right.recognized);
  EXPECT_GT(left.velocity.twist.angular.z, 0.0);
  EXPECT_LT(right.velocity.twist.angular.z, 0.0);
}

TEST(SemanticActionMapper, UnknownActionIsNotRecognized)
{
  walking_zoo_vla::SemanticActionMapper mapper;
  const auto mapping = mapper.map(make_action("do_a_backflip"));

  EXPECT_FALSE(mapping.recognized);
  EXPECT_FALSE(mapping.stop);
}
