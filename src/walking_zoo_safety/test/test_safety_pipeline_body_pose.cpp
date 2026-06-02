#include <gtest/gtest.h>

#include "walking_zoo_msgs/msg/body_pose_command.hpp"
#include "walking_zoo_safety/safety_pipeline.hpp"

using walking_zoo_safety::FallState;
using walking_zoo_safety::SafetyPipeline;
using walking_zoo_core::CommandStatus;
using BodyPoseCommand = walking_zoo_msgs::msg::BodyPoseCommand;

namespace
{

BodyPoseCommand make_pose(float roll, float pitch, float duration = 1.0F)
{
  BodyPoseCommand cmd;
  cmd.roll = roll;
  cmd.pitch = pitch;
  cmd.duration_sec = duration;
  cmd.source = "test";
  return cmd;
}

SafetyPipeline make_pipeline()
{
  SafetyPipeline pipeline;
  pipeline.set_body_pose_limits(0.2, 0.2);
  pipeline.set_fall_thresholds(0.35, 0.70);
  return pipeline;
}

}  // namespace

TEST(SafetyPipelineBodyPose, AcceptsPoseWithinLimits)
{
  auto pipeline = make_pipeline();
  const auto out = pipeline.filter_body_pose(make_pose(0.1F, 0.1F));
  EXPECT_TRUE(out.result.accepted);
  EXPECT_EQ(out.result.status, CommandStatus::ACCEPTED);
  EXPECT_FLOAT_EQ(out.command.roll, 0.1F);
  EXPECT_FLOAT_EQ(out.command.pitch, 0.1F);
}

TEST(SafetyPipelineBodyPose, ClampsBeyondPerAxisLimit)
{
  auto pipeline = make_pipeline();
  // 0.3 rad roll is past the 0.2 limit but well short of the fall band.
  const auto out = pipeline.filter_body_pose(make_pose(0.3F, 0.0F));
  EXPECT_TRUE(out.result.accepted);
  EXPECT_EQ(out.result.status, CommandStatus::LIMITED);
  EXPECT_FLOAT_EQ(out.command.roll, 0.2F);
}

TEST(SafetyPipelineBodyPose, ClampsPreservesSign)
{
  auto pipeline = make_pipeline();
  const auto out = pipeline.filter_body_pose(make_pose(0.0F, -0.5F));
  EXPECT_TRUE(out.result.accepted);
  EXPECT_EQ(out.result.status, CommandStatus::LIMITED);
  EXPECT_FLOAT_EQ(out.command.pitch, -0.2F);
}

TEST(SafetyPipelineBodyPose, RejectsGrossOverTiltAsFall)
{
  auto pipeline = make_pipeline();
  // Combined tilt sqrt(0.6^2 + 0.6^2) ~= 0.85 rad >= 0.70 fall threshold.
  const auto out = pipeline.filter_body_pose(make_pose(0.6F, 0.6F));
  EXPECT_FALSE(out.result.accepted);
  EXPECT_EQ(out.result.status, CommandStatus::REJECTED);
}

TEST(SafetyPipelineBodyPose, BlocksWhenEstopActive)
{
  auto pipeline = make_pipeline();
  pipeline.set_estop_active(true);
  const auto out = pipeline.filter_body_pose(make_pose(0.05F, 0.05F));
  EXPECT_FALSE(out.result.accepted);
  EXPECT_EQ(out.result.status, CommandStatus::BLOCKED);
}

TEST(SafetyPipelineBodyPose, ClassifyTiltMatchesThresholds)
{
  auto pipeline = make_pipeline();
  EXPECT_EQ(pipeline.classify_tilt(0.0, 0.0), FallState::UPRIGHT);
  EXPECT_EQ(pipeline.classify_tilt(0.0, 0.5), FallState::TILTED);
  EXPECT_EQ(pipeline.classify_tilt(0.6, 0.6), FallState::FALLEN);
}
