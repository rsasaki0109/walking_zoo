#include <gtest/gtest.h>

#include "locomotion_ros2_runtime/footstep_planner.hpp"

TEST(FootstepPlanner, StepCountMatchesRequest)
{
  locomotion_ros2_runtime::FootstepPlanner planner;
  locomotion_ros2_runtime::FootstepPlanParams params;
  params.step_count = 8;

  const auto plan = planner.plan(params);
  EXPECT_EQ(plan.footsteps.size(), 8u);
}

TEST(FootstepPlanner, ZeroStepsGivesEmptyPlan)
{
  locomotion_ros2_runtime::FootstepPlanner planner;
  locomotion_ros2_runtime::FootstepPlanParams params;
  params.step_count = 0;

  const auto plan = planner.plan(params);
  EXPECT_TRUE(plan.footsteps.empty());
}

TEST(FootstepPlanner, LegsAlternateStartingWithStartLeg)
{
  locomotion_ros2_runtime::FootstepPlanner planner;
  locomotion_ros2_runtime::FootstepPlanParams params;
  params.step_count = 4;
  params.start_leg = "left";

  const auto plan = planner.plan(params);
  ASSERT_EQ(plan.footsteps.size(), 4u);
  EXPECT_EQ(plan.footsteps[0].leg_name, "left");
  EXPECT_EQ(plan.footsteps[1].leg_name, "right");
  EXPECT_EQ(plan.footsteps[2].leg_name, "left");
  EXPECT_EQ(plan.footsteps[3].leg_name, "right");
}

TEST(FootstepPlanner, RespectsStartLegRight)
{
  locomotion_ros2_runtime::FootstepPlanner planner;
  locomotion_ros2_runtime::FootstepPlanParams params;
  params.step_count = 2;
  params.start_leg = "right";

  const auto plan = planner.plan(params);
  ASSERT_EQ(plan.footsteps.size(), 2u);
  EXPECT_EQ(plan.footsteps[0].leg_name, "right");
  EXPECT_EQ(plan.footsteps[1].leg_name, "left");
}

TEST(FootstepPlanner, ForwardProgressIsMonotonic)
{
  locomotion_ros2_runtime::FootstepPlanner planner;
  locomotion_ros2_runtime::FootstepPlanParams params;
  params.step_count = 6;

  const auto plan = planner.plan(params);
  ASSERT_EQ(plan.footsteps.size(), 6u);
  for (std::size_t i = 1; i < plan.footsteps.size(); ++i) {
    EXPECT_GT(plan.footsteps[i].pose.position.x, plan.footsteps[i - 1].pose.position.x);
  }
}

TEST(FootstepPlanner, LeftAndRightAreLaterallyOffset)
{
  locomotion_ros2_runtime::FootstepPlanner planner;
  locomotion_ros2_runtime::FootstepPlanParams params;
  params.step_count = 4;
  params.lateral_shift = 0.0;

  const auto plan = planner.plan(params);
  ASSERT_EQ(plan.footsteps.size(), 4u);
  for (const auto & footstep : plan.footsteps) {
    if (footstep.leg_name == "left") {
      EXPECT_GT(footstep.pose.position.y, 0.0);
    } else {
      EXPECT_LT(footstep.pose.position.y, 0.0);
    }
  }
}

TEST(FootstepPlanner, LateralShiftDriftsSideways)
{
  locomotion_ros2_runtime::FootstepPlanner planner;
  locomotion_ros2_runtime::FootstepPlanParams straight_params;
  straight_params.step_count = 4;
  straight_params.lateral_shift = 0.0;

  locomotion_ros2_runtime::FootstepPlanParams shifted_params = straight_params;
  shifted_params.lateral_shift = 0.05;

  const auto straight = planner.plan(straight_params);
  const auto shifted = planner.plan(shifted_params);
  ASSERT_EQ(straight.footsteps.size(), shifted.footsteps.size());

  // The last footstep should be shifted further along +y when a lateral drift
  // is requested.
  const auto & last_straight = straight.footsteps.back();
  const auto & last_shifted = shifted.footsteps.back();
  EXPECT_GT(last_shifted.pose.position.y, last_straight.pose.position.y);
}

TEST(FootstepPlanner, MetadataIsPopulated)
{
  locomotion_ros2_runtime::FootstepPlanner planner;
  locomotion_ros2_runtime::FootstepPlanParams params;
  params.step_count = 3;
  params.step_duration = 0.5;
  params.frame_id = "odom";

  const auto plan = planner.plan(params);
  EXPECT_EQ(plan.frame_id, "odom");
  EXPECT_EQ(plan.header.frame_id, "odom");
  EXPECT_FALSE(plan.planner_id.empty());
  EXPECT_FLOAT_EQ(plan.nominal_duration, 1.5f);
  for (const auto & footstep : plan.footsteps) {
    EXPECT_FLOAT_EQ(footstep.duration, 0.5f);
    EXPECT_GT(footstep.swing_height, 0.0f);
    EXPECT_DOUBLE_EQ(footstep.pose.orientation.w, 1.0);
  }
}
