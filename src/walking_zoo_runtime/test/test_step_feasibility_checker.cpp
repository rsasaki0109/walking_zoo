#include <gtest/gtest.h>

#include "walking_zoo_runtime/footstep_planner.hpp"
#include "walking_zoo_runtime/step_feasibility_checker.hpp"

namespace
{

walking_zoo_msgs::msg::Footstep make_footstep(double x, double y, float swing = 0.05f)
{
  walking_zoo_msgs::msg::Footstep footstep;
  footstep.pose.position.x = x;
  footstep.pose.position.y = y;
  footstep.swing_height = swing;
  return footstep;
}

}  // namespace

TEST(StepFeasibilityChecker, NominalPlanIsFeasible)
{
  walking_zoo_runtime::FootstepPlanner planner;
  walking_zoo_runtime::FootstepPlanParams params;
  params.step_count = 6;

  walking_zoo_runtime::StepFeasibilityChecker checker;
  const auto feasibility = checker.evaluate(planner.plan(params), {});

  EXPECT_TRUE(feasibility.feasible);
  EXPECT_EQ(feasibility.steps.size(), 6u);
  for (const auto & step : feasibility.steps) {
    EXPECT_TRUE(step.feasible);
    EXPECT_TRUE(step.reason.empty());
  }
}

TEST(StepFeasibilityChecker, ResultHasOneEntryPerFootstep)
{
  walking_zoo_msgs::msg::FootstepPlan plan;
  plan.footsteps.push_back(make_footstep(0.1, 0.16));
  plan.footsteps.push_back(make_footstep(0.2, -0.16));

  walking_zoo_runtime::StepFeasibilityChecker checker;
  const auto feasibility = checker.evaluate(plan, {});
  EXPECT_EQ(feasibility.steps.size(), 2u);
}

TEST(StepFeasibilityChecker, OversizedStrideIsInfeasible)
{
  walking_zoo_msgs::msg::FootstepPlan plan;
  plan.footsteps.push_back(make_footstep(0.0, 0.16));
  plan.footsteps.push_back(make_footstep(2.0, -0.16));  // huge forward jump

  walking_zoo_runtime::StepFeasibilityChecker checker;
  const auto feasibility = checker.evaluate(plan, {});

  EXPECT_FALSE(feasibility.feasible);
  EXPECT_TRUE(feasibility.steps[0].feasible);
  EXPECT_FALSE(feasibility.steps[1].feasible);
  EXPECT_FALSE(feasibility.steps[1].reason.empty());
}

TEST(StepFeasibilityChecker, ExcessiveLateralIsInfeasible)
{
  walking_zoo_msgs::msg::FootstepPlan plan;
  plan.footsteps.push_back(make_footstep(0.05, 0.90));  // way off the centre line

  walking_zoo_runtime::StepFeasibilityChecker checker;
  const auto feasibility = checker.evaluate(plan, {});

  EXPECT_FALSE(feasibility.feasible);
  EXPECT_FALSE(feasibility.steps[0].feasible);
}

TEST(StepFeasibilityChecker, ExcessiveSwingHeightIsInfeasible)
{
  walking_zoo_msgs::msg::FootstepPlan plan;
  plan.footsteps.push_back(make_footstep(0.05, 0.16, 0.50f));  // 0.5 m swing apex

  walking_zoo_runtime::StepFeasibilityChecker checker;
  const auto feasibility = checker.evaluate(plan, {});

  EXPECT_FALSE(feasibility.feasible);
  EXPECT_FALSE(feasibility.steps[0].feasible);
}

TEST(StepFeasibilityChecker, EmptyPlanIsFeasible)
{
  walking_zoo_msgs::msg::FootstepPlan plan;
  walking_zoo_runtime::StepFeasibilityChecker checker;
  const auto feasibility = checker.evaluate(plan, {});

  EXPECT_TRUE(feasibility.feasible);
  EXPECT_TRUE(feasibility.steps.empty());
}
