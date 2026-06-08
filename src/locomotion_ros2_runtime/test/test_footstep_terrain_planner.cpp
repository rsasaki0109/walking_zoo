#include <gtest/gtest.h>

#include <cmath>

#include "locomotion_ros2_runtime/footstep_planner.hpp"
#include "locomotion_ros2_runtime/terrain_model.hpp"

using locomotion_ros2_runtime::FootstepPlanner;
using locomotion_ros2_runtime::FootstepPlanParams;
using locomotion_ros2_runtime::TerrainBox;
using locomotion_ros2_runtime::TerrainModel;

TEST(FootstepTerrainPlanner, EmptyTerrainMatchesFlatPlan)
{
  FootstepPlanner planner;
  FootstepPlanParams params;
  params.step_count = 6;

  const auto flat = planner.plan(params);
  const auto terrain = planner.plan_over_terrain(params, TerrainModel{});

  ASSERT_EQ(terrain.plan.footsteps.size(), flat.footsteps.size());
  ASSERT_EQ(terrain.placements.size(), flat.footsteps.size());
  EXPECT_TRUE(terrain.fully_planned);
  for (std::size_t i = 0; i < flat.footsteps.size(); ++i) {
    EXPECT_DOUBLE_EQ(terrain.plan.footsteps[i].pose.position.x, flat.footsteps[i].pose.position.x);
    EXPECT_DOUBLE_EQ(terrain.plan.footsteps[i].pose.position.y, flat.footsteps[i].pose.position.y);
    EXPECT_DOUBLE_EQ(terrain.plan.footsteps[i].pose.position.z, 0.0);
    EXPECT_FLOAT_EQ(terrain.plan.footsteps[i].swing_height, flat.footsteps[i].swing_height);
    EXPECT_FALSE(terrain.placements[i].adjusted);
    EXPECT_FALSE(terrain.placements[i].blocked);
  }
}

TEST(FootstepTerrainPlanner, NudgesFootAroundKeepOutZone)
{
  FootstepPlanner planner;
  FootstepPlanParams params;
  params.step_count = 6;
  // First left foot lands near (0.1, 0.16); box it in.
  TerrainModel terrain;
  terrain.add_box(TerrainBox{0.0, 0.10, 0.2, 0.22, 0.0, true});

  const auto result = planner.plan_over_terrain(params, terrain);

  ASSERT_GE(result.plan.footsteps.size(), 1u);
  EXPECT_TRUE(result.fully_planned);
  EXPECT_TRUE(result.placements[0].adjusted);
  // The chosen foothold must be clear of the keep-out box.
  const auto & foot = result.plan.footsteps[0];
  EXPECT_FALSE(terrain.is_no_step(foot.pose.position.x, foot.pose.position.y));
  EXPECT_GT(std::abs(result.placements[0].lateral_adjust), 0.0);
}

TEST(FootstepTerrainPlanner, BlocksWhenNoFootholdInSearchWindow)
{
  FootstepPlanner planner;
  FootstepPlanParams params;
  params.step_count = 6;
  // A keep-out band wide enough that no lateral nudge escapes it.
  TerrainModel terrain;
  terrain.add_box(TerrainBox{0.0, -1.0, 0.2, 1.0, 0.0, true});

  const auto result = planner.plan_over_terrain(params, terrain);

  ASSERT_GE(result.placements.size(), 1u);
  EXPECT_TRUE(result.placements[0].blocked);
  EXPECT_FALSE(result.fully_planned);
}

TEST(FootstepTerrainPlanner, RaisesFootAndSwingOnCurb)
{
  FootstepPlanner planner;
  FootstepPlanParams params;
  params.step_count = 6;
  params.swing_height = 0.05;
  // Curb covering footsteps at x = 0.3, 0.4, 0.5 (height 0.12).
  TerrainModel terrain;
  terrain.add_box(TerrainBox{0.25, -1.0, 0.55, 1.0, 0.12, false});

  const auto result = planner.plan_over_terrain(params, terrain);
  ASSERT_EQ(result.plan.footsteps.size(), 6u);

  // x = 0.3 is the step-up onto the curb: raised foot AND raised swing.
  const auto & step_up = result.plan.footsteps[2];
  EXPECT_NEAR(step_up.pose.position.x, 0.3, 1e-9);
  EXPECT_NEAR(step_up.pose.position.z, 0.12, 1e-9);
  EXPECT_GT(step_up.swing_height, 0.12f);  // base + rise + clearance

  // x = 0.4 is already on the curb: raised foot, nominal swing.
  const auto & on_curb = result.plan.footsteps[3];
  EXPECT_NEAR(on_curb.pose.position.z, 0.12, 1e-9);
  EXPECT_NEAR(on_curb.swing_height, 0.05f, 1e-4);

  // x = 0.6 steps back down to the ground: no swing penalty.
  const auto & step_down = result.plan.footsteps[5];
  EXPECT_NEAR(step_down.pose.position.z, 0.0, 1e-9);
  EXPECT_NEAR(step_down.swing_height, 0.05f, 1e-4);
}

TEST(FootstepTerrainPlanner, UsesTerrainPlannerId)
{
  FootstepPlanner planner;
  FootstepPlanParams params;
  TerrainModel terrain;
  terrain.add_box(TerrainBox{0.0, -1.0, 0.2, 1.0, 0.1, false});

  const auto result = planner.plan_over_terrain(params, terrain);
  EXPECT_EQ(result.plan.planner_id, "locomotion_ros2_terrain_planner");
}
