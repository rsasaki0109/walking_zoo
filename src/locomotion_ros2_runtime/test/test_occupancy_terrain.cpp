// Tests that a real Nav2-style OccupancyGrid drives the terrain keep-out and
// elevation channels, and that the footstep planner reacts to a costmap-sourced
// obstacle exactly as it does to hand-authored boxes.

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "locomotion_ros2_runtime/footstep_planner.hpp"
#include "locomotion_ros2_runtime/occupancy_terrain.hpp"
#include "locomotion_ros2_runtime/terrain_model.hpp"

using locomotion_ros2_runtime::FootstepPlanner;
using locomotion_ros2_runtime::FootstepPlanParams;
using locomotion_ros2_runtime::TerrainModel;

namespace
{
// A costmap of `width`x`height` cells at the given origin/resolution, free
// everywhere except a lethal vertical stripe at column `lethal_col`.
nav_msgs::msg::OccupancyGrid make_costmap(
  unsigned width, unsigned height, double resolution,
  double origin_x, double origin_y, int lethal_col)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = resolution;
  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.data.assign(static_cast<std::size_t>(width) * height, 0);
  if (lethal_col >= 0) {
    for (unsigned row = 0; row < height; ++row) {
      grid.data[row * width + static_cast<unsigned>(lethal_col)] = 100;
    }
  }
  return grid;
}
}  // namespace

TEST(OccupancyTerrain, CostmapGeometryIsCopied)
{
  auto costmap = make_costmap(4, 2, 0.5, -1.0, -0.5, 1);
  const auto grid = locomotion_ros2_runtime::terrain_grid_from_costmap(costmap, 50);

  EXPECT_EQ(grid.width, 4u);
  EXPECT_EQ(grid.height, 2u);
  EXPECT_DOUBLE_EQ(grid.resolution, 0.5);
  EXPECT_DOUBLE_EQ(grid.origin_x, -1.0);
  EXPECT_DOUBLE_EQ(grid.origin_y, -0.5);
  ASSERT_EQ(grid.occupancy.size(), 8u);
  EXPECT_EQ(grid.occupancy[1], 100);
  EXPECT_EQ(grid.occupancy[0], 0);
}

TEST(OccupancyTerrain, CostmapDrivesKeepOut)
{
  auto costmap = make_costmap(4, 2, 0.5, 0.0, 0.0, 1);  // lethal col covers x in [0.5,1.0)
  TerrainModel terrain;
  terrain.set_grid(locomotion_ros2_runtime::terrain_grid_from_costmap(costmap, 50));

  EXPECT_TRUE(terrain.is_no_step(0.6, 0.2));
  EXPECT_FALSE(terrain.is_no_step(0.2, 0.2));
}

TEST(OccupancyTerrain, ThresholdControlsWhatCountsAsBlocked)
{
  auto costmap = make_costmap(2, 1, 0.5, 0.0, 0.0, -1);
  costmap.data[0] = 60;
  TerrainModel strict;
  strict.set_grid(locomotion_ros2_runtime::terrain_grid_from_costmap(costmap, 50));
  EXPECT_TRUE(strict.is_no_step(0.2, 0.2));

  TerrainModel lenient;
  lenient.set_grid(locomotion_ros2_runtime::terrain_grid_from_costmap(costmap, 80));
  EXPECT_FALSE(lenient.is_no_step(0.2, 0.2));
}

TEST(OccupancyTerrain, ElevationGridOverlaysHeight)
{
  auto costmap = make_costmap(2, 1, 0.5, 0.0, 0.0, -1);
  auto grid = locomotion_ros2_runtime::terrain_grid_from_costmap(costmap, 50);

  auto elevation = make_costmap(2, 1, 0.5, 0.0, 0.0, -1);
  elevation.data[1] = 80;  // 80 * 0.0015 m = 0.12 m
  ASSERT_TRUE(locomotion_ros2_runtime::set_elevation_from_grid(grid, elevation, 0.0015));

  TerrainModel terrain;
  terrain.set_grid(grid);
  EXPECT_NEAR(terrain.height_at(0.7, 0.2), 0.12, 1e-6);
  EXPECT_DOUBLE_EQ(terrain.height_at(0.2, 0.2), 0.0);
}

TEST(OccupancyTerrain, MismatchedElevationGeometryIsRejected)
{
  auto grid = locomotion_ros2_runtime::terrain_grid_from_costmap(
    make_costmap(2, 1, 0.5, 0.0, 0.0, -1), 50);
  auto wrong = make_costmap(3, 1, 0.5, 0.0, 0.0, -1);
  EXPECT_FALSE(locomotion_ros2_runtime::set_elevation_from_grid(grid, wrong, 0.0015));
  EXPECT_TRUE(grid.elevation.empty());
}

TEST(OccupancyTerrain, PlannerDodgesCostmapObstacle)
{
  // A fine grid (0.1 m cells) over x in [0,0.5), y in [-0.5,0.5). The planner
  // nudges feet only laterally, so block exactly the single cell the first
  // foothold lands in and leave the neighbouring lateral cells free so a nudge
  // can succeed. With stride_length 0.30 the first foot sits at x=0.15 (interior
  // of column 1, avoiding cell-boundary rounding) and y=+0.16 (row 6).
  nav_msgs::msg::OccupancyGrid costmap;
  costmap.header.frame_id = "map";
  costmap.info.width = 5;
  costmap.info.height = 10;
  costmap.info.resolution = 0.1;
  costmap.info.origin.position.x = 0.0;
  costmap.info.origin.position.y = -0.5;
  costmap.data.assign(5u * 10u, 0);
  costmap.data[6 * 5 + 1] = 100;  // col=1, row=6 lethal

  TerrainModel terrain;
  terrain.set_grid(locomotion_ros2_runtime::terrain_grid_from_costmap(costmap, 50));
  ASSERT_TRUE(terrain.is_no_step(0.15, 0.16));   // nominal foothold is blocked
  ASSERT_FALSE(terrain.is_no_step(0.15, 0.26));  // a lateral nudge is clear

  FootstepPlanParams params;
  params.step_count = 4;
  params.stride_length = 0.30;
  params.stride_width = 0.16;
  const auto result = FootstepPlanner{}.plan_over_terrain(params, terrain);

  EXPECT_EQ(result.plan.planner_id, "locomotion_ros2_terrain_planner");
  // The first foot must be nudged clear, and no placed foot may sit in a cell
  // the grid still reports as blocked.
  EXPECT_TRUE(result.placements[0].adjusted) << "first foot was not nudged off the obstacle";
  for (std::size_t i = 0; i < result.plan.footsteps.size(); ++i) {
    const auto & foot = result.plan.footsteps[i];
    if (!result.placements[i].blocked) {
      EXPECT_FALSE(terrain.is_no_step(foot.pose.position.x, foot.pose.position.y))
        << "placed foot " << i << " sits in a blocked cell";
    }
  }
}
