#include <gtest/gtest.h>

#include "walking_zoo_runtime/terrain_model.hpp"

using walking_zoo_runtime::TerrainBox;
using walking_zoo_runtime::TerrainModel;

TEST(TerrainModel, EmptyModelIsFlatGround)
{
  TerrainModel terrain;
  EXPECT_TRUE(terrain.empty());
  EXPECT_DOUBLE_EQ(terrain.height_at(1.0, 2.0), 0.0);
  EXPECT_FALSE(terrain.is_no_step(1.0, 2.0));
}

TEST(TerrainModel, GroundHeightApplies)
{
  TerrainModel terrain;
  terrain.set_ground_height(0.5);
  EXPECT_DOUBLE_EQ(terrain.height_at(0.0, 0.0), 0.5);
}

TEST(TerrainModel, CurbRaisesHeightInsideBox)
{
  TerrainModel terrain;
  TerrainBox curb;
  curb.min_x = 0.5;
  curb.min_y = -0.5;
  curb.max_x = 1.5;
  curb.max_y = 0.5;
  curb.height = 0.12;
  terrain.add_box(curb);

  EXPECT_DOUBLE_EQ(terrain.height_at(1.0, 0.0), 0.12);
  EXPECT_DOUBLE_EQ(terrain.height_at(0.4, 0.0), 0.0);  // just outside
  EXPECT_FALSE(terrain.is_no_step(1.0, 0.0));          // a curb is steppable
}

TEST(TerrainModel, TallestBoxWins)
{
  TerrainModel terrain;
  terrain.add_box(TerrainBox{0.0, -1.0, 2.0, 1.0, 0.05, false});
  terrain.add_box(TerrainBox{0.5, -1.0, 1.5, 1.0, 0.20, false});
  EXPECT_DOUBLE_EQ(terrain.height_at(1.0, 0.0), 0.20);
  EXPECT_DOUBLE_EQ(terrain.height_at(0.1, 0.0), 0.05);
}

TEST(TerrainModel, NoStepZoneBlocksFootholds)
{
  TerrainModel terrain;
  TerrainBox keepout;
  keepout.min_x = 0.0;
  keepout.min_y = -0.1;
  keepout.max_x = 1.0;
  keepout.max_y = 0.1;
  keepout.no_step = true;
  terrain.add_box(keepout);

  EXPECT_TRUE(terrain.is_no_step(0.5, 0.0));
  EXPECT_FALSE(terrain.is_no_step(0.5, 0.5));
  // A keep-out box with no height does not raise the ground.
  EXPECT_DOUBLE_EQ(terrain.height_at(0.5, 0.0), 0.0);
}

TEST(TerrainModel, BoundaryIsInclusive)
{
  TerrainModel terrain;
  terrain.add_box(TerrainBox{0.0, 0.0, 1.0, 1.0, 0.1, false});
  EXPECT_DOUBLE_EQ(terrain.height_at(0.0, 0.0), 0.1);
  EXPECT_DOUBLE_EQ(terrain.height_at(1.0, 1.0), 0.1);
}

namespace
{
// A 4x2 grid at origin (0,0) with 0.5 m cells. The cell at col=1,row=0 (covering
// x in [0.5,1.0), y in [0,0.5)) is lethal; everything else is free.
walking_zoo_runtime::TerrainGrid make_grid()
{
  walking_zoo_runtime::TerrainGrid grid;
  grid.origin_x = 0.0;
  grid.origin_y = 0.0;
  grid.resolution = 0.5;
  grid.width = 4;
  grid.height = 2;
  grid.occupied_threshold = 50;
  grid.occupancy = {0, 100, 0, 0,
                    0, 0, 0, 0};
  return grid;
}
}  // namespace

TEST(TerrainModel, GridOccupiedCellBlocksFoothold)
{
  TerrainModel terrain;
  terrain.set_grid(make_grid());
  EXPECT_TRUE(terrain.has_grid());
  EXPECT_FALSE(terrain.empty());

  EXPECT_TRUE(terrain.is_no_step(0.6, 0.2));    // inside the lethal cell
  EXPECT_FALSE(terrain.is_no_step(0.2, 0.2));   // free cell to the left
  EXPECT_FALSE(terrain.is_no_step(0.6, 0.7));   // free cell below
}

TEST(TerrainModel, GridQueriesOutsideBoundsAreFlatAndFree)
{
  TerrainModel terrain;
  terrain.set_grid(make_grid());
  EXPECT_FALSE(terrain.is_no_step(-0.5, 0.2));   // left of the grid
  EXPECT_FALSE(terrain.is_no_step(5.0, 0.2));    // right of the grid
  EXPECT_DOUBLE_EQ(terrain.height_at(5.0, 0.2), 0.0);
}

TEST(TerrainModel, GridUnknownCellsBlockOnlyWhenConfigured)
{
  auto grid = make_grid();
  grid.occupancy[2] = -1;  // col=2,row=0 unknown
  TerrainModel relaxed;
  relaxed.set_grid(grid);
  EXPECT_FALSE(relaxed.is_no_step(1.2, 0.2));  // unknown allowed by default

  grid.unknown_is_no_step = true;
  TerrainModel strict;
  strict.set_grid(grid);
  EXPECT_TRUE(strict.is_no_step(1.2, 0.2));
}

TEST(TerrainModel, GridElevationRaisesHeight)
{
  auto grid = make_grid();
  grid.elevation.assign(grid.width * grid.height, 0.0f);
  grid.elevation[1] = 0.12f;  // raise the same cell as the lethal one
  TerrainModel terrain;
  terrain.set_grid(grid);

  EXPECT_NEAR(terrain.height_at(0.6, 0.2), 0.12, 1e-5);
  EXPECT_DOUBLE_EQ(terrain.height_at(0.2, 0.2), 0.0);
}

TEST(TerrainModel, GridAndBoxesCombine)
{
  TerrainModel terrain;
  terrain.set_grid(make_grid());
  TerrainBox keepout;
  keepout.min_x = 1.4;
  keepout.min_y = 0.0;
  keepout.max_x = 1.6;
  keepout.max_y = 0.5;
  keepout.no_step = true;
  terrain.add_box(keepout);

  EXPECT_TRUE(terrain.is_no_step(0.6, 0.2));   // blocked by the grid cell
  EXPECT_TRUE(terrain.is_no_step(1.5, 0.2));   // blocked by the box
  EXPECT_FALSE(terrain.is_no_step(0.2, 0.2));  // clear of both

  terrain.clear_grid();
  EXPECT_FALSE(terrain.is_no_step(0.6, 0.2));  // grid gone, box remains
  EXPECT_TRUE(terrain.is_no_step(1.5, 0.2));
}
