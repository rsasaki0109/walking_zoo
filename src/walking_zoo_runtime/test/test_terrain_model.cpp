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
