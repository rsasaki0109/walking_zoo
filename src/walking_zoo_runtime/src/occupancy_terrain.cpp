#include "walking_zoo_runtime/occupancy_terrain.hpp"

#include <cstddef>

namespace walking_zoo_runtime
{

TerrainGrid terrain_grid_from_costmap(
  const nav_msgs::msg::OccupancyGrid & costmap,
  std::int8_t occupied_threshold,
  bool unknown_is_no_step)
{
  TerrainGrid grid;
  grid.origin_x = costmap.info.origin.position.x;
  grid.origin_y = costmap.info.origin.position.y;
  grid.resolution = costmap.info.resolution;
  grid.width = costmap.info.width;
  grid.height = costmap.info.height;
  grid.occupied_threshold = occupied_threshold;
  grid.unknown_is_no_step = unknown_is_no_step;
  grid.occupancy.assign(costmap.data.begin(), costmap.data.end());
  return grid;
}

bool set_elevation_from_grid(
  TerrainGrid & grid,
  const nav_msgs::msg::OccupancyGrid & elevation,
  double height_per_unit)
{
  if (elevation.info.width != grid.width || elevation.info.height != grid.height) {
    return false;
  }
  const std::size_t cells = grid.width * grid.height;
  if (elevation.data.size() < cells) {
    return false;
  }
  grid.elevation.assign(cells, 0.0f);
  for (std::size_t i = 0; i < cells; ++i) {
    const std::int8_t value = elevation.data[i];
    if (value > 0) {
      grid.elevation[i] = static_cast<float>(static_cast<double>(value) * height_per_unit);
    }
  }
  return true;
}

}  // namespace walking_zoo_runtime
