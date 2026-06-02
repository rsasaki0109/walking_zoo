#ifndef WALKING_ZOO_RUNTIME__OCCUPANCY_TERRAIN_HPP_
#define WALKING_ZOO_RUNTIME__OCCUPANCY_TERRAIN_HPP_

#include <cstdint>

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "walking_zoo_runtime/terrain_model.hpp"

namespace walking_zoo_runtime
{

// Build a TerrainGrid keep-out channel from a Nav2-style costmap. The grid's
// geometry (origin, resolution, size) and cost cells are copied verbatim, so
// cells at or above `occupied_threshold` become keep-out footholds and the
// footstep planner dodges real obstacles. Grid yaw is ignored (assumed 0); the
// occupancy frame is the costmap's `header.frame_id`.
TerrainGrid terrain_grid_from_costmap(
  const nav_msgs::msg::OccupancyGrid & costmap,
  std::int8_t occupied_threshold = 50,
  bool unknown_is_no_step = false);

// Overlay a per-cell elevation channel from a second grid whose cost values are
// read as a coarse height field: metres = value * height_per_unit for value >= 0
// (negative/unknown cells stay at ground). The elevation grid must share the
// keep-out grid's geometry; mismatched geometry leaves elevation untouched and
// returns false.
bool set_elevation_from_grid(
  TerrainGrid & grid,
  const nav_msgs::msg::OccupancyGrid & elevation,
  double height_per_unit);

}  // namespace walking_zoo_runtime

#endif  // WALKING_ZOO_RUNTIME__OCCUPANCY_TERRAIN_HPP_
