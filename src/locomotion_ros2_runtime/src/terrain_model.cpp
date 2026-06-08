#include "locomotion_ros2_runtime/terrain_model.hpp"

#include <cmath>

namespace locomotion_ros2_runtime
{

bool TerrainModel::contains(const TerrainBox & box, double x, double y)
{
  return x >= box.min_x && x <= box.max_x && y >= box.min_y && y <= box.max_y;
}

bool TerrainModel::grid_index(double x, double y, std::size_t & index) const
{
  if (!has_grid_ || grid_.width == 0 || grid_.height == 0 || grid_.resolution <= 0.0) {
    return false;
  }
  const double fx = (x - grid_.origin_x) / grid_.resolution;
  const double fy = (y - grid_.origin_y) / grid_.resolution;
  if (fx < 0.0 || fy < 0.0) {
    return false;
  }
  const auto col = static_cast<std::size_t>(std::floor(fx));
  const auto row = static_cast<std::size_t>(std::floor(fy));
  if (col >= grid_.width || row >= grid_.height) {
    return false;
  }
  index = row * grid_.width + col;
  return true;
}

double TerrainModel::height_at(double x, double y) const
{
  double height = ground_height_;
  for (const auto & box : boxes_) {
    if (contains(box, x, y) && box.height > height) {
      height = box.height;
    }
  }
  std::size_t index = 0;
  if (!grid_.elevation.empty() && grid_index(x, y, index) && index < grid_.elevation.size()) {
    const double cell = static_cast<double>(grid_.elevation[index]);
    if (cell > height) {
      height = cell;
    }
  }
  return height;
}

bool TerrainModel::is_no_step(double x, double y) const
{
  for (const auto & box : boxes_) {
    if (box.no_step && contains(box, x, y)) {
      return true;
    }
  }
  std::size_t index = 0;
  if (!grid_.occupancy.empty() && grid_index(x, y, index) && index < grid_.occupancy.size()) {
    const std::int8_t cell = grid_.occupancy[index];
    if (cell < 0) {
      return grid_.unknown_is_no_step;
    }
    if (cell >= grid_.occupied_threshold) {
      return true;
    }
  }
  return false;
}

}  // namespace locomotion_ros2_runtime
