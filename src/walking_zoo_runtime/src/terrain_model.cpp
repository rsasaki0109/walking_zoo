#include "walking_zoo_runtime/terrain_model.hpp"

namespace walking_zoo_runtime
{

bool TerrainModel::contains(const TerrainBox & box, double x, double y)
{
  return x >= box.min_x && x <= box.max_x && y >= box.min_y && y <= box.max_y;
}

double TerrainModel::height_at(double x, double y) const
{
  double height = ground_height_;
  for (const auto & box : boxes_) {
    if (contains(box, x, y) && box.height > height) {
      height = box.height;
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
  return false;
}

}  // namespace walking_zoo_runtime
