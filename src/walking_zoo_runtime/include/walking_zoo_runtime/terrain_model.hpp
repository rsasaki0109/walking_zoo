#ifndef WALKING_ZOO_RUNTIME__TERRAIN_MODEL_HPP_
#define WALKING_ZOO_RUNTIME__TERRAIN_MODEL_HPP_

#include <vector>

namespace walking_zoo_runtime
{

// An axis-aligned terrain patch expressed in the footstep plan frame. A box can
// raise the ground (a curb or stair tread via `height`) and/or forbid foot
// placement inside it (`no_step`, e.g. a hole, a painted keep-out, or an
// obstacle footprint). This is still a coarse placeholder world model, not a
// real elevation map, but it is enough to drive terrain-aware foot placement.
struct TerrainBox
{
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
  double height{0.0};    // top surface height, metres (relative to ground)
  bool no_step{false};   // true => feet may not land inside this box
};

// A minimal terrain world made of stacked axis-aligned boxes over a flat
// ground. Queries are point-in-box tests, kept deliberately simple so the
// footstep planner stays deterministic and unit-testable without a real map.
class TerrainModel
{
public:
  void set_ground_height(double height) {ground_height_ = height;}
  void add_box(const TerrainBox & box) {boxes_.push_back(box);}
  void clear() {boxes_.clear();}

  bool empty() const {return boxes_.empty();}

  // Top surface height at (x, y): the tallest box covering the point, or the
  // ground height when no box covers it.
  double height_at(double x, double y) const;

  // True when any keep-out box covers (x, y) and a foot may not be placed there.
  bool is_no_step(double x, double y) const;

private:
  static bool contains(const TerrainBox & box, double x, double y);

  std::vector<TerrainBox> boxes_;
  double ground_height_{0.0};
};

}  // namespace walking_zoo_runtime

#endif  // WALKING_ZOO_RUNTIME__TERRAIN_MODEL_HPP_
