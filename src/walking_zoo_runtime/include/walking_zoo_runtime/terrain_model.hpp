#ifndef WALKING_ZOO_RUNTIME__TERRAIN_MODEL_HPP_
#define WALKING_ZOO_RUNTIME__TERRAIN_MODEL_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace walking_zoo_runtime
{

// An axis-aligned terrain patch expressed in the footstep plan frame. A box can
// raise the ground (a curb or stair tread via `height`) and/or forbid foot
// placement inside it (`no_step`, e.g. a hole, a painted keep-out, or an
// obstacle footprint). Boxes remain a hand-authored coarse model; for real
// data, populate the grid below from a costmap / elevation source instead.
struct TerrainBox
{
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
  double height{0.0};    // top surface height, metres (relative to ground)
  bool no_step{false};   // true => feet may not land inside this box
};

// A regular grid sampled from a real map source. `occupancy` carries Nav2
// costmap-style values (-1 unknown, 0..100 cost) and any cell at or above
// `occupied_threshold` forbids foot placement, so a real costmap drives the
// keep-out zones. `elevation` (optional, metres) carries per-cell ground height
// from an elevation source, so a real height field drives step-ups. Either
// channel may be empty. The grid is axis-aligned (cell (col,row) covers
// [origin + col*res, origin + (col+1)*res) etc.); grid rotation is not modelled.
struct TerrainGrid
{
  double origin_x{0.0};
  double origin_y{0.0};
  double resolution{0.05};
  std::size_t width{0};
  std::size_t height{0};
  std::vector<std::int8_t> occupancy;   // size width*height, or empty
  std::int8_t occupied_threshold{50};
  bool unknown_is_no_step{false};       // treat -1 cells as keep-out
  std::vector<float> elevation;         // size width*height, or empty
};

// A terrain world made of stacked axis-aligned boxes plus an optional grid
// sampled from a real map source, over a flat ground. Box queries are
// point-in-box tests and grid queries are O(1) cell lookups, kept deterministic
// so the footstep planner stays unit-testable with or without a live map.
class TerrainModel
{
public:
  void set_ground_height(double height) {ground_height_ = height;}
  void add_box(const TerrainBox & box) {boxes_.push_back(box);}
  void set_grid(const TerrainGrid & grid) {grid_ = grid; has_grid_ = true;}
  void clear_grid() {has_grid_ = false; grid_ = TerrainGrid{};}
  void clear() {boxes_.clear(); clear_grid();}

  bool has_grid() const {return has_grid_;}
  bool empty() const {return boxes_.empty() && !has_grid_;}

  // Top surface height at (x, y): the tallest of the ground, any covering box,
  // and the grid elevation cell (when present).
  double height_at(double x, double y) const;

  // True when a keep-out box covers (x, y) or the grid cell at (x, y) is at or
  // above the occupied threshold (or unknown, when configured to block).
  bool is_no_step(double x, double y) const;

private:
  static bool contains(const TerrainBox & box, double x, double y);
  // Resolve (x, y) to a flat cell index; returns false when outside the grid.
  bool grid_index(double x, double y, std::size_t & index) const;

  std::vector<TerrainBox> boxes_;
  double ground_height_{0.0};
  TerrainGrid grid_;
  bool has_grid_{false};
};

}  // namespace walking_zoo_runtime

#endif  // WALKING_ZOO_RUNTIME__TERRAIN_MODEL_HPP_
