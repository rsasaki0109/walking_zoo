#ifndef LOCOMOTION_ROS2_RUNTIME__FOOTSTEP_PLANNER_HPP_
#define LOCOMOTION_ROS2_RUNTIME__FOOTSTEP_PLANNER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "locomotion_ros2_msgs/msg/footstep_plan.hpp"
#include "locomotion_ros2_runtime/terrain_model.hpp"

namespace locomotion_ros2_runtime
{

// Parameters for a simple alternating-leg footstep plan. This is a deterministic
// planner intended for visualization and mock/sim adapters. On flat ground it
// produces the original leapfrog gait; with a TerrainModel it additionally
// nudges feet around keep-out zones and lifts them onto raised patches.
struct FootstepPlanParams
{
  double stride_length{0.20};   // forward progress per step, metres
  double stride_width{0.16};    // lateral foot offset from the centre line, metres
  double lateral_shift{0.0};    // sideways drift per step (sidestep), metres
  double swing_height{0.05};    // nominal swing apex, metres
  double step_duration{0.6};    // seconds per step
  std::size_t step_count{6};
  std::string start_leg{"left"};
  std::string frame_id{"base_link"};
  std::string planner_id{"locomotion_ros2_stub_planner"};
};

// How hard the planner searches for an alternative foothold when the nominal
// placement lands inside a keep-out zone.
struct FootstepSearchConfig
{
  double lateral_step{0.04};        // candidate spacing along the lateral axis, metres
  double max_lateral_search{0.20};  // furthest lateral nudge tried, metres
  double step_clearance{0.03};      // extra swing apex added when stepping up, metres
};

// Per-step outcome of terrain-aware placement, parallel to plan.footsteps.
struct StepPlacement
{
  bool adjusted{false};         // foot was nudged laterally to dodge a keep-out
  bool blocked{false};          // no valid foothold found within the search window
  double lateral_adjust{0.0};   // signed lateral nudge applied, metres
  double ground_height{0.0};    // terrain height the foot was placed on, metres
};

struct TerrainAwarePlan
{
  locomotion_ros2_msgs::msg::FootstepPlan plan;
  std::vector<StepPlacement> placements;
  bool fully_planned{true};     // false when at least one step is blocked
};

class FootstepPlanner
{
public:
  // Flat-ground leapfrog plan (unchanged legacy behaviour).
  locomotion_ros2_msgs::msg::FootstepPlan plan(const FootstepPlanParams & params) const;

  // Terrain-aware plan: same nominal gait, but feet are moved out of keep-out
  // zones, placed on top of raised patches, and given extra swing to clear
  // height changes.
  TerrainAwarePlan plan_over_terrain(
    const FootstepPlanParams & params,
    const TerrainModel & terrain,
    const FootstepSearchConfig & search = FootstepSearchConfig{}) const;

private:
  // Nominal (terrain-free) foothold for a given step index.
  void nominal_footstep(
    const FootstepPlanParams & params, std::size_t step,
    std::string & leg_name, double & x, double & y) const;
};

}  // namespace locomotion_ros2_runtime

#endif  // LOCOMOTION_ROS2_RUNTIME__FOOTSTEP_PLANNER_HPP_
