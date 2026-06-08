#include "locomotion_ros2_runtime/footstep_planner.hpp"

#include <algorithm>
#include <cmath>

namespace locomotion_ros2_runtime
{

void FootstepPlanner::nominal_footstep(
  const FootstepPlanParams & params, std::size_t step,
  std::string & leg_name, double & x, double & y) const
{
  const std::string other_leg = (params.start_leg == "left") ? "right" : "left";
  const bool is_start_leg = (step % 2 == 0);
  leg_name = is_start_leg ? params.start_leg : other_leg;

  // Feet leapfrog forward half a stride at a time so the body advances one
  // stride per left/right pair.
  x = static_cast<double>(step + 1) * params.stride_length * 0.5;

  const double lateral_sign = (leg_name == "left") ? 1.0 : -1.0;
  y = lateral_sign * params.stride_width +
    params.lateral_shift * static_cast<double>(step + 1) * 0.5;
}

locomotion_ros2_msgs::msg::FootstepPlan FootstepPlanner::plan(
  const FootstepPlanParams & params) const
{
  locomotion_ros2_msgs::msg::FootstepPlan plan;
  plan.frame_id = params.frame_id;
  plan.header.frame_id = params.frame_id;
  plan.planner_id = params.planner_id;
  plan.nominal_duration =
    static_cast<float>(params.step_count) * static_cast<float>(params.step_duration);

  for (std::size_t step = 0; step < params.step_count; ++step) {
    locomotion_ros2_msgs::msg::Footstep footstep;
    footstep.header.frame_id = params.frame_id;

    nominal_footstep(params, step, footstep.leg_name,
      footstep.pose.position.x, footstep.pose.position.y);
    footstep.pose.position.z = 0.0;
    footstep.pose.orientation.w = 1.0;

    footstep.swing_height = static_cast<float>(params.swing_height);
    footstep.duration = static_cast<float>(params.step_duration);
    footstep.is_support = false;

    plan.footsteps.push_back(footstep);
  }

  return plan;
}

TerrainAwarePlan FootstepPlanner::plan_over_terrain(
  const FootstepPlanParams & params,
  const TerrainModel & terrain,
  const FootstepSearchConfig & search) const
{
  TerrainAwarePlan result;
  result.plan.frame_id = params.frame_id;
  result.plan.header.frame_id = params.frame_id;
  result.plan.planner_id = terrain.empty() ? params.planner_id : "locomotion_ros2_terrain_planner";
  result.plan.nominal_duration =
    static_cast<float>(params.step_count) * static_cast<float>(params.step_duration);

  double prev_height = terrain.height_at(0.0, 0.0);

  for (std::size_t step = 0; step < params.step_count; ++step) {
    locomotion_ros2_msgs::msg::Footstep footstep;
    footstep.header.frame_id = params.frame_id;

    double nominal_x = 0.0;
    double nominal_y = 0.0;
    nominal_footstep(params, step, footstep.leg_name, nominal_x, nominal_y);

    StepPlacement placement;
    double chosen_y = nominal_y;

    if (terrain.is_no_step(nominal_x, nominal_y)) {
      // Sweep symmetrically outward along the lateral axis for the nearest
      // foothold that is clear of every keep-out box.
      bool found = false;
      for (double offset = search.lateral_step;
        offset <= search.max_lateral_search + 1e-9;
        offset += search.lateral_step)
      {
        for (const double candidate : {nominal_y + offset, nominal_y - offset}) {
          if (!terrain.is_no_step(nominal_x, candidate)) {
            chosen_y = candidate;
            placement.adjusted = true;
            placement.lateral_adjust = candidate - nominal_y;
            found = true;
            break;
          }
        }
        if (found) {
          break;
        }
      }
      if (!found) {
        placement.blocked = true;
        result.fully_planned = false;
      }
    }

    const double height = terrain.height_at(nominal_x, chosen_y);
    placement.ground_height = height;

    footstep.pose.position.x = nominal_x;
    footstep.pose.position.y = chosen_y;
    footstep.pose.position.z = height;
    footstep.pose.orientation.w = 1.0;

    // Lift the foot higher when stepping up onto a raised patch so the swing
    // clears the rise. Stepping down keeps the nominal swing.
    const double rise = std::max(0.0, height - prev_height);
    double swing = params.swing_height + rise;
    if (rise > 0.0) {
      swing += search.step_clearance;
    }
    footstep.swing_height = static_cast<float>(swing);
    footstep.duration = static_cast<float>(params.step_duration);
    footstep.is_support = false;

    result.plan.footsteps.push_back(footstep);
    result.placements.push_back(placement);
    prev_height = height;
  }

  return result;
}

}  // namespace locomotion_ros2_runtime
