#include "walking_zoo_runtime/footstep_planner.hpp"

namespace walking_zoo_runtime
{

walking_zoo_msgs::msg::FootstepPlan FootstepPlanner::plan(
  const FootstepPlanParams & params) const
{
  walking_zoo_msgs::msg::FootstepPlan plan;
  plan.frame_id = params.frame_id;
  plan.header.frame_id = params.frame_id;
  plan.planner_id = params.planner_id;
  plan.nominal_duration =
    static_cast<float>(params.step_count) * static_cast<float>(params.step_duration);

  const std::string other_leg = (params.start_leg == "left") ? "right" : "left";

  for (std::size_t step = 0; step < params.step_count; ++step) {
    walking_zoo_msgs::msg::Footstep footstep;
    footstep.header.frame_id = params.frame_id;

    const bool is_start_leg = (step % 2 == 0);
    footstep.leg_name = is_start_leg ? params.start_leg : other_leg;

    // Feet leapfrog forward half a stride at a time so the body advances one
    // stride per left/right pair.
    footstep.pose.position.x =
      static_cast<double>(step + 1) * params.stride_length * 0.5;

    const double lateral_sign = (footstep.leg_name == "left") ? 1.0 : -1.0;
    footstep.pose.position.y =
      lateral_sign * params.stride_width +
      params.lateral_shift * static_cast<double>(step + 1) * 0.5;
    footstep.pose.position.z = 0.0;
    footstep.pose.orientation.w = 1.0;

    footstep.swing_height = static_cast<float>(params.swing_height);
    footstep.duration = static_cast<float>(params.step_duration);
    footstep.is_support = false;

    plan.footsteps.push_back(footstep);
  }

  return plan;
}

}  // namespace walking_zoo_runtime
