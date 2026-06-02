#ifndef WALKING_ZOO_RUNTIME__FOOTSTEP_PLANNER_HPP_
#define WALKING_ZOO_RUNTIME__FOOTSTEP_PLANNER_HPP_

#include <cstddef>
#include <string>

#include "walking_zoo_msgs/msg/footstep_plan.hpp"

namespace walking_zoo_runtime
{

// Parameters for a simple alternating-leg footstep plan. This is a deterministic
// placeholder planner intended for visualization and mock/sim adapters; it does
// not check terrain or balance feasibility.
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
  std::string planner_id{"walking_zoo_stub_planner"};
};

class FootstepPlanner
{
public:
  walking_zoo_msgs::msg::FootstepPlan plan(const FootstepPlanParams & params) const;
};

}  // namespace walking_zoo_runtime

#endif  // WALKING_ZOO_RUNTIME__FOOTSTEP_PLANNER_HPP_
