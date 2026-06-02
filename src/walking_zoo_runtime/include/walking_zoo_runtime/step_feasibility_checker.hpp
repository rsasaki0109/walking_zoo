#ifndef WALKING_ZOO_RUNTIME__STEP_FEASIBILITY_CHECKER_HPP_
#define WALKING_ZOO_RUNTIME__STEP_FEASIBILITY_CHECKER_HPP_

#include <string>
#include <vector>

#include "walking_zoo_msgs/msg/footstep_plan.hpp"

namespace walking_zoo_runtime
{

// Conservative kinematic limits for a placeholder feasibility check. This does
// not model terrain, balance, or collisions; it only rejects obviously
// out-of-range placeholder footsteps so visualizers and mock adapters can flag
// them before a real planner exists.
struct StepFeasibilityLimits
{
  double max_step_distance{0.45};   // max move from the previous footstep, metres
  double max_lateral{0.40};         // max |y| from the centre line, metres
  double max_swing_height{0.20};    // max swing apex, metres
};

struct StepFeasibility
{
  bool feasible{true};
  std::string reason;
};

struct PlanFeasibility
{
  bool feasible{true};
  std::vector<StepFeasibility> steps;
};

class StepFeasibilityChecker
{
public:
  PlanFeasibility evaluate(
    const walking_zoo_msgs::msg::FootstepPlan & plan,
    const StepFeasibilityLimits & limits) const;
};

}  // namespace walking_zoo_runtime

#endif  // WALKING_ZOO_RUNTIME__STEP_FEASIBILITY_CHECKER_HPP_
