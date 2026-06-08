#include "locomotion_ros2_runtime/step_feasibility_checker.hpp"

#include <cmath>

namespace locomotion_ros2_runtime
{

PlanFeasibility StepFeasibilityChecker::evaluate(
  const locomotion_ros2_msgs::msg::FootstepPlan & plan,
  const StepFeasibilityLimits & limits) const
{
  PlanFeasibility result;

  double prev_x = 0.0;
  double prev_y = 0.0;
  bool have_prev = false;

  for (const auto & footstep : plan.footsteps) {
    StepFeasibility step;

    const double x = footstep.pose.position.x;
    const double y = footstep.pose.position.y;

    if (have_prev) {
      const double dx = x - prev_x;
      const double dy = y - prev_y;
      const double distance = std::sqrt(dx * dx + dy * dy);
      if (distance > limits.max_step_distance) {
        step.feasible = false;
        step.reason = "step distance exceeds limit";
      }
    }

    if (step.feasible && std::abs(y) > limits.max_lateral) {
      step.feasible = false;
      step.reason = "lateral offset exceeds limit";
    }

    if (step.feasible && footstep.swing_height > limits.max_swing_height) {
      step.feasible = false;
      step.reason = "swing height exceeds limit";
    }

    if (!step.feasible) {
      result.feasible = false;
    }

    result.steps.push_back(step);
    prev_x = x;
    prev_y = y;
    have_prev = true;
  }

  return result;
}

}  // namespace locomotion_ros2_runtime
