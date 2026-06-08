#include "locomotion_ros2_vla/semantic_action_mapper.hpp"

namespace locomotion_ros2_vla
{

SemanticMapping SemanticActionMapper::map(
  const locomotion_ros2_msgs::msg::SemanticAction & action) const
{
  SemanticMapping mapping;
  mapping.velocity.header = action.header;

  if (action.action == "stop") {
    mapping.recognized = true;
    mapping.stop = true;
    mapping.status_text = "mapped semantic stop";
    return mapping;
  }
  if (action.action == "move_forward" || action.action == "walk_forward") {
    mapping.recognized = true;
    mapping.velocity.twist.linear.x = 0.2;
    mapping.status_text = "mapped move_forward to conservative velocity";
    return mapping;
  }
  if (
    action.action == "slow_careful_walk" || action.action == "slow_walk" ||
    action.action == "careful_walk" || action.action == "slow_walk_forward")
  {
    mapping.recognized = true;
    mapping.velocity.twist.linear.x = 0.10;
    mapping.status_text = "mapped slow_careful_walk to cautious forward velocity";
    return mapping;
  }
  if (action.action == "run_forward") {
    mapping.recognized = true;
    mapping.velocity.twist.linear.x = 0.35;
    mapping.status_text = "mapped run_forward to faster forward velocity";
    return mapping;
  }
  if (action.action == "move_backward" || action.action == "walk_backward") {
    mapping.recognized = true;
    mapping.velocity.twist.linear.x = -0.15;
    mapping.status_text = "mapped move_backward to conservative reverse velocity";
    return mapping;
  }
  if (action.action == "sidestep_left") {
    mapping.recognized = true;
    mapping.velocity.twist.linear.y = 0.2;
    mapping.status_text = "mapped sidestep_left to conservative lateral velocity";
    return mapping;
  }
  if (action.action == "sidestep_right") {
    mapping.recognized = true;
    mapping.velocity.twist.linear.y = -0.2;
    mapping.status_text = "mapped sidestep_right to conservative lateral velocity";
    return mapping;
  }
  if (action.action == "turn_left") {
    mapping.recognized = true;
    mapping.velocity.twist.angular.z = 0.3;
    mapping.status_text = "mapped turn_left to conservative yaw velocity";
    return mapping;
  }
  if (action.action == "turn_right") {
    mapping.recognized = true;
    mapping.velocity.twist.angular.z = -0.3;
    mapping.status_text = "mapped turn_right to conservative yaw velocity";
    return mapping;
  }

  mapping.status_text = "unrecognized semantic action";
  return mapping;
}

}  // namespace locomotion_ros2_vla
