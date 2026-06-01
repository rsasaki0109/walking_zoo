#ifndef WALKING_ZOO_CORE__WALKING_ADAPTER_HPP_
#define WALKING_ZOO_CORE__WALKING_ADAPTER_HPP_

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "walking_zoo_core/adapter_context.hpp"
#include "walking_zoo_core/command_result.hpp"
#include "walking_zoo_core/types.hpp"
#include "walking_zoo_msgs/msg/adapter_status.hpp"
#include "walking_zoo_msgs/msg/body_pose_command.hpp"
#include "walking_zoo_msgs/msg/footstep_plan.hpp"
#include "walking_zoo_msgs/msg/walking_state.hpp"

namespace walking_zoo_core
{

class WalkingAdapter
{
public:
  virtual ~WalkingAdapter() = default;

  virtual CallbackReturn configure(const AdapterContext & context) = 0;
  virtual CallbackReturn activate() = 0;
  virtual CallbackReturn deactivate() = 0;
  virtual CallbackReturn cleanup() = 0;

  virtual RobotProfile get_robot_profile() const = 0;
  virtual walking_zoo_msgs::msg::AdapterStatus get_status() const = 0;
  virtual walking_zoo_msgs::msg::WalkingState read_state() = 0;

  virtual CommandResult command_velocity(
    const geometry_msgs::msg::TwistStamped & cmd) = 0;
  virtual CommandResult command_body_pose(
    const walking_zoo_msgs::msg::BodyPoseCommand & cmd) = 0;
  virtual CommandResult execute_footstep_plan(
    const walking_zoo_msgs::msg::FootstepPlan & plan) = 0;

  virtual CommandResult stop(StopMode mode) = 0;
  virtual CommandResult emergency_stop() = 0;
  virtual CommandResult clear_fault() = 0;
};

}  // namespace walking_zoo_core

#endif  // WALKING_ZOO_CORE__WALKING_ADAPTER_HPP_
