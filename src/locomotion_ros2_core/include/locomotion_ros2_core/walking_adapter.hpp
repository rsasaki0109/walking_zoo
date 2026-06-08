#ifndef LOCOMOTION_ROS2_CORE__WALKING_ADAPTER_HPP_
#define LOCOMOTION_ROS2_CORE__WALKING_ADAPTER_HPP_

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "locomotion_ros2_core/adapter_context.hpp"
#include "locomotion_ros2_core/command_result.hpp"
#include "locomotion_ros2_core/types.hpp"
#include "locomotion_ros2_msgs/msg/adapter_status.hpp"
#include "locomotion_ros2_msgs/msg/body_pose_command.hpp"
#include "locomotion_ros2_msgs/msg/footstep_plan.hpp"
#include "locomotion_ros2_msgs/msg/walking_state.hpp"

namespace locomotion_ros2_core
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
  virtual locomotion_ros2_msgs::msg::AdapterStatus get_status() const = 0;
  virtual locomotion_ros2_msgs::msg::WalkingState read_state() = 0;

  virtual CommandResult command_velocity(
    const geometry_msgs::msg::TwistStamped & cmd) = 0;
  virtual CommandResult command_body_pose(
    const locomotion_ros2_msgs::msg::BodyPoseCommand & cmd) = 0;
  virtual CommandResult execute_footstep_plan(
    const locomotion_ros2_msgs::msg::FootstepPlan & plan) = 0;

  virtual CommandResult stop(StopMode mode) = 0;
  virtual CommandResult emergency_stop() = 0;
  virtual CommandResult clear_fault() = 0;
};

}  // namespace locomotion_ros2_core

#endif  // LOCOMOTION_ROS2_CORE__WALKING_ADAPTER_HPP_
