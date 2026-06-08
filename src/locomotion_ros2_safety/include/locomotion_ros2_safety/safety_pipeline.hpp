#ifndef LOCOMOTION_ROS2_SAFETY__SAFETY_PIPELINE_HPP_
#define LOCOMOTION_ROS2_SAFETY__SAFETY_PIPELINE_HPP_

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/time.hpp"
#include "locomotion_ros2_core/command_result.hpp"
#include "locomotion_ros2_msgs/msg/body_pose_command.hpp"
#include "locomotion_ros2_msgs/msg/safety_state.hpp"
#include "locomotion_ros2_safety/command_watchdog.hpp"
#include "locomotion_ros2_safety/estop_gate.hpp"
#include "locomotion_ros2_safety/fall_detector.hpp"
#include "locomotion_ros2_safety/velocity_limiter.hpp"

namespace locomotion_ros2_safety
{

struct SafetyResult
{
  locomotion_ros2_core::CommandResult result;
  geometry_msgs::msg::TwistStamped command;
};

struct BodyPoseSafetyResult
{
  locomotion_ros2_core::CommandResult result;
  locomotion_ros2_msgs::msg::BodyPoseCommand command;
};

class SafetyPipeline
{
public:
  SafetyPipeline();

  SafetyResult filter_velocity(
    const geometry_msgs::msg::TwistStamped & command,
    const rclcpp::Time & now) const;

  // Gate a body-pose command. The fall detector rejects gross over-tilt that
  // would topple the torso (checked on the raw request); per-axis roll/pitch
  // limits then clamp anything still beyond the configured safe band.
  BodyPoseSafetyResult filter_body_pose(
    const locomotion_ros2_msgs::msg::BodyPoseCommand & command) const;

  locomotion_ros2_msgs::msg::SafetyState make_state_msg() const;

  void set_limits(const VelocityLimits & limits);
  void set_body_pose_limits(double max_roll_rad, double max_pitch_rad);
  void set_fall_thresholds(double tilt_warn_rad, double tilt_fall_rad);
  FallState classify_tilt(double roll, double pitch) const;
  void set_estop_active(bool active);
  bool estop_active() const;
  void set_command_timeout_sec(double timeout_sec);

private:
  VelocityLimiter velocity_limiter_;
  EStopGate estop_gate_;
  CommandWatchdog watchdog_;
  FallDetector fall_detector_;
  double max_body_roll_{0.2};
  double max_body_pitch_{0.2};
};

}  // namespace locomotion_ros2_safety

#endif  // LOCOMOTION_ROS2_SAFETY__SAFETY_PIPELINE_HPP_
