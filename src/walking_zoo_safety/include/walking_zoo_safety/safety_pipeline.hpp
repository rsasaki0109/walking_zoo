#ifndef WALKING_ZOO_SAFETY__SAFETY_PIPELINE_HPP_
#define WALKING_ZOO_SAFETY__SAFETY_PIPELINE_HPP_

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/time.hpp"
#include "walking_zoo_core/command_result.hpp"
#include "walking_zoo_msgs/msg/body_pose_command.hpp"
#include "walking_zoo_msgs/msg/safety_state.hpp"
#include "walking_zoo_safety/command_watchdog.hpp"
#include "walking_zoo_safety/estop_gate.hpp"
#include "walking_zoo_safety/fall_detector.hpp"
#include "walking_zoo_safety/velocity_limiter.hpp"

namespace walking_zoo_safety
{

struct SafetyResult
{
  walking_zoo_core::CommandResult result;
  geometry_msgs::msg::TwistStamped command;
};

struct BodyPoseSafetyResult
{
  walking_zoo_core::CommandResult result;
  walking_zoo_msgs::msg::BodyPoseCommand command;
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
    const walking_zoo_msgs::msg::BodyPoseCommand & command) const;

  walking_zoo_msgs::msg::SafetyState make_state_msg() const;

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

}  // namespace walking_zoo_safety

#endif  // WALKING_ZOO_SAFETY__SAFETY_PIPELINE_HPP_
