#ifndef LOCOMOTION_ROS2_UNITREE_GO2__GO2_SPORT_COMMAND_HPP_
#define LOCOMOTION_ROS2_UNITREE_GO2__GO2_SPORT_COMMAND_HPP_

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "locomotion_ros2_msgs/msg/body_pose_command.hpp"

namespace locomotion_ros2_unitree_go2
{

// Coarse model of the Unitree Go2 high-level "sport mode" FSM. The vendor
// SportClient exposes a richer set of states; these are the ones the quadruped
// adapter needs to track so it can report locomotion state and gate transitions
// without linking the SDK. Unlike the G1 humanoid model, the Go2 rests on the
// ground (STAND_DOWN) when idle and can self-right into a stand, so the FSM is
// genuinely different rather than a humanoid copy.
enum class SportMode
{
  DAMP,           // motors damped, entered on emergency stop
  STAND_DOWN,     // lying / sitting on the ground (powered-on rest pose)
  BALANCE_STAND,  // standing on all four feet, ready for velocity or posture
  LOCOMOTION,     // trotting / velocity tracking
};

// Conservative Unitree Go2 high-level velocity envelope (m/s, rad/s). The
// quadruped walks faster forward than back and yaws quickly; lateral crab-steps
// are slower. Asymmetric on purpose.
struct Go2VelocityLimits
{
  double max_forward{0.6};
  double max_backward{0.4};
  double max_lateral{0.4};
  double max_yaw_rate{0.9};
};

// Translated velocity ready for the vendor SportClient `Move(vx, vy, vyaw)` call.
struct Go2VelocityCommand
{
  double vx{0.0};
  double vy{0.0};
  double vyaw{0.0};
  bool clamped{false};
};

Go2VelocityCommand translate_velocity(
  const geometry_msgs::msg::TwistStamped & cmd,
  const Go2VelocityLimits & limits);

// Body-orientation envelope for the Go2 standing torso, in radians and meters
// (height is relative to the nominal stand height). Mapped to the SportClient
// `Euler(roll, pitch, yaw)` + `BodyHeight(height)` calls.
struct Go2PostureLimits
{
  double max_roll{0.3};
  double max_pitch{0.4};
  double max_yaw{0.4};
  double min_height{-0.1};
  double max_height{0.1};
};

struct Go2PostureCommand
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
  double height{0.0};
  bool clamped{false};
};

Go2PostureCommand translate_body_pose(
  const locomotion_ros2_msgs::msg::BodyPoseCommand & cmd,
  const Go2PostureLimits & limits);

// Whether a transition into `target` is permitted from `current`. Damp is always
// reachable (safety). The quadruped can recovery-stand into balance-stand from
// any state, can lie back down from a stand, but must be balance-standing before
// it will trot.
bool sport_transition_allowed(SportMode current, SportMode target);

const char * to_string(SportMode mode);

}  // namespace locomotion_ros2_unitree_go2

#endif  // LOCOMOTION_ROS2_UNITREE_GO2__GO2_SPORT_COMMAND_HPP_
