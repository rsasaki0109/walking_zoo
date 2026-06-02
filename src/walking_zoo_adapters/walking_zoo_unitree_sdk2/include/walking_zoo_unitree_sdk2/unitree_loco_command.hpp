#ifndef WALKING_ZOO_UNITREE_SDK2__UNITREE_LOCO_COMMAND_HPP_
#define WALKING_ZOO_UNITREE_SDK2__UNITREE_LOCO_COMMAND_HPP_

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "walking_zoo_msgs/msg/body_pose_command.hpp"

namespace walking_zoo_unitree_sdk2
{

// Coarse model of the Unitree G1 high-level locomotion FSM. The vendor
// LocoClient exposes a richer set of FSM ids; these are the modes the adapter
// needs to track so it can report locomotion state and gate transitions without
// linking the SDK.
enum class LocoMode
{
  ZERO_TORQUE,    // motors limp, no control
  DAMP,           // damped hold, entered on emergency stop
  BALANCE_STAND,  // standing and balancing, ready for body-pose control
  LOCOMOTION,     // walking / velocity tracking
};

// Conservative Unitree G1 high-level velocity envelope (m/s, rad/s). Forward and
// backward limits differ because the controller walks faster forward than back.
struct G1VelocityLimits
{
  double max_forward{0.6};
  double max_backward{0.4};
  double max_lateral{0.4};
  double max_yaw_rate{0.8};
};

// Translated velocity ready for the vendor LocoClient `Move(vx, vy, vyaw)` call.
struct LocoVelocityCommand
{
  double vx{0.0};
  double vy{0.0};
  double vyaw{0.0};
  bool clamped{false};
};

LocoVelocityCommand translate_velocity(
  const geometry_msgs::msg::TwistStamped & cmd,
  const G1VelocityLimits & limits);

// Body posture envelope for the G1 balance-stand pose, in radians and meters
// (height is relative to the nominal stand height).
struct G1PostureLimits
{
  double max_roll{0.3};
  double max_pitch{0.3};
  double max_yaw{0.6};
  double min_height{-0.2};
  double max_height{0.1};
};

struct LocoPostureCommand
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
  double height{0.0};
  bool clamped{false};
};

LocoPostureCommand translate_body_pose(
  const walking_zoo_msgs::msg::BodyPoseCommand & cmd,
  const G1PostureLimits & limits);

// Whether a transition into `target` is permitted from `current`. Damp and
// zero-torque are always reachable (safety); locomotion requires first being in
// balance-stand (or already locomoting).
bool loco_transition_allowed(LocoMode current, LocoMode target);

const char * to_string(LocoMode mode);

}  // namespace walking_zoo_unitree_sdk2

#endif  // WALKING_ZOO_UNITREE_SDK2__UNITREE_LOCO_COMMAND_HPP_
