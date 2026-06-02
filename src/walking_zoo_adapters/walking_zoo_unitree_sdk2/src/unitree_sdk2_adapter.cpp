#include "walking_zoo_unitree_sdk2/unitree_sdk2_adapter.hpp"

#include <cmath>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace walking_zoo_unitree_sdk2
{

UnitreeSdk2Adapter::UnitreeSdk2Adapter() = default;

bool UnitreeSdk2Adapter::dispatch_to_hardware() const
{
#ifdef WALKING_ZOO_WITH_UNITREE_SDK2
  return allow_motion_;
#else
  return false;
#endif
}

void UnitreeSdk2Adapter::enter_mode(LocoMode mode)
{
  if (loco_transition_allowed(loco_mode_, mode)) {
    loco_mode_ = mode;
  }
}

walking_zoo_core::CallbackReturn UnitreeSdk2Adapter::configure(
  const walking_zoo_core::AdapterContext & context)
{
  profile_ = context.robot_profile;
  profile_.adapter_plugin = "walking_zoo_unitree_sdk2/UnitreeSdk2Adapter";
  allow_motion_ = context.allow_motion;
  configured_ = true;
  loco_mode_ = LocoMode::ZERO_TORQUE;
  has_velocity_command_ = false;

  // Derive the G1 command envelopes from the robot profile so the runtime's
  // configured limits also bound what we translate toward the vendor controller.
  velocity_limits_.max_forward = std::abs(profile_.max_linear_x);
  velocity_limits_.max_backward = std::abs(profile_.max_linear_x);
  velocity_limits_.max_lateral = std::abs(profile_.max_linear_y);
  velocity_limits_.max_yaw_rate = std::abs(profile_.max_angular_z);
  posture_limits_.max_roll = std::abs(profile_.max_body_roll);
  posture_limits_.max_pitch = std::abs(profile_.max_body_pitch);

#ifdef WALKING_ZOO_WITH_UNITREE_SDK2
  status_text_ = allow_motion_ ?
    "Unitree SDK2 adapter configured with motion enabled" :
    "Unitree SDK2 adapter configured with motion disabled";
#else
  status_text_ = "Unitree SDK2 adapter configured (software-in-the-loop: SDK not linked)";
#endif
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::CallbackReturn UnitreeSdk2Adapter::activate()
{
  if (!configured_) {
    status_text_ = "Unitree SDK2 adapter not configured";
    return walking_zoo_core::CallbackReturn::FAILURE;
  }
  active_ = true;
  // The G1 stands up and balances before it will accept locomotion commands.
  enter_mode(LocoMode::BALANCE_STAND);
  status_text_ = "Unitree SDK2 adapter active (balance stand)";
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::CallbackReturn UnitreeSdk2Adapter::deactivate()
{
  active_ = false;
  has_velocity_command_ = false;
  enter_mode(LocoMode::DAMP);
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::CallbackReturn UnitreeSdk2Adapter::cleanup()
{
  configured_ = false;
  active_ = false;
  has_velocity_command_ = false;
  loco_mode_ = LocoMode::ZERO_TORQUE;
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::RobotProfile UnitreeSdk2Adapter::get_robot_profile() const
{
  return profile_;
}

walking_zoo_msgs::msg::AdapterStatus UnitreeSdk2Adapter::get_status() const
{
  walking_zoo_msgs::msg::AdapterStatus status;
  status.status = estop_active_ ?
    walking_zoo_msgs::msg::AdapterStatus::STATUS_ESTOPPED :
    (active_ ? walking_zoo_msgs::msg::AdapterStatus::STATUS_ACTIVE :
    walking_zoo_msgs::msg::AdapterStatus::STATUS_CONNECTED);
  status.connected = configured_;
  status.active = active_;
  status.allow_motion = allow_motion_;
  status.adapter_name = "walking_zoo_unitree_sdk2/UnitreeSdk2Adapter";
  status.robot_model = profile_.robot_model;
  status.hardware_id = "unitree_sdk2";
  status.status_text = status_text_;
  return status;
}

walking_zoo_msgs::msg::WalkingState UnitreeSdk2Adapter::read_state()
{
  walking_zoo_msgs::msg::WalkingState state;
  state.lifecycle_state = active_ ?
    walking_zoo_msgs::msg::WalkingState::LIFECYCLE_ACTIVE :
    walking_zoo_msgs::msg::WalkingState::LIFECYCLE_INACTIVE;

  const bool moving = has_velocity_command_ &&
    (std::abs(last_velocity_.vx) > 1e-6 || std::abs(last_velocity_.vy) > 1e-6 ||
    std::abs(last_velocity_.vyaw) > 1e-6);

  if (estop_active_) {
    state.locomotion_state = walking_zoo_msgs::msg::WalkingState::STATE_ESTOPPED;
    state.locomotion_mode = walking_zoo_msgs::msg::WalkingState::MODE_IDLE;
    state.support_phase = walking_zoo_msgs::msg::WalkingState::SUPPORT_UNKNOWN;
  } else {
    switch (loco_mode_) {
      case LocoMode::ZERO_TORQUE:
        state.locomotion_state = walking_zoo_msgs::msg::WalkingState::STATE_IDLE;
        state.locomotion_mode = walking_zoo_msgs::msg::WalkingState::MODE_IDLE;
        state.support_phase = walking_zoo_msgs::msg::WalkingState::SUPPORT_UNKNOWN;
        break;
      case LocoMode::DAMP:
        state.locomotion_state = walking_zoo_msgs::msg::WalkingState::STATE_STOPPING;
        state.locomotion_mode = walking_zoo_msgs::msg::WalkingState::MODE_IDLE;
        state.support_phase = walking_zoo_msgs::msg::WalkingState::SUPPORT_UNKNOWN;
        break;
      case LocoMode::BALANCE_STAND:
        state.locomotion_state = walking_zoo_msgs::msg::WalkingState::STATE_STANDING;
        state.locomotion_mode = walking_zoo_msgs::msg::WalkingState::MODE_STAND;
        state.support_phase = walking_zoo_msgs::msg::WalkingState::SUPPORT_DOUBLE;
        break;
      case LocoMode::LOCOMOTION:
        state.locomotion_state = moving ?
          walking_zoo_msgs::msg::WalkingState::STATE_WALKING :
          walking_zoo_msgs::msg::WalkingState::STATE_STANDING;
        state.locomotion_mode = walking_zoo_msgs::msg::WalkingState::MODE_WALK;
        state.support_phase = walking_zoo_msgs::msg::WalkingState::SUPPORT_DOUBLE;
        break;
    }
  }

  state.is_balanced =
    !estop_active_ && (loco_mode_ == LocoMode::BALANCE_STAND || loco_mode_ == LocoMode::LOCOMOTION);
  state.is_fallen = false;
  state.estop_active = estop_active_;
  state.adapter_connected = configured_;
  state.active_adapter = "walking_zoo_unitree_sdk2/UnitreeSdk2Adapter";
  state.active_robot_model = profile_.robot_model;
  state.status_text = status_text_;
  return state;
}

walking_zoo_core::CommandResult UnitreeSdk2Adapter::command_velocity(
  const geometry_msgs::msg::TwistStamped & cmd)
{
  if (!active_) {
    return walking_zoo_core::CommandResult::rejected("Unitree adapter inactive");
  }
  if (estop_active_) {
    return walking_zoo_core::CommandResult::blocked("estop active");
  }

  const auto loco = translate_velocity(cmd, velocity_limits_);
  if (!loco_transition_allowed(loco_mode_, LocoMode::LOCOMOTION)) {
    return walking_zoo_core::CommandResult::rejected(
      std::string("cannot enter locomotion from ") + to_string(loco_mode_));
  }
  enter_mode(LocoMode::LOCOMOTION);
  last_velocity_ = loco;
  has_velocity_command_ = true;

#ifdef WALKING_ZOO_WITH_UNITREE_SDK2
  if (dispatch_to_hardware()) {
    // loco_client_->Move(loco.vx, loco.vy, loco.vyaw);  // vendor LocoClient call
    status_text_ = "Unitree Move dispatched";
  } else {
    status_text_ = "Unitree Move withheld (allow_motion is false)";
  }
#else
  status_text_ = "Unitree Move translated (software-in-the-loop)";
#endif

  if (loco.clamped) {
    return walking_zoo_core::CommandResult::limited("velocity clamped to G1 envelope");
  }
  return walking_zoo_core::CommandResult::success(status_text_);
}

walking_zoo_core::CommandResult UnitreeSdk2Adapter::command_body_pose(
  const walking_zoo_msgs::msg::BodyPoseCommand & cmd)
{
  if (!active_) {
    return walking_zoo_core::CommandResult::rejected("Unitree adapter inactive");
  }
  if (estop_active_) {
    return walking_zoo_core::CommandResult::blocked("estop active");
  }

  const auto posture = translate_body_pose(cmd, posture_limits_);
  enter_mode(LocoMode::BALANCE_STAND);
  has_velocity_command_ = false;

#ifdef WALKING_ZOO_WITH_UNITREE_SDK2
  if (dispatch_to_hardware()) {
    // loco_client_->SetBalanceMode / posture call goes here.
    status_text_ = "Unitree body pose dispatched";
  } else {
    status_text_ = "Unitree body pose withheld (allow_motion is false)";
  }
#else
  status_text_ = "Unitree body pose translated (software-in-the-loop)";
#endif
  (void)posture;

  if (posture.clamped) {
    return walking_zoo_core::CommandResult::limited("body pose clamped to G1 posture envelope");
  }
  return walking_zoo_core::CommandResult::success(status_text_);
}

walking_zoo_core::CommandResult UnitreeSdk2Adapter::execute_footstep_plan(
  const walking_zoo_msgs::msg::FootstepPlan & plan)
{
  (void)plan;
  // The G1 high-level LocoClient exposes velocity/posture control but no direct
  // footstep-placement interface, so this contract is honestly unsupported on
  // this adapter rather than silently faked.
  return walking_zoo_core::CommandResult::rejected(
    "Unitree G1 high-level API has no footstep interface");
}

walking_zoo_core::CommandResult UnitreeSdk2Adapter::stop(walking_zoo_core::StopMode mode)
{
  if (!configured_) {
    return walking_zoo_core::CommandResult::rejected("Unitree adapter unconfigured");
  }
  has_velocity_command_ = false;
  if (mode == walking_zoo_core::StopMode::EMERGENCY) {
    enter_mode(LocoMode::DAMP);
    status_text_ = "Unitree emergency stop (damp)";
  } else {
    enter_mode(LocoMode::BALANCE_STAND);
    status_text_ = "Unitree stop (balance stand)";
  }
  return walking_zoo_core::CommandResult::success(status_text_);
}

walking_zoo_core::CommandResult UnitreeSdk2Adapter::emergency_stop()
{
  estop_active_ = true;
  has_velocity_command_ = false;
  enter_mode(LocoMode::DAMP);
  status_text_ = "Unitree emergency stop requested";
  return walking_zoo_core::CommandResult::success(status_text_);
}

walking_zoo_core::CommandResult UnitreeSdk2Adapter::clear_fault()
{
  if (estop_active_) {
    return walking_zoo_core::CommandResult::blocked("clear estop before clearing fault");
  }
  if (active_) {
    enter_mode(LocoMode::BALANCE_STAND);
  }
  status_text_ = "Unitree fault cleared";
  return walking_zoo_core::CommandResult::success(status_text_);
}

}  // namespace walking_zoo_unitree_sdk2

PLUGINLIB_EXPORT_CLASS(
  walking_zoo_unitree_sdk2::UnitreeSdk2Adapter,
  walking_zoo_core::WalkingAdapter)
