#include "locomotion_ros2_unitree_sdk2/unitree_sdk2_adapter.hpp"

#include <cmath>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace locomotion_ros2_unitree_sdk2
{

UnitreeSdk2Adapter::UnitreeSdk2Adapter() = default;

bool UnitreeSdk2Adapter::dispatch_to_hardware() const
{
  return backend_ && backend_->dispatches_to_hardware() && allow_motion_;
}

bool UnitreeSdk2Adapter::should_forward() const
{
  // SIL backend: always forward so it records the command. Hardware backend:
  // forward only when motion is allowed.
  return backend_ && (!backend_->dispatches_to_hardware() || allow_motion_);
}

void UnitreeSdk2Adapter::enter_mode(LocoMode mode)
{
  if (loco_transition_allowed(loco_mode_, mode)) {
    loco_mode_ = mode;
    if (should_forward()) {
      backend_->set_mode(mode);
    }
  }
}

locomotion_ros2_core::CallbackReturn UnitreeSdk2Adapter::configure(
  const locomotion_ros2_core::AdapterContext & context)
{
  profile_ = context.robot_profile;
  profile_.adapter_plugin = "locomotion_ros2_unitree_sdk2/UnitreeSdk2Adapter";
  allow_motion_ = context.allow_motion;
  configured_ = true;
  loco_mode_ = LocoMode::ZERO_TORQUE;
  has_velocity_command_ = false;

  // Select the dispatch backend at compile time (SDK2 when built with vendor
  // support, otherwise software-in-the-loop) and bring its channel up.
  backend_ = make_loco_backend();
  backend_->connect(network_interface_);

  // Derive the G1 command envelopes from the robot profile so the runtime's
  // configured limits also bound what we translate toward the vendor controller.
  velocity_limits_.max_forward = std::abs(profile_.max_linear_x);
  velocity_limits_.max_backward = std::abs(profile_.max_linear_x);
  velocity_limits_.max_lateral = std::abs(profile_.max_linear_y);
  velocity_limits_.max_yaw_rate = std::abs(profile_.max_angular_z);
  posture_limits_.max_roll = std::abs(profile_.max_body_roll);
  posture_limits_.max_pitch = std::abs(profile_.max_body_pitch);

  if (backend_->dispatches_to_hardware()) {
    status_text_ = allow_motion_ ?
      "Unitree SDK2 adapter configured with motion enabled" :
      "Unitree SDK2 adapter configured with motion disabled";
  } else {
    status_text_ = "Unitree SDK2 adapter configured (software-in-the-loop: SDK not linked)";
  }
  return locomotion_ros2_core::CallbackReturn::SUCCESS;
}

locomotion_ros2_core::CallbackReturn UnitreeSdk2Adapter::activate()
{
  if (!configured_) {
    status_text_ = "Unitree SDK2 adapter not configured";
    return locomotion_ros2_core::CallbackReturn::FAILURE;
  }
  active_ = true;
  // The G1 stands up and balances before it will accept locomotion commands.
  enter_mode(LocoMode::BALANCE_STAND);
  status_text_ = "Unitree SDK2 adapter active (balance stand)";
  return locomotion_ros2_core::CallbackReturn::SUCCESS;
}

locomotion_ros2_core::CallbackReturn UnitreeSdk2Adapter::deactivate()
{
  active_ = false;
  has_velocity_command_ = false;
  enter_mode(LocoMode::DAMP);
  return locomotion_ros2_core::CallbackReturn::SUCCESS;
}

locomotion_ros2_core::CallbackReturn UnitreeSdk2Adapter::cleanup()
{
  configured_ = false;
  active_ = false;
  has_velocity_command_ = false;
  loco_mode_ = LocoMode::ZERO_TORQUE;
  return locomotion_ros2_core::CallbackReturn::SUCCESS;
}

locomotion_ros2_core::RobotProfile UnitreeSdk2Adapter::get_robot_profile() const
{
  return profile_;
}

locomotion_ros2_msgs::msg::AdapterStatus UnitreeSdk2Adapter::get_status() const
{
  locomotion_ros2_msgs::msg::AdapterStatus status;
  status.status = estop_active_ ?
    locomotion_ros2_msgs::msg::AdapterStatus::STATUS_ESTOPPED :
    (active_ ? locomotion_ros2_msgs::msg::AdapterStatus::STATUS_ACTIVE :
    locomotion_ros2_msgs::msg::AdapterStatus::STATUS_CONNECTED);
  status.connected = configured_;
  status.active = active_;
  status.allow_motion = allow_motion_;
  status.adapter_name = "locomotion_ros2_unitree_sdk2/UnitreeSdk2Adapter";
  status.robot_model = profile_.robot_model;
  status.hardware_id = "unitree_sdk2";
  status.status_text = status_text_;
  return status;
}

locomotion_ros2_msgs::msg::WalkingState UnitreeSdk2Adapter::read_state()
{
  locomotion_ros2_msgs::msg::WalkingState state;
  state.lifecycle_state = active_ ?
    locomotion_ros2_msgs::msg::WalkingState::LIFECYCLE_ACTIVE :
    locomotion_ros2_msgs::msg::WalkingState::LIFECYCLE_INACTIVE;

  const bool moving = has_velocity_command_ &&
    (std::abs(last_velocity_.vx) > 1e-6 || std::abs(last_velocity_.vy) > 1e-6 ||
    std::abs(last_velocity_.vyaw) > 1e-6);

  if (estop_active_) {
    state.locomotion_state = locomotion_ros2_msgs::msg::WalkingState::STATE_ESTOPPED;
    state.locomotion_mode = locomotion_ros2_msgs::msg::WalkingState::MODE_IDLE;
    state.support_phase = locomotion_ros2_msgs::msg::WalkingState::SUPPORT_UNKNOWN;
  } else {
    switch (loco_mode_) {
      case LocoMode::ZERO_TORQUE:
        state.locomotion_state = locomotion_ros2_msgs::msg::WalkingState::STATE_IDLE;
        state.locomotion_mode = locomotion_ros2_msgs::msg::WalkingState::MODE_IDLE;
        state.support_phase = locomotion_ros2_msgs::msg::WalkingState::SUPPORT_UNKNOWN;
        break;
      case LocoMode::DAMP:
        state.locomotion_state = locomotion_ros2_msgs::msg::WalkingState::STATE_STOPPING;
        state.locomotion_mode = locomotion_ros2_msgs::msg::WalkingState::MODE_IDLE;
        state.support_phase = locomotion_ros2_msgs::msg::WalkingState::SUPPORT_UNKNOWN;
        break;
      case LocoMode::BALANCE_STAND:
        state.locomotion_state = locomotion_ros2_msgs::msg::WalkingState::STATE_STANDING;
        state.locomotion_mode = locomotion_ros2_msgs::msg::WalkingState::MODE_STAND;
        state.support_phase = locomotion_ros2_msgs::msg::WalkingState::SUPPORT_DOUBLE;
        break;
      case LocoMode::LOCOMOTION:
        state.locomotion_state = moving ?
          locomotion_ros2_msgs::msg::WalkingState::STATE_WALKING :
          locomotion_ros2_msgs::msg::WalkingState::STATE_STANDING;
        state.locomotion_mode = locomotion_ros2_msgs::msg::WalkingState::MODE_WALK;
        state.support_phase = locomotion_ros2_msgs::msg::WalkingState::SUPPORT_DOUBLE;
        break;
    }
  }

  state.is_balanced =
    !estop_active_ && (loco_mode_ == LocoMode::BALANCE_STAND || loco_mode_ == LocoMode::LOCOMOTION);
  state.is_fallen = false;
  state.estop_active = estop_active_;
  state.adapter_connected = configured_;
  state.active_adapter = "locomotion_ros2_unitree_sdk2/UnitreeSdk2Adapter";
  state.active_robot_model = profile_.robot_model;
  state.status_text = status_text_;
  return state;
}

locomotion_ros2_core::CommandResult UnitreeSdk2Adapter::command_velocity(
  const geometry_msgs::msg::TwistStamped & cmd)
{
  if (!active_) {
    return locomotion_ros2_core::CommandResult::rejected("Unitree adapter inactive");
  }
  if (estop_active_) {
    return locomotion_ros2_core::CommandResult::blocked("estop active");
  }

  const auto loco = translate_velocity(cmd, velocity_limits_);
  if (!loco_transition_allowed(loco_mode_, LocoMode::LOCOMOTION)) {
    return locomotion_ros2_core::CommandResult::rejected(
      std::string("cannot enter locomotion from ") + to_string(loco_mode_));
  }
  enter_mode(LocoMode::LOCOMOTION);
  last_velocity_ = loco;
  has_velocity_command_ = true;

  if (should_forward()) {
    backend_->send_velocity(loco);
  }
  if (dispatch_to_hardware()) {
    status_text_ = "Unitree Move dispatched";
  } else if (backend_->dispatches_to_hardware()) {
    status_text_ = "Unitree Move withheld (allow_motion is false)";
  } else {
    status_text_ = "Unitree Move translated (software-in-the-loop)";
  }

  if (loco.clamped) {
    return locomotion_ros2_core::CommandResult::limited("velocity clamped to G1 envelope");
  }
  return locomotion_ros2_core::CommandResult::success(status_text_);
}

locomotion_ros2_core::CommandResult UnitreeSdk2Adapter::command_body_pose(
  const locomotion_ros2_msgs::msg::BodyPoseCommand & cmd)
{
  if (!active_) {
    return locomotion_ros2_core::CommandResult::rejected("Unitree adapter inactive");
  }
  if (estop_active_) {
    return locomotion_ros2_core::CommandResult::blocked("estop active");
  }

  const auto posture = translate_body_pose(cmd, posture_limits_);
  enter_mode(LocoMode::BALANCE_STAND);
  has_velocity_command_ = false;

  if (should_forward()) {
    backend_->send_posture(posture);
  }
  if (dispatch_to_hardware()) {
    status_text_ = "Unitree body pose dispatched";
  } else if (backend_->dispatches_to_hardware()) {
    status_text_ = "Unitree body pose withheld (allow_motion is false)";
  } else {
    status_text_ = "Unitree body pose translated (software-in-the-loop)";
  }

  if (posture.clamped) {
    return locomotion_ros2_core::CommandResult::limited("body pose clamped to G1 posture envelope");
  }
  return locomotion_ros2_core::CommandResult::success(status_text_);
}

locomotion_ros2_core::CommandResult UnitreeSdk2Adapter::execute_footstep_plan(
  const locomotion_ros2_msgs::msg::FootstepPlan & plan)
{
  (void)plan;
  // The G1 high-level LocoClient exposes velocity/posture control but no direct
  // footstep-placement interface, so this contract is honestly unsupported on
  // this adapter rather than silently faked.
  return locomotion_ros2_core::CommandResult::rejected(
    "Unitree G1 high-level API has no footstep interface");
}

locomotion_ros2_core::CommandResult UnitreeSdk2Adapter::stop(locomotion_ros2_core::StopMode mode)
{
  if (!configured_) {
    return locomotion_ros2_core::CommandResult::rejected("Unitree adapter unconfigured");
  }
  has_velocity_command_ = false;
  if (mode == locomotion_ros2_core::StopMode::EMERGENCY) {
    enter_mode(LocoMode::DAMP);
    status_text_ = "Unitree emergency stop (damp)";
  } else {
    enter_mode(LocoMode::BALANCE_STAND);
    status_text_ = "Unitree stop (balance stand)";
  }
  return locomotion_ros2_core::CommandResult::success(status_text_);
}

locomotion_ros2_core::CommandResult UnitreeSdk2Adapter::emergency_stop()
{
  estop_active_ = true;
  has_velocity_command_ = false;
  enter_mode(LocoMode::DAMP);
  // Emergency damp goes to hardware regardless of allow_motion: stopping is
  // always permitted.
  if (backend_) {
    backend_->emergency_damp();
  }
  status_text_ = "Unitree emergency stop requested";
  return locomotion_ros2_core::CommandResult::success(status_text_);
}

locomotion_ros2_core::CommandResult UnitreeSdk2Adapter::clear_fault()
{
  if (estop_active_) {
    return locomotion_ros2_core::CommandResult::blocked("clear estop before clearing fault");
  }
  if (active_) {
    enter_mode(LocoMode::BALANCE_STAND);
  }
  status_text_ = "Unitree fault cleared";
  return locomotion_ros2_core::CommandResult::success(status_text_);
}

}  // namespace locomotion_ros2_unitree_sdk2

PLUGINLIB_EXPORT_CLASS(
  locomotion_ros2_unitree_sdk2::UnitreeSdk2Adapter,
  locomotion_ros2_core::WalkingAdapter)
