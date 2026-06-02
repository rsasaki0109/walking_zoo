#include "walking_zoo_unitree_go2/unitree_go2_adapter.hpp"

#include <cmath>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace walking_zoo_unitree_go2
{

UnitreeGo2Adapter::UnitreeGo2Adapter() = default;

bool UnitreeGo2Adapter::dispatch_to_hardware() const
{
  return backend_ && backend_->dispatches_to_hardware() && allow_motion_;
}

bool UnitreeGo2Adapter::should_forward() const
{
  // SIL backend: always forward so it records the command. Hardware backend:
  // forward only when motion is allowed.
  return backend_ && (!backend_->dispatches_to_hardware() || allow_motion_);
}

void UnitreeGo2Adapter::enter_mode(SportMode mode)
{
  if (sport_transition_allowed(sport_mode_, mode)) {
    sport_mode_ = mode;
    if (should_forward()) {
      backend_->set_mode(mode);
    }
  }
}

walking_zoo_core::CallbackReturn UnitreeGo2Adapter::configure(
  const walking_zoo_core::AdapterContext & context)
{
  profile_ = context.robot_profile;
  profile_.adapter_plugin = "walking_zoo_unitree_go2/UnitreeGo2Adapter";
  allow_motion_ = context.allow_motion;
  configured_ = true;
  // A powered-on Go2 rests on the ground until commanded to stand.
  sport_mode_ = SportMode::STAND_DOWN;
  has_velocity_command_ = false;

  // Select the dispatch backend at compile time (SDK2 when built with vendor
  // support, otherwise software-in-the-loop) and bring its channel up.
  backend_ = make_sport_backend();
  backend_->connect(network_interface_);

  // Derive the Go2 command envelopes from the robot profile so the runtime's
  // configured limits also bound what we translate toward the vendor controller.
  velocity_limits_.max_forward = std::abs(profile_.max_linear_x);
  velocity_limits_.max_backward = std::abs(profile_.max_linear_x);
  velocity_limits_.max_lateral = std::abs(profile_.max_linear_y);
  velocity_limits_.max_yaw_rate = std::abs(profile_.max_angular_z);
  posture_limits_.max_roll = std::abs(profile_.max_body_roll);
  posture_limits_.max_pitch = std::abs(profile_.max_body_pitch);

  if (backend_->dispatches_to_hardware()) {
    status_text_ = allow_motion_ ?
      "Unitree Go2 adapter configured with motion enabled" :
      "Unitree Go2 adapter configured with motion disabled";
  } else {
    status_text_ = "Unitree Go2 adapter configured (software-in-the-loop: SDK not linked)";
  }
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::CallbackReturn UnitreeGo2Adapter::activate()
{
  if (!configured_) {
    status_text_ = "Unitree Go2 adapter not configured";
    return walking_zoo_core::CallbackReturn::FAILURE;
  }
  active_ = true;
  // The quadruped stands up from its resting pose and balances before it will
  // accept locomotion commands.
  enter_mode(SportMode::BALANCE_STAND);
  status_text_ = "Unitree Go2 adapter active (balance stand)";
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::CallbackReturn UnitreeGo2Adapter::deactivate()
{
  active_ = false;
  has_velocity_command_ = false;
  // A deactivated quadruped lies back down rather than damping in place.
  enter_mode(SportMode::STAND_DOWN);
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::CallbackReturn UnitreeGo2Adapter::cleanup()
{
  configured_ = false;
  active_ = false;
  has_velocity_command_ = false;
  sport_mode_ = SportMode::STAND_DOWN;
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::RobotProfile UnitreeGo2Adapter::get_robot_profile() const
{
  return profile_;
}

walking_zoo_msgs::msg::AdapterStatus UnitreeGo2Adapter::get_status() const
{
  walking_zoo_msgs::msg::AdapterStatus status;
  status.status = estop_active_ ?
    walking_zoo_msgs::msg::AdapterStatus::STATUS_ESTOPPED :
    (active_ ? walking_zoo_msgs::msg::AdapterStatus::STATUS_ACTIVE :
    walking_zoo_msgs::msg::AdapterStatus::STATUS_CONNECTED);
  status.connected = configured_;
  status.active = active_;
  status.allow_motion = allow_motion_;
  status.adapter_name = "walking_zoo_unitree_go2/UnitreeGo2Adapter";
  status.robot_model = profile_.robot_model;
  status.hardware_id = "unitree_sdk2";
  status.status_text = status_text_;
  return status;
}

walking_zoo_msgs::msg::WalkingState UnitreeGo2Adapter::read_state()
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
    switch (sport_mode_) {
      case SportMode::DAMP:
        state.locomotion_state = walking_zoo_msgs::msg::WalkingState::STATE_STOPPING;
        state.locomotion_mode = walking_zoo_msgs::msg::WalkingState::MODE_IDLE;
        state.support_phase = walking_zoo_msgs::msg::WalkingState::SUPPORT_UNKNOWN;
        break;
      case SportMode::STAND_DOWN:
        // Resting on the ground: a quadruped-specific sitting state.
        state.locomotion_state = walking_zoo_msgs::msg::WalkingState::STATE_SITTING;
        state.locomotion_mode = walking_zoo_msgs::msg::WalkingState::MODE_IDLE;
        state.support_phase = walking_zoo_msgs::msg::WalkingState::SUPPORT_UNKNOWN;
        break;
      case SportMode::BALANCE_STAND:
        state.locomotion_state = walking_zoo_msgs::msg::WalkingState::STATE_STANDING;
        state.locomotion_mode = walking_zoo_msgs::msg::WalkingState::MODE_STAND;
        state.support_phase = walking_zoo_msgs::msg::WalkingState::SUPPORT_QUADRUPED;
        break;
      case SportMode::LOCOMOTION:
        state.locomotion_state = moving ?
          walking_zoo_msgs::msg::WalkingState::STATE_WALKING :
          walking_zoo_msgs::msg::WalkingState::STATE_STANDING;
        state.locomotion_mode = walking_zoo_msgs::msg::WalkingState::MODE_WALK;
        state.support_phase = walking_zoo_msgs::msg::WalkingState::SUPPORT_QUADRUPED;
        break;
    }
  }

  state.is_balanced =
    !estop_active_ && (sport_mode_ == SportMode::BALANCE_STAND || sport_mode_ == SportMode::LOCOMOTION);
  state.is_fallen = false;
  state.estop_active = estop_active_;
  state.adapter_connected = configured_;
  state.active_adapter = "walking_zoo_unitree_go2/UnitreeGo2Adapter";
  state.active_robot_model = profile_.robot_model;
  state.status_text = status_text_;
  return state;
}

walking_zoo_core::CommandResult UnitreeGo2Adapter::command_velocity(
  const geometry_msgs::msg::TwistStamped & cmd)
{
  if (!active_) {
    return walking_zoo_core::CommandResult::rejected("Unitree Go2 adapter inactive");
  }
  if (estop_active_) {
    return walking_zoo_core::CommandResult::blocked("estop active");
  }

  const auto sport = translate_velocity(cmd, velocity_limits_);
  if (!sport_transition_allowed(sport_mode_, SportMode::LOCOMOTION)) {
    return walking_zoo_core::CommandResult::rejected(
      std::string("cannot trot from ") + to_string(sport_mode_) + " (stand up first)");
  }
  enter_mode(SportMode::LOCOMOTION);
  last_velocity_ = sport;
  has_velocity_command_ = true;

  if (should_forward()) {
    backend_->send_velocity(sport);
  }
  if (dispatch_to_hardware()) {
    status_text_ = "Unitree Go2 Move dispatched";
  } else if (backend_->dispatches_to_hardware()) {
    status_text_ = "Unitree Go2 Move withheld (allow_motion is false)";
  } else {
    status_text_ = "Unitree Go2 Move translated (software-in-the-loop)";
  }

  if (sport.clamped) {
    return walking_zoo_core::CommandResult::limited("velocity clamped to Go2 envelope");
  }
  return walking_zoo_core::CommandResult::success(status_text_);
}

walking_zoo_core::CommandResult UnitreeGo2Adapter::command_body_pose(
  const walking_zoo_msgs::msg::BodyPoseCommand & cmd)
{
  if (!active_) {
    return walking_zoo_core::CommandResult::rejected("Unitree Go2 adapter inactive");
  }
  if (estop_active_) {
    return walking_zoo_core::CommandResult::blocked("estop active");
  }

  const auto posture = translate_body_pose(cmd, posture_limits_);
  enter_mode(SportMode::BALANCE_STAND);
  has_velocity_command_ = false;

  if (should_forward()) {
    backend_->send_posture(posture);
  }
  if (dispatch_to_hardware()) {
    status_text_ = "Unitree Go2 body pose dispatched";
  } else if (backend_->dispatches_to_hardware()) {
    status_text_ = "Unitree Go2 body pose withheld (allow_motion is false)";
  } else {
    status_text_ = "Unitree Go2 body pose translated (software-in-the-loop)";
  }

  if (posture.clamped) {
    return walking_zoo_core::CommandResult::limited("body pose clamped to Go2 posture envelope");
  }
  return walking_zoo_core::CommandResult::success(status_text_);
}

walking_zoo_core::CommandResult UnitreeGo2Adapter::execute_footstep_plan(
  const walking_zoo_msgs::msg::FootstepPlan & plan)
{
  (void)plan;
  // The Go2 high-level sport API exposes velocity/posture control but no direct
  // footstep-placement interface, so this contract is honestly unsupported on
  // this adapter rather than silently faked.
  return walking_zoo_core::CommandResult::rejected(
    "Unitree Go2 high-level API has no footstep interface");
}

walking_zoo_core::CommandResult UnitreeGo2Adapter::stop(walking_zoo_core::StopMode mode)
{
  if (!configured_) {
    return walking_zoo_core::CommandResult::rejected("Unitree Go2 adapter unconfigured");
  }
  has_velocity_command_ = false;
  if (mode == walking_zoo_core::StopMode::EMERGENCY) {
    enter_mode(SportMode::DAMP);
    status_text_ = "Unitree Go2 emergency stop (damp)";
  } else if (mode == walking_zoo_core::StopMode::QUICK) {
    // A quick stop sits the quadruped down onto the ground.
    enter_mode(SportMode::STAND_DOWN);
    status_text_ = "Unitree Go2 quick stop (sit down)";
  } else {
    enter_mode(SportMode::BALANCE_STAND);
    status_text_ = "Unitree Go2 stop (balance stand)";
  }
  return walking_zoo_core::CommandResult::success(status_text_);
}

walking_zoo_core::CommandResult UnitreeGo2Adapter::emergency_stop()
{
  estop_active_ = true;
  has_velocity_command_ = false;
  enter_mode(SportMode::DAMP);
  // Emergency damp goes to hardware regardless of allow_motion: stopping is
  // always permitted.
  if (backend_) {
    backend_->emergency_damp();
  }
  status_text_ = "Unitree Go2 emergency stop requested";
  return walking_zoo_core::CommandResult::success(status_text_);
}

walking_zoo_core::CommandResult UnitreeGo2Adapter::clear_fault()
{
  if (estop_active_) {
    return walking_zoo_core::CommandResult::blocked("clear estop before clearing fault");
  }
  if (active_) {
    enter_mode(SportMode::BALANCE_STAND);
  }
  status_text_ = "Unitree Go2 fault cleared";
  return walking_zoo_core::CommandResult::success(status_text_);
}

}  // namespace walking_zoo_unitree_go2

PLUGINLIB_EXPORT_CLASS(
  walking_zoo_unitree_go2::UnitreeGo2Adapter,
  walking_zoo_core::WalkingAdapter)
