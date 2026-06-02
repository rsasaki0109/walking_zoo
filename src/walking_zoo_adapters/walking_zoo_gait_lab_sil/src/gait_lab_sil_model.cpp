#include "walking_zoo_gait_lab_sil/gait_lab_sil_model.hpp"

#include <cmath>

namespace walking_zoo_gait_lab_sil
{

namespace
{
using WalkingState = walking_zoo_msgs::msg::WalkingState;
using AdapterStatus = walking_zoo_msgs::msg::AdapterStatus;

bool is_nonzero_velocity(const geometry_msgs::msg::TwistStamped & cmd)
{
  return std::abs(cmd.twist.linear.x) > 1e-6 ||
         std::abs(cmd.twist.linear.y) > 1e-6 ||
         std::abs(cmd.twist.angular.z) > 1e-6;
}
}  // namespace

void GaitLabSilModel::configure(const walking_zoo_core::RobotProfile & profile)
{
  profile_ = profile;
  profile_.adapter_plugin = PLUGIN_NAME;
  configured_ = true;
  active_ = false;
  estop_active_ = false;
  fault_active_ = false;
  have_sim_state_ = false;
  status_text_ = "gait_lab SIL configured (awaiting sim)";
}

walking_zoo_core::CallbackReturn GaitLabSilModel::activate()
{
  if (!configured_) {
    status_text_ = "gait_lab SIL not configured";
    return walking_zoo_core::CallbackReturn::FAILURE;
  }
  active_ = true;
  status_text_ = "gait_lab SIL active";
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::CallbackReturn GaitLabSilModel::deactivate()
{
  active_ = false;
  status_text_ = "gait_lab SIL inactive";
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::CallbackReturn GaitLabSilModel::cleanup()
{
  configured_ = false;
  active_ = false;
  have_sim_state_ = false;
  status_text_ = "gait_lab SIL cleaned up";
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::CommandResult GaitLabSilModel::command_velocity_gate(
  const geometry_msgs::msg::TwistStamped & cmd)
{
  if (!active_) {
    return walking_zoo_core::CommandResult::rejected("gait_lab SIL inactive");
  }
  if (estop_active_) {
    return walking_zoo_core::CommandResult::blocked("gait_lab SIL estopped");
  }
  status_text_ = is_nonzero_velocity(cmd) ?
    "gait_lab SIL walking command forwarded" :
    "gait_lab SIL hold command forwarded";
  return walking_zoo_core::CommandResult::success(status_text_);
}

walking_zoo_core::CommandResult GaitLabSilModel::body_pose_gate()
{
  if (!active_) {
    return walking_zoo_core::CommandResult::rejected("gait_lab SIL inactive");
  }
  if (estop_active_) {
    return walking_zoo_core::CommandResult::blocked("gait_lab SIL estopped");
  }
  // The walking policy is a velocity gait; body-pose hold is not modelled.
  return walking_zoo_core::CommandResult::rejected(
    "gait_lab SIL does not implement body-pose control");
}

walking_zoo_core::CommandResult GaitLabSilModel::footstep_gate()
{
  if (!active_) {
    return walking_zoo_core::CommandResult::rejected("gait_lab SIL inactive");
  }
  return walking_zoo_core::CommandResult::rejected(
    "gait_lab SIL is a velocity gait; footstep plans are unsupported");
}

walking_zoo_core::CommandResult GaitLabSilModel::stop_gate(walking_zoo_core::StopMode mode)
{
  (void)mode;
  if (!configured_) {
    return walking_zoo_core::CommandResult::rejected("gait_lab SIL unconfigured");
  }
  status_text_ = "gait_lab SIL stop forwarded";
  return walking_zoo_core::CommandResult::success(status_text_);
}

walking_zoo_core::CommandResult GaitLabSilModel::emergency_stop_gate()
{
  estop_active_ = true;
  status_text_ = "gait_lab SIL emergency stop";
  return walking_zoo_core::CommandResult::success(status_text_);
}

walking_zoo_core::CommandResult GaitLabSilModel::clear_fault_gate()
{
  // Release the estop latch and clear the fault so the sim can stand again. The
  // operator-estop interlock lives one layer up in the runtime, not here.
  estop_active_ = false;
  fault_active_ = false;
  status_text_ = "gait_lab SIL fault cleared";
  return walking_zoo_core::CommandResult::success(status_text_);
}

std::string GaitLabSilModel::control_for_stop(walking_zoo_core::StopMode mode) const
{
  switch (mode) {
    case walking_zoo_core::StopMode::EMERGENCY:
      return CTRL_ESTOP;
    case walking_zoo_core::StopMode::QUICK:
      return CTRL_STOP_QUICK;
    case walking_zoo_core::StopMode::NORMAL:
    default:
      return CTRL_STOP_NORMAL;
  }
}

void GaitLabSilModel::ingest_sim_state(const WalkingState & state, double now_sec)
{
  sim_state_ = state;
  last_sim_state_sec_ = now_sec;
  have_sim_state_ = true;
}

bool GaitLabSilModel::sim_connected(double now_sec) const
{
  return have_sim_state_ &&
         (now_sec - last_sim_state_sec_) <= freshness_timeout_sec_;
}

WalkingState GaitLabSilModel::read_state(double now_sec) const
{
  WalkingState state;
  const bool connected = sim_connected(now_sec);
  if (connected) {
    // Trust the simulated robot's own report (balance, fall, support phase, …)
    // but stamp the adapter's authoritative lifecycle/estop bookkeeping on top.
    state = sim_state_;
  } else {
    // No fresh sim: synthesize from the adapter's lifecycle. The robot is, as
    // far as we can honestly say, standing (or idle) — not whatever the sim last
    // claimed before it went away.
    state.is_balanced = !fault_active_ && !estop_active_;
    state.is_fallen = false;
    state.support_phase = WalkingState::SUPPORT_DOUBLE;
    state.locomotion_state = active_ ?
      WalkingState::STATE_STANDING : WalkingState::STATE_IDLE;
  }

  state.lifecycle_state = active_ ?
    WalkingState::LIFECYCLE_ACTIVE : WalkingState::LIFECYCLE_INACTIVE;
  if (estop_active_) {
    state.lifecycle_state = WalkingState::LIFECYCLE_ESTOPPED;
    state.locomotion_state = WalkingState::STATE_ESTOPPED;
  }
  state.locomotion_mode = WalkingState::MODE_WALK;
  state.estop_active = estop_active_;
  state.adapter_connected = connected;
  state.active_adapter = PLUGIN_NAME;
  state.active_robot_model = profile_.robot_model;
  state.status_text = connected ? status_text_ :
    (configured_ ? "gait_lab SIL: waiting for the MuJoCo sim node" : status_text_);
  return state;
}

AdapterStatus GaitLabSilModel::get_status(double now_sec) const
{
  AdapterStatus status;
  const bool connected = sim_connected(now_sec);
  status.status = estop_active_ ?
    AdapterStatus::STATUS_ESTOPPED :
    (active_ ? AdapterStatus::STATUS_ACTIVE :
    (connected ? AdapterStatus::STATUS_CONNECTED : AdapterStatus::STATUS_DISCONNECTED));
  status.connected = connected;
  status.active = active_;
  // This is software-in-the-loop: it drives a simulated robot, never hardware.
  status.allow_motion = false;
  status.adapter_name = PLUGIN_NAME;
  status.robot_model = profile_.robot_model;
  status.hardware_id = "gait_lab_sil";
  status.status_text = connected ?
    status_text_ : "gait_lab SIL: MuJoCo sim node not connected";
  return status;
}

}  // namespace walking_zoo_gait_lab_sil
