#include "walking_zoo_unitree_sdk2/unitree_sdk2_adapter.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace walking_zoo_unitree_sdk2
{

UnitreeSdk2Adapter::UnitreeSdk2Adapter() = default;

walking_zoo_core::CallbackReturn UnitreeSdk2Adapter::configure(
  const walking_zoo_core::AdapterContext & context)
{
  profile_ = context.robot_profile;
  profile_.adapter_plugin = "walking_zoo_unitree_sdk2/UnitreeSdk2Adapter";
  allow_motion_ = context.allow_motion;
  configured_ = true;
#ifdef WALKING_ZOO_WITH_UNITREE_SDK2
  status_text_ = allow_motion_ ?
    "Unitree SDK2 adapter configured with motion enabled" :
    "Unitree SDK2 adapter configured with motion disabled";
#else
  status_text_ = "Unitree SDK2 support not compiled";
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
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::CallbackReturn UnitreeSdk2Adapter::deactivate()
{
  active_ = false;
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::CallbackReturn UnitreeSdk2Adapter::cleanup()
{
  configured_ = false;
  active_ = false;
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
  state.locomotion_state = estop_active_ ?
    walking_zoo_msgs::msg::WalkingState::STATE_ESTOPPED :
    walking_zoo_msgs::msg::WalkingState::STATE_IDLE;
  state.locomotion_mode = walking_zoo_msgs::msg::WalkingState::MODE_IDLE;
  state.support_phase = walking_zoo_msgs::msg::WalkingState::SUPPORT_UNKNOWN;
  state.is_balanced = false;
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
  (void)cmd;
#ifndef WALKING_ZOO_WITH_UNITREE_SDK2
  return walking_zoo_core::CommandResult::rejected("Unitree SDK2 support not compiled");
#else
  if (!allow_motion_) {
    return walking_zoo_core::CommandResult::blocked("allow_motion is false");
  }
  return walking_zoo_core::CommandResult::rejected("Unitree SDK2 command hook TODO");
#endif
}

walking_zoo_core::CommandResult UnitreeSdk2Adapter::command_body_pose(
  const walking_zoo_msgs::msg::BodyPoseCommand & cmd)
{
  (void)cmd;
  return walking_zoo_core::CommandResult::rejected("Unitree body pose hook TODO");
}

walking_zoo_core::CommandResult UnitreeSdk2Adapter::execute_footstep_plan(
  const walking_zoo_msgs::msg::FootstepPlan & plan)
{
  (void)plan;
  return walking_zoo_core::CommandResult::rejected("Unitree footstep hook TODO");
}

walking_zoo_core::CommandResult UnitreeSdk2Adapter::stop(walking_zoo_core::StopMode mode)
{
  (void)mode;
  return walking_zoo_core::CommandResult::success("Unitree stop hook accepted");
}

walking_zoo_core::CommandResult UnitreeSdk2Adapter::emergency_stop()
{
  estop_active_ = true;
  status_text_ = "Unitree emergency stop requested";
  return walking_zoo_core::CommandResult::success(status_text_);
}

walking_zoo_core::CommandResult UnitreeSdk2Adapter::clear_fault()
{
  if (estop_active_) {
    return walking_zoo_core::CommandResult::blocked("clear estop before clearing fault");
  }
  status_text_ = "Unitree fault clear hook accepted";
  return walking_zoo_core::CommandResult::success(status_text_);
}

}  // namespace walking_zoo_unitree_sdk2

PLUGINLIB_EXPORT_CLASS(
  walking_zoo_unitree_sdk2::UnitreeSdk2Adapter,
  walking_zoo_core::WalkingAdapter)
