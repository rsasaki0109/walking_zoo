#include "walking_zoo_mock_adapter/mock_walking_adapter.hpp"

#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace walking_zoo_mock_adapter
{

MockWalkingAdapter::MockWalkingAdapter() = default;

walking_zoo_core::CallbackReturn MockWalkingAdapter::configure(
  const walking_zoo_core::AdapterContext & context)
{
  profile_ = context.robot_profile;
  profile_.adapter_plugin = "walking_zoo_mock_adapter/MockWalkingAdapter";
  profile_.real_robot_motion_allowed = false;
  configured_ = true;
  active_ = false;
  estop_active_ = false;
  fault_active_ = false;
  locomotion_state_ = walking_zoo_msgs::msg::WalkingState::STATE_IDLE;
  status_text_ = "mock adapter configured";
  (void)context;
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::CallbackReturn MockWalkingAdapter::activate()
{
  if (!configured_) {
    status_text_ = "mock adapter not configured";
    return walking_zoo_core::CallbackReturn::FAILURE;
  }
  active_ = true;
  locomotion_state_ = walking_zoo_msgs::msg::WalkingState::STATE_STANDING;
  status_text_ = "mock adapter active";
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::CallbackReturn MockWalkingAdapter::deactivate()
{
  active_ = false;
  locomotion_state_ = walking_zoo_msgs::msg::WalkingState::STATE_IDLE;
  status_text_ = "mock adapter inactive";
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::CallbackReturn MockWalkingAdapter::cleanup()
{
  configured_ = false;
  active_ = false;
  locomotion_state_ = walking_zoo_msgs::msg::WalkingState::STATE_UNKNOWN;
  status_text_ = "mock adapter cleaned up";
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::RobotProfile MockWalkingAdapter::get_robot_profile() const
{
  return profile_;
}

walking_zoo_msgs::msg::AdapterStatus MockWalkingAdapter::get_status() const
{
  walking_zoo_msgs::msg::AdapterStatus status;
  status.status = estop_active_ ?
    walking_zoo_msgs::msg::AdapterStatus::STATUS_ESTOPPED :
    (active_ ? walking_zoo_msgs::msg::AdapterStatus::STATUS_ACTIVE :
    walking_zoo_msgs::msg::AdapterStatus::STATUS_CONNECTED);
  status.connected = configured_;
  status.active = active_;
  status.allow_motion = false;
  status.adapter_name = "walking_zoo_mock_adapter/MockWalkingAdapter";
  status.robot_model = profile_.robot_model;
  status.hardware_id = "mock";
  status.status_text = status_text_;
  return status;
}

walking_zoo_msgs::msg::WalkingState MockWalkingAdapter::read_state()
{
  walking_zoo_msgs::msg::WalkingState state;
  state.lifecycle_state = active_ ?
    walking_zoo_msgs::msg::WalkingState::LIFECYCLE_ACTIVE :
    walking_zoo_msgs::msg::WalkingState::LIFECYCLE_INACTIVE;
  state.locomotion_state = estop_active_ ?
    walking_zoo_msgs::msg::WalkingState::STATE_ESTOPPED :
    locomotion_state_;
  state.locomotion_mode = walking_zoo_msgs::msg::WalkingState::MODE_WALK;
  state.support_phase = walking_zoo_msgs::msg::WalkingState::SUPPORT_QUADRUPED;
  state.is_balanced = !fault_active_ && !estop_active_;
  state.is_fallen = false;
  state.estop_active = estop_active_;
  state.adapter_connected = configured_;
  state.active_adapter = "walking_zoo_mock_adapter/MockWalkingAdapter";
  state.active_robot_model = profile_.robot_model;
  state.status_text = status_text_;
  return state;
}

walking_zoo_core::CommandResult MockWalkingAdapter::command_velocity(
  const geometry_msgs::msg::TwistStamped & cmd)
{
  if (!active_) {
    return walking_zoo_core::CommandResult::rejected("mock adapter inactive");
  }
  if (estop_active_) {
    return walking_zoo_core::CommandResult::blocked("mock adapter estopped");
  }

  if (is_nonzero_velocity(cmd)) {
    locomotion_state_ = std::abs(cmd.twist.angular.z) > 1e-6 &&
      std::abs(cmd.twist.linear.x) < 1e-6 &&
      std::abs(cmd.twist.linear.y) < 1e-6 ?
      walking_zoo_msgs::msg::WalkingState::STATE_TURNING :
      walking_zoo_msgs::msg::WalkingState::STATE_WALKING;
    status_text_ = "mock walking command accepted";
  } else {
    locomotion_state_ = walking_zoo_msgs::msg::WalkingState::STATE_STANDING;
    status_text_ = "mock zero velocity accepted";
  }
  return walking_zoo_core::CommandResult::success(status_text_);
}

walking_zoo_core::CommandResult MockWalkingAdapter::command_body_pose(
  const walking_zoo_msgs::msg::BodyPoseCommand & cmd)
{
  (void)cmd;
  if (!active_) {
    return walking_zoo_core::CommandResult::rejected("mock adapter inactive");
  }
  locomotion_state_ = walking_zoo_msgs::msg::WalkingState::STATE_BODY_POSE_CONTROL;
  status_text_ = "mock body pose command accepted";
  return walking_zoo_core::CommandResult::success(status_text_);
}

walking_zoo_core::CommandResult MockWalkingAdapter::execute_footstep_plan(
  const walking_zoo_msgs::msg::FootstepPlan & plan)
{
  (void)plan;
  if (!active_) {
    return walking_zoo_core::CommandResult::rejected("mock adapter inactive");
  }
  locomotion_state_ = walking_zoo_msgs::msg::WalkingState::STATE_EXECUTING_FOOTSTEPS;
  status_text_ = "mock footstep plan accepted";
  return walking_zoo_core::CommandResult::success(status_text_);
}

walking_zoo_core::CommandResult MockWalkingAdapter::stop(walking_zoo_core::StopMode mode)
{
  (void)mode;
  if (!configured_) {
    return walking_zoo_core::CommandResult::rejected("mock adapter unconfigured");
  }
  locomotion_state_ = walking_zoo_msgs::msg::WalkingState::STATE_STANDING;
  status_text_ = "mock stop complete";
  return walking_zoo_core::CommandResult::success(status_text_);
}

walking_zoo_core::CommandResult MockWalkingAdapter::emergency_stop()
{
  estop_active_ = true;
  locomotion_state_ = walking_zoo_msgs::msg::WalkingState::STATE_ESTOPPED;
  status_text_ = "mock emergency stop active";
  return walking_zoo_core::CommandResult::success(status_text_);
}

walking_zoo_core::CommandResult MockWalkingAdapter::clear_fault()
{
  // clear_fault re-enables the driver: it clears the adapter fault and releases
  // the emergency-stop latch so the robot can stand again. The operator-estop
  // interlock (do not re-enable while the runtime estop is still engaged) lives
  // one layer up in the runtime, not in the adapter.
  estop_active_ = false;
  fault_active_ = false;
  locomotion_state_ = active_ ?
    walking_zoo_msgs::msg::WalkingState::STATE_STANDING :
    walking_zoo_msgs::msg::WalkingState::STATE_IDLE;
  status_text_ = "mock fault cleared";
  return walking_zoo_core::CommandResult::success(status_text_);
}

bool MockWalkingAdapter::is_nonzero_velocity(
  const geometry_msgs::msg::TwistStamped & cmd)
{
  return std::abs(cmd.twist.linear.x) > 1e-6 ||
         std::abs(cmd.twist.linear.y) > 1e-6 ||
         std::abs(cmd.twist.angular.z) > 1e-6;
}

}  // namespace walking_zoo_mock_adapter

PLUGINLIB_EXPORT_CLASS(
  walking_zoo_mock_adapter::MockWalkingAdapter,
  walking_zoo_core::WalkingAdapter)
