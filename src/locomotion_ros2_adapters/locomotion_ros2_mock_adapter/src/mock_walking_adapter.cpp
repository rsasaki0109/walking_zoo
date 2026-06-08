#include "locomotion_ros2_mock_adapter/mock_walking_adapter.hpp"

#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace locomotion_ros2_mock_adapter
{

MockWalkingAdapter::MockWalkingAdapter() = default;

locomotion_ros2_core::CallbackReturn MockWalkingAdapter::configure(
  const locomotion_ros2_core::AdapterContext & context)
{
  profile_ = context.robot_profile;
  profile_.adapter_plugin = "locomotion_ros2_mock_adapter/MockWalkingAdapter";
  profile_.real_robot_motion_allowed = false;
  configured_ = true;
  active_ = false;
  estop_active_ = false;
  fault_active_ = false;
  locomotion_state_ = locomotion_ros2_msgs::msg::WalkingState::STATE_IDLE;
  status_text_ = "mock adapter configured";
  (void)context;
  return locomotion_ros2_core::CallbackReturn::SUCCESS;
}

locomotion_ros2_core::CallbackReturn MockWalkingAdapter::activate()
{
  if (!configured_) {
    status_text_ = "mock adapter not configured";
    return locomotion_ros2_core::CallbackReturn::FAILURE;
  }
  active_ = true;
  locomotion_state_ = locomotion_ros2_msgs::msg::WalkingState::STATE_STANDING;
  status_text_ = "mock adapter active";
  return locomotion_ros2_core::CallbackReturn::SUCCESS;
}

locomotion_ros2_core::CallbackReturn MockWalkingAdapter::deactivate()
{
  active_ = false;
  locomotion_state_ = locomotion_ros2_msgs::msg::WalkingState::STATE_IDLE;
  status_text_ = "mock adapter inactive";
  return locomotion_ros2_core::CallbackReturn::SUCCESS;
}

locomotion_ros2_core::CallbackReturn MockWalkingAdapter::cleanup()
{
  configured_ = false;
  active_ = false;
  locomotion_state_ = locomotion_ros2_msgs::msg::WalkingState::STATE_UNKNOWN;
  status_text_ = "mock adapter cleaned up";
  return locomotion_ros2_core::CallbackReturn::SUCCESS;
}

locomotion_ros2_core::RobotProfile MockWalkingAdapter::get_robot_profile() const
{
  return profile_;
}

locomotion_ros2_msgs::msg::AdapterStatus MockWalkingAdapter::get_status() const
{
  locomotion_ros2_msgs::msg::AdapterStatus status;
  status.status = estop_active_ ?
    locomotion_ros2_msgs::msg::AdapterStatus::STATUS_ESTOPPED :
    (active_ ? locomotion_ros2_msgs::msg::AdapterStatus::STATUS_ACTIVE :
    locomotion_ros2_msgs::msg::AdapterStatus::STATUS_CONNECTED);
  status.connected = configured_;
  status.active = active_;
  status.allow_motion = false;
  status.adapter_name = "locomotion_ros2_mock_adapter/MockWalkingAdapter";
  status.robot_model = profile_.robot_model;
  status.hardware_id = "mock";
  status.status_text = status_text_;
  return status;
}

locomotion_ros2_msgs::msg::WalkingState MockWalkingAdapter::read_state()
{
  locomotion_ros2_msgs::msg::WalkingState state;
  state.lifecycle_state = active_ ?
    locomotion_ros2_msgs::msg::WalkingState::LIFECYCLE_ACTIVE :
    locomotion_ros2_msgs::msg::WalkingState::LIFECYCLE_INACTIVE;
  state.locomotion_state = estop_active_ ?
    locomotion_ros2_msgs::msg::WalkingState::STATE_ESTOPPED :
    locomotion_state_;
  state.locomotion_mode = locomotion_ros2_msgs::msg::WalkingState::MODE_WALK;
  state.support_phase = locomotion_ros2_msgs::msg::WalkingState::SUPPORT_QUADRUPED;
  state.is_balanced = !fault_active_ && !estop_active_;
  state.is_fallen = false;
  state.estop_active = estop_active_;
  state.adapter_connected = configured_;
  state.active_adapter = "locomotion_ros2_mock_adapter/MockWalkingAdapter";
  state.active_robot_model = profile_.robot_model;
  state.status_text = status_text_;
  return state;
}

locomotion_ros2_core::CommandResult MockWalkingAdapter::command_velocity(
  const geometry_msgs::msg::TwistStamped & cmd)
{
  if (!active_) {
    return locomotion_ros2_core::CommandResult::rejected("mock adapter inactive");
  }
  if (estop_active_) {
    return locomotion_ros2_core::CommandResult::blocked("mock adapter estopped");
  }

  if (is_nonzero_velocity(cmd)) {
    locomotion_state_ = std::abs(cmd.twist.angular.z) > 1e-6 &&
      std::abs(cmd.twist.linear.x) < 1e-6 &&
      std::abs(cmd.twist.linear.y) < 1e-6 ?
      locomotion_ros2_msgs::msg::WalkingState::STATE_TURNING :
      locomotion_ros2_msgs::msg::WalkingState::STATE_WALKING;
    status_text_ = "mock walking command accepted";
  } else {
    locomotion_state_ = locomotion_ros2_msgs::msg::WalkingState::STATE_STANDING;
    status_text_ = "mock zero velocity accepted";
  }
  return locomotion_ros2_core::CommandResult::success(status_text_);
}

locomotion_ros2_core::CommandResult MockWalkingAdapter::command_body_pose(
  const locomotion_ros2_msgs::msg::BodyPoseCommand & cmd)
{
  (void)cmd;
  if (!active_) {
    return locomotion_ros2_core::CommandResult::rejected("mock adapter inactive");
  }
  locomotion_state_ = locomotion_ros2_msgs::msg::WalkingState::STATE_BODY_POSE_CONTROL;
  status_text_ = "mock body pose command accepted";
  return locomotion_ros2_core::CommandResult::success(status_text_);
}

locomotion_ros2_core::CommandResult MockWalkingAdapter::execute_footstep_plan(
  const locomotion_ros2_msgs::msg::FootstepPlan & plan)
{
  (void)plan;
  if (!active_) {
    return locomotion_ros2_core::CommandResult::rejected("mock adapter inactive");
  }
  locomotion_state_ = locomotion_ros2_msgs::msg::WalkingState::STATE_EXECUTING_FOOTSTEPS;
  status_text_ = "mock footstep plan accepted";
  return locomotion_ros2_core::CommandResult::success(status_text_);
}

locomotion_ros2_core::CommandResult MockWalkingAdapter::stop(locomotion_ros2_core::StopMode mode)
{
  (void)mode;
  if (!configured_) {
    return locomotion_ros2_core::CommandResult::rejected("mock adapter unconfigured");
  }
  locomotion_state_ = locomotion_ros2_msgs::msg::WalkingState::STATE_STANDING;
  status_text_ = "mock stop complete";
  return locomotion_ros2_core::CommandResult::success(status_text_);
}

locomotion_ros2_core::CommandResult MockWalkingAdapter::emergency_stop()
{
  estop_active_ = true;
  locomotion_state_ = locomotion_ros2_msgs::msg::WalkingState::STATE_ESTOPPED;
  status_text_ = "mock emergency stop active";
  return locomotion_ros2_core::CommandResult::success(status_text_);
}

locomotion_ros2_core::CommandResult MockWalkingAdapter::clear_fault()
{
  // clear_fault re-enables the driver: it clears the adapter fault and releases
  // the emergency-stop latch so the robot can stand again. The operator-estop
  // interlock (do not re-enable while the runtime estop is still engaged) lives
  // one layer up in the runtime, not in the adapter.
  estop_active_ = false;
  fault_active_ = false;
  locomotion_state_ = active_ ?
    locomotion_ros2_msgs::msg::WalkingState::STATE_STANDING :
    locomotion_ros2_msgs::msg::WalkingState::STATE_IDLE;
  status_text_ = "mock fault cleared";
  return locomotion_ros2_core::CommandResult::success(status_text_);
}

bool MockWalkingAdapter::is_nonzero_velocity(
  const geometry_msgs::msg::TwistStamped & cmd)
{
  return std::abs(cmd.twist.linear.x) > 1e-6 ||
         std::abs(cmd.twist.linear.y) > 1e-6 ||
         std::abs(cmd.twist.angular.z) > 1e-6;
}

}  // namespace locomotion_ros2_mock_adapter

PLUGINLIB_EXPORT_CLASS(
  locomotion_ros2_mock_adapter::MockWalkingAdapter,
  locomotion_ros2_core::WalkingAdapter)
