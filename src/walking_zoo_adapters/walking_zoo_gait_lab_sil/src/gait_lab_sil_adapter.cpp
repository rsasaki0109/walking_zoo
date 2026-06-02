#include "walking_zoo_gait_lab_sil/gait_lab_sil_adapter.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace walking_zoo_gait_lab_sil
{

namespace
{
constexpr const char * kCommandTopic = "gait_lab_sil/command_velocity";
constexpr const char * kControlTopic = "gait_lab_sil/control";
constexpr const char * kStateTopic = "gait_lab_sil/robot_state";
}  // namespace

GaitLabSilAdapter::GaitLabSilAdapter()
: logger_(rclcpp::get_logger("gait_lab_sil_adapter"))
{
}

GaitLabSilAdapter::~GaitLabSilAdapter()
{
  cmd_pub_.reset();
  control_pub_.reset();
  state_sub_.reset();
  node_.reset();
}

double GaitLabSilAdapter::now_sec() const
{
  return clock_ ? clock_->now().seconds() : 0.0;
}

void GaitLabSilAdapter::drain()
{
  if (node_) {
    rclcpp::spin_some(node_);
  }
}

void GaitLabSilAdapter::publish_control(const std::string & signal)
{
  if (control_pub_) {
    std_msgs::msg::String msg;
    msg.data = signal;
    control_pub_->publish(msg);
  }
}

walking_zoo_core::CallbackReturn GaitLabSilAdapter::configure(
  const walking_zoo_core::AdapterContext & context)
{
  logger_ = context.logger;
  clock_ = context.clock;
  model_.configure(context.robot_profile);

  // The bridge owns a small private node. It is drained non-blockingly from
  // read_state() (spin_some), so no background executor thread is needed.
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(logger_, "gait_lab SIL: rclcpp is not initialised");
    return walking_zoo_core::CallbackReturn::ERROR;
  }
  node_ = std::make_shared<rclcpp::Node>("gait_lab_sil_bridge");
  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(kCommandTopic, 10);
  // Control (lifecycle) signals are latched: the sim node loads MuJoCo and joins
  // a second or two after the runtime autostarts, so it would otherwise miss the
  // initial "activate". transient_local delivers the last signal to late joiners.
  control_pub_ = node_->create_publisher<std_msgs::msg::String>(
    kControlTopic, rclcpp::QoS(1).transient_local());
  state_sub_ = node_->create_subscription<walking_zoo_msgs::msg::WalkingState>(
    kStateTopic, rclcpp::QoS(10),
    [this](walking_zoo_msgs::msg::WalkingState::SharedPtr msg) {
      model_.ingest_sim_state(*msg, now_sec());
    });
  RCLCPP_INFO(
    logger_,
    "gait_lab SIL bridge configured: commands -> %s, state <- %s "
    "(start the MuJoCo sim node gait_lab_sil_sim.py)",
    kCommandTopic, kStateTopic);
  return walking_zoo_core::CallbackReturn::SUCCESS;
}

walking_zoo_core::CallbackReturn GaitLabSilAdapter::activate()
{
  const auto cb = model_.activate();
  if (cb == walking_zoo_core::CallbackReturn::SUCCESS) {
    publish_control(GaitLabSilModel::CTRL_ACTIVATE);
  }
  return cb;
}

walking_zoo_core::CallbackReturn GaitLabSilAdapter::deactivate()
{
  const auto cb = model_.deactivate();
  publish_control(GaitLabSilModel::CTRL_DEACTIVATE);
  return cb;
}

walking_zoo_core::CallbackReturn GaitLabSilAdapter::cleanup()
{
  const auto cb = model_.cleanup();
  cmd_pub_.reset();
  control_pub_.reset();
  state_sub_.reset();
  node_.reset();
  return cb;
}

walking_zoo_core::RobotProfile GaitLabSilAdapter::get_robot_profile() const
{
  return model_.profile();
}

walking_zoo_msgs::msg::AdapterStatus GaitLabSilAdapter::get_status() const
{
  auto status = model_.get_status(now_sec());
  if (clock_) {
    status.header.stamp = clock_->now();
  }
  return status;
}

walking_zoo_msgs::msg::WalkingState GaitLabSilAdapter::read_state()
{
  drain();  // pull any fresh state the sim published
  auto state = model_.read_state(now_sec());
  if (clock_) {
    state.header.stamp = clock_->now();
  }
  return state;
}

walking_zoo_core::CommandResult GaitLabSilAdapter::command_velocity(
  const geometry_msgs::msg::TwistStamped & cmd)
{
  const auto result = model_.command_velocity_gate(cmd);
  if (result.accepted && cmd_pub_) {
    cmd_pub_->publish(cmd);  // forward the safety-filtered command to the sim
  }
  return result;
}

walking_zoo_core::CommandResult GaitLabSilAdapter::command_body_pose(
  const walking_zoo_msgs::msg::BodyPoseCommand & cmd)
{
  (void)cmd;
  return model_.body_pose_gate();
}

walking_zoo_core::CommandResult GaitLabSilAdapter::execute_footstep_plan(
  const walking_zoo_msgs::msg::FootstepPlan & plan)
{
  (void)plan;
  return model_.footstep_gate();
}

walking_zoo_core::CommandResult GaitLabSilAdapter::stop(walking_zoo_core::StopMode mode)
{
  const auto result = model_.stop_gate(mode);
  if (result.accepted) {
    publish_control(model_.control_for_stop(mode));
  }
  return result;
}

walking_zoo_core::CommandResult GaitLabSilAdapter::emergency_stop()
{
  const auto result = model_.emergency_stop_gate();
  publish_control(GaitLabSilModel::CTRL_ESTOP);
  return result;
}

walking_zoo_core::CommandResult GaitLabSilAdapter::clear_fault()
{
  const auto result = model_.clear_fault_gate();
  publish_control(GaitLabSilModel::CTRL_CLEAR_FAULT);
  return result;
}

}  // namespace walking_zoo_gait_lab_sil

PLUGINLIB_EXPORT_CLASS(
  walking_zoo_gait_lab_sil::GaitLabSilAdapter,
  walking_zoo_core::WalkingAdapter)
