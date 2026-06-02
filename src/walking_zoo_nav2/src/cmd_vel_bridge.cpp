#include "walking_zoo_nav2/cmd_vel_bridge.hpp"

namespace walking_zoo_nav2
{

CmdVelBridge::CmdVelBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("walking_zoo_cmd_vel_bridge", options),
  last_cmd_time_(0, 0, RCL_ROS_TIME)
{
  const auto input_topic = declare_parameter<std::string>("input_topic", "/cmd_vel");
  const auto output_topic = declare_parameter<std::string>("output_topic", "/walking_zoo/cmd_vel");
  const auto state_topic = declare_parameter<std::string>("state_topic", "/walking_zoo/state");
  frame_id_ = declare_parameter<std::string>("frame_id", "base_link");

  legged_aware_ = declare_parameter<bool>("legged_aware", true);
  require_ready_ = declare_parameter<bool>("require_ready", true);
  input_stamped_ = declare_parameter<bool>("input_stamped", false);

  LeggedMotionLimits limits;
  limits.max_forward = declare_parameter<double>("legged.max_forward", limits.max_forward);
  limits.max_backward = declare_parameter<double>("legged.max_backward", limits.max_backward);
  limits.max_lateral = declare_parameter<double>("legged.max_lateral", limits.max_lateral);
  limits.max_yaw_rate = declare_parameter<double>("legged.max_yaw_rate", limits.max_yaw_rate);
  limits.max_linear_accel =
    declare_parameter<double>("legged.max_linear_accel", limits.max_linear_accel);
  limits.max_yaw_accel = declare_parameter<double>("legged.max_yaw_accel", limits.max_yaw_accel);
  limits.lateral_deadband =
    declare_parameter<double>("legged.lateral_deadband", limits.lateral_deadband);
  limits.turn_speed_coupling =
    declare_parameter<double>("legged.turn_speed_coupling", limits.turn_speed_coupling);
  shaper_.set_limits(limits);

  pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    output_topic,
    rclcpp::SystemDefaultsQoS());
  if (input_stamped_) {
    sub_stamped_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      input_topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&CmdVelBridge::handle_cmd_vel_stamped, this, std::placeholders::_1));
  } else {
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&CmdVelBridge::handle_cmd_vel, this, std::placeholders::_1));
  }

  if (require_ready_) {
    state_sub_ = create_subscription<walking_zoo_msgs::msg::WalkingState>(
      state_topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&CmdVelBridge::handle_state, this, std::placeholders::_1));
  }

  RCLCPP_INFO(
    get_logger(),
    "Bridging %s Twist to %s TwistStamped (legged_aware=%s, require_ready=%s)",
    input_topic.c_str(),
    output_topic.c_str(),
    legged_aware_ ? "true" : "false",
    require_ready_ ? "true" : "false");
}

void CmdVelBridge::handle_state(const walking_zoo_msgs::msg::WalkingState::SharedPtr msg)
{
  got_state_ = true;
  robot_balanced_ = msg->is_balanced;
  robot_estopped_ = msg->estop_active;
}

bool CmdVelBridge::robot_ready() const
{
  if (!require_ready_) {
    return true;
  }
  // Until the first state arrives, do not block startup; once we hear from the
  // runtime, only forward motion while balanced and not e-stopped.
  if (!got_state_) {
    return true;
  }
  return robot_balanced_ && !robot_estopped_;
}

void CmdVelBridge::handle_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  process_twist(*msg);
}

void CmdVelBridge::handle_cmd_vel_stamped(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  process_twist(msg->twist);
}

void CmdVelBridge::process_twist(const geometry_msgs::msg::Twist & msg)
{
  geometry_msgs::msg::TwistStamped stamped;
  stamped.header.stamp = now();
  stamped.header.frame_id = frame_id_;

  if (!robot_ready()) {
    // Hold: publish a zero command and forget shaper history so re-acceleration
    // is smooth once the robot is ready again.
    shaper_.reset();
    has_last_cmd_time_ = false;
    pub_->publish(stamped);
    if (!suppressing_) {
      RCLCPP_WARN(
        get_logger(), "Robot not ready (estop=%s balanced=%s); holding Nav2 velocity",
        robot_estopped_ ? "true" : "false", robot_balanced_ ? "true" : "false");
      suppressing_ = true;
    }
    return;
  }
  if (suppressing_) {
    RCLCPP_INFO(get_logger(), "Robot ready; resuming Nav2 velocity bridge");
    suppressing_ = false;
  }

  if (!legged_aware_) {
    stamped.twist = msg;
    pub_->publish(stamped);
    return;
  }

  const auto stamp = now();
  double dt = 0.0;
  if (has_last_cmd_time_) {
    dt = (stamp - last_cmd_time_).seconds();
  }
  last_cmd_time_ = stamp;
  has_last_cmd_time_ = true;

  const auto shaped = shaper_.shape(
    msg.linear.x, msg.linear.y, msg.angular.z, dt);
  stamped.twist.linear.x = shaped.vx;
  stamped.twist.linear.y = shaped.vy;
  stamped.twist.angular.z = shaped.vyaw;
  pub_->publish(stamped);
}

}  // namespace walking_zoo_nav2
