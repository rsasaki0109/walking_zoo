#include "walking_zoo_nav2/cmd_vel_bridge.hpp"

namespace walking_zoo_nav2
{

CmdVelBridge::CmdVelBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("walking_zoo_cmd_vel_bridge", options)
{
  const auto input_topic = declare_parameter<std::string>("input_topic", "/cmd_vel");
  const auto output_topic = declare_parameter<std::string>("output_topic", "/walking_zoo/cmd_vel");
  frame_id_ = declare_parameter<std::string>("frame_id", "base_link");

  pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    output_topic,
    rclcpp::SystemDefaultsQoS());
  sub_ = create_subscription<geometry_msgs::msg::Twist>(
    input_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&CmdVelBridge::handle_cmd_vel, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Bridging %s geometry_msgs/Twist to %s geometry_msgs/TwistStamped",
    input_topic.c_str(),
    output_topic.c_str());
}

void CmdVelBridge::handle_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  geometry_msgs::msg::TwistStamped stamped;
  stamped.header.stamp = now();
  stamped.header.frame_id = frame_id_;
  stamped.twist = *msg;
  pub_->publish(stamped);
}

}  // namespace walking_zoo_nav2
