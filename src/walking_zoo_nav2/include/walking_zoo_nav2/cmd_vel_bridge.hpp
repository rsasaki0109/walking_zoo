#ifndef WALKING_ZOO_NAV2__CMD_VEL_BRIDGE_HPP_
#define WALKING_ZOO_NAV2__CMD_VEL_BRIDGE_HPP_

#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace walking_zoo_nav2
{

class CmdVelBridge : public rclcpp::Node
{
public:
  explicit CmdVelBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void handle_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  std::string frame_id_;
};

}  // namespace walking_zoo_nav2

#endif  // WALKING_ZOO_NAV2__CMD_VEL_BRIDGE_HPP_
