#ifndef WALKING_ZOO_NAV2__CMD_VEL_BRIDGE_HPP_
#define WALKING_ZOO_NAV2__CMD_VEL_BRIDGE_HPP_

#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "walking_zoo_msgs/msg/walking_state.hpp"
#include "walking_zoo_nav2/legged_velocity_shaper.hpp"

namespace walking_zoo_nav2
{

// Bridges a Nav2 `geometry_msgs/Twist` stream to the walking_zoo runtime's
// `geometry_msgs/TwistStamped` input. Beyond stamping, it can shape the command
// to a legged motion envelope (LeggedVelocityShaper) and gate it on the robot's
// published readiness so Nav2 velocities are never forwarded while the robot is
// e-stopped or not balanced.
class CmdVelBridge : public rclcpp::Node
{
public:
  explicit CmdVelBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void handle_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void handle_state(const walking_zoo_msgs::msg::WalkingState::SharedPtr msg);
  bool robot_ready() const;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Subscription<walking_zoo_msgs::msg::WalkingState>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  std::string frame_id_;

  bool legged_aware_{true};
  bool require_ready_{true};
  LeggedVelocityShaper shaper_;

  rclcpp::Time last_cmd_time_;
  bool has_last_cmd_time_{false};

  bool got_state_{false};
  bool robot_balanced_{false};
  bool robot_estopped_{false};
  bool suppressing_{false};
};

}  // namespace walking_zoo_nav2

#endif  // WALKING_ZOO_NAV2__CMD_VEL_BRIDGE_HPP_
