#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "walking_zoo_nav2/cmd_vel_bridge.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<walking_zoo_nav2::CmdVelBridge>());
  rclcpp::shutdown();
  return 0;
}
