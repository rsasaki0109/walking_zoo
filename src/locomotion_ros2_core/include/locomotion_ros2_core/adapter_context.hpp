#ifndef LOCOMOTION_ROS2_CORE__ADAPTER_CONTEXT_HPP_
#define LOCOMOTION_ROS2_CORE__ADAPTER_CONTEXT_HPP_

#include <memory>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "locomotion_ros2_core/robot_profile.hpp"

namespace locomotion_ros2_core
{

struct AdapterContext
{
  AdapterContext(
    const rclcpp::Logger & logger_in,
    rclcpp::Clock::SharedPtr clock_in)
  : logger(logger_in),
    clock(std::move(clock_in))
  {
  }

  rclcpp::Logger logger;
  rclcpp::Clock::SharedPtr clock;
  RobotProfile robot_profile;
  bool allow_motion{false};
  std::string robot_profile_path;
};

}  // namespace locomotion_ros2_core

#endif  // LOCOMOTION_ROS2_CORE__ADAPTER_CONTEXT_HPP_
