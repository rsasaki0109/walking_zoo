#ifndef LOCOMOTION_ROS2_CORE__ROBOT_PROFILE_HPP_
#define LOCOMOTION_ROS2_CORE__ROBOT_PROFILE_HPP_

#include <string>

#include "locomotion_ros2_msgs/msg/robot_profile.hpp"

namespace locomotion_ros2_core
{

struct RobotProfile
{
  std::string robot_model{"mock_legged_robot"};
  std::string robot_family{"mock"};
  std::string adapter_plugin{"locomotion_ros2_mock_adapter/MockWalkingAdapter"};

  bool velocity_command{true};
  bool body_pose_command{true};
  bool footstep_plan{false};
  bool whole_body_goal{false};
  bool sit_stand{true};
  bool estop{true};
  bool lateral_step{true};
  bool turn_in_place{true};

  double max_linear_x{0.3};
  double max_linear_y{0.2};
  double max_angular_z{0.5};
  double max_body_roll{0.2};
  double max_body_pitch{0.2};
  double command_timeout_sec{0.25};

  std::string base_frame{"base_link"};
  std::string odom_frame{"odom"};
  std::string map_frame{"map"};
  bool real_robot_motion_allowed{false};
  std::string status_text{"mock profile"};

  locomotion_ros2_msgs::msg::RobotProfile to_msg() const;
};

RobotProfile load_robot_profile_from_yaml(
  const std::string & path,
  const RobotProfile & defaults = RobotProfile{});

}  // namespace locomotion_ros2_core

#endif  // LOCOMOTION_ROS2_CORE__ROBOT_PROFILE_HPP_
