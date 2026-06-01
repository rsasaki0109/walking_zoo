#include "walking_zoo_core/types.hpp"

#include "walking_zoo_core/robot_profile.hpp"

namespace walking_zoo_core
{

std::string to_string(CallbackReturn value)
{
  switch (value) {
    case CallbackReturn::SUCCESS:
      return "success";
    case CallbackReturn::FAILURE:
      return "failure";
    case CallbackReturn::ERROR:
      return "error";
  }
  return "unknown";
}

std::string to_string(StopMode value)
{
  switch (value) {
    case StopMode::NORMAL:
      return "normal";
    case StopMode::QUICK:
      return "quick";
    case StopMode::EMERGENCY:
      return "emergency";
  }
  return "unknown";
}

std::string to_string(CommandStatus value)
{
  switch (value) {
    case CommandStatus::ACCEPTED:
      return "accepted";
    case CommandStatus::REJECTED:
      return "rejected";
    case CommandStatus::LIMITED:
      return "limited";
    case CommandStatus::BLOCKED:
      return "blocked";
    case CommandStatus::ERROR:
      return "error";
  }
  return "unknown";
}

walking_zoo_msgs::msg::RobotProfile RobotProfile::to_msg() const
{
  walking_zoo_msgs::msg::RobotProfile msg;
  msg.robot_model = robot_model;
  msg.robot_family = robot_family;
  msg.adapter_plugin = adapter_plugin;
  msg.velocity_command = velocity_command;
  msg.body_pose_command = body_pose_command;
  msg.footstep_plan = footstep_plan;
  msg.whole_body_goal = whole_body_goal;
  msg.sit_stand = sit_stand;
  msg.estop = estop;
  msg.lateral_step = lateral_step;
  msg.turn_in_place = turn_in_place;
  msg.max_linear_x = static_cast<float>(max_linear_x);
  msg.max_linear_y = static_cast<float>(max_linear_y);
  msg.max_angular_z = static_cast<float>(max_angular_z);
  msg.max_body_roll = static_cast<float>(max_body_roll);
  msg.max_body_pitch = static_cast<float>(max_body_pitch);
  msg.command_timeout_sec = static_cast<float>(command_timeout_sec);
  msg.base_frame = base_frame;
  msg.odom_frame = odom_frame;
  msg.map_frame = map_frame;
  msg.real_robot_motion_allowed = real_robot_motion_allowed;
  msg.status_text = status_text;
  return msg;
}

}  // namespace walking_zoo_core
