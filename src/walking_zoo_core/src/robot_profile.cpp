#include "walking_zoo_core/robot_profile.hpp"

#include <stdexcept>
#include <string>

#include "yaml-cpp/yaml.h"

namespace walking_zoo_core
{

namespace
{

template<typename T>
T scalar_or(const YAML::Node & node, const std::string & key, const T & fallback)
{
  if (!node || !node[key]) {
    return fallback;
  }
  try {
    return node[key].as<T>();
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument("invalid robot profile key '" + key + "': " + error.what());
  }
}

bool bool_or_planned(const YAML::Node & node, const std::string & key, bool fallback)
{
  if (!node || !node[key]) {
    return fallback;
  }
  try {
    return node[key].as<bool>();
  } catch (const YAML::Exception &) {
    const auto value = node[key].as<std::string>();
    if (value == "planned" || value == "todo" || value == "false") {
      return false;
    }
    if (value == "true") {
      return true;
    }
    throw std::invalid_argument("invalid boolean capability '" + key + "': " + value);
  }
}

}  // namespace

RobotProfile load_robot_profile_from_yaml(
  const std::string & path,
  const RobotProfile & defaults)
{
  if (path.empty() || path == "mock") {
    return defaults;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("failed to load robot profile '" + path + "': " + error.what());
  }

  RobotProfile profile = defaults;
  profile.robot_model = scalar_or(root, "robot_model", profile.robot_model);
  profile.robot_family = scalar_or(root, "robot_family", profile.robot_family);
  profile.adapter_plugin = scalar_or(root, "adapter_plugin", profile.adapter_plugin);

  const auto capabilities = root["capabilities"];
  profile.velocity_command = bool_or_planned(
    capabilities, "velocity_command", profile.velocity_command);
  profile.body_pose_command = bool_or_planned(
    capabilities, "body_pose_command", profile.body_pose_command);
  profile.footstep_plan = bool_or_planned(
    capabilities, "footstep_plan", profile.footstep_plan);
  profile.whole_body_goal = bool_or_planned(
    capabilities, "whole_body_goal", profile.whole_body_goal);
  profile.sit_stand = bool_or_planned(capabilities, "sit_stand", profile.sit_stand);
  profile.estop = bool_or_planned(capabilities, "estop", profile.estop);
  profile.lateral_step = bool_or_planned(capabilities, "lateral_step", profile.lateral_step);
  profile.turn_in_place = bool_or_planned(
    capabilities, "turn_in_place", profile.turn_in_place);

  const auto limits = root["limits"];
  profile.max_linear_x = scalar_or(limits, "max_linear_x", profile.max_linear_x);
  profile.max_linear_y = scalar_or(limits, "max_linear_y", profile.max_linear_y);
  profile.max_angular_z = scalar_or(limits, "max_angular_z", profile.max_angular_z);
  profile.max_body_roll = scalar_or(limits, "max_body_roll", profile.max_body_roll);
  profile.max_body_pitch = scalar_or(limits, "max_body_pitch", profile.max_body_pitch);
  profile.command_timeout_sec = scalar_or(
    limits, "command_timeout_sec", profile.command_timeout_sec);

  const auto frames = root["frames"];
  profile.base_frame = scalar_or(frames, "base_frame", profile.base_frame);
  profile.odom_frame = scalar_or(frames, "odom_frame", profile.odom_frame);
  profile.map_frame = scalar_or(frames, "map_frame", profile.map_frame);

  const auto safety = root["safety"];
  profile.real_robot_motion_allowed = bool_or_planned(
    safety, "allow_motion_default", profile.real_robot_motion_allowed);
  profile.status_text = "loaded robot profile from " + path;
  return profile;
}

}  // namespace walking_zoo_core
