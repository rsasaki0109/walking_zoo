#include "locomotion_ros2_gait_lab_sil/gait_lab_sil_rl_residual_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace locomotion_ros2_gait_lab_sil
{

controller_interface::InterfaceConfiguration
GaitLabSilRlResidualController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(joint_names_.size());
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

controller_interface::InterfaceConfiguration
GaitLabSilRlResidualController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn GaitLabSilRlResidualController::on_init()
{
  try {
    joint_names_ = auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::string>("policy_path", "");
    auto_declare<double>("action_scale", 0.25);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_node()->get_logger(), "declare parameters failed: %s", ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GaitLabSilRlResidualController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  action_scale_ = get_node()->get_parameter("action_scale").as_double();
  auto policy_path = get_node()->get_parameter("policy_path").as_string();
  if (policy_path.empty()) {
    const char * gait_lab_root = std::getenv("LOCOMOTION_ROS2_GAIT_LAB_PATH");
    const char * controller_name = std::getenv("LOCOMOTION_ROS2_GAIT_LAB_CONTROLLER");
    const std::string selected = controller_name ? controller_name : "rl-residual";
    std::string policy_file = "rl_policy.npz";
    if (selected == "rl-steerable") {
      policy_file = "rl_policy_steer.npz";
    } else if (selected == "rl-steerable-footstep") {
      policy_file = "rl_policy_steer_fs.npz";
    }
    if (gait_lab_root == nullptr) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "'policy_path' is empty and LOCOMOTION_ROS2_GAIT_LAB_PATH is unset");
      return controller_interface::CallbackReturn::ERROR;
    }
    policy_path = (std::filesystem::path(gait_lab_root) / "gait_lab" / policy_file).string();
  }
  std::string error;
  if (!policy_.load(policy_path, &error)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "failed to load RL policy from %s: %s",
      policy_path.c_str(), error.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  observation_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/observation", rclcpp::SystemDefaultsQoS(),
    std::bind(&GaitLabSilRlResidualController::on_observation, this, std::placeholders::_1));
  feedforward_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/feedforward", rclcpp::SystemDefaultsQoS(),
    std::bind(&GaitLabSilRlResidualController::on_feedforward, this, std::placeholders::_1));

  feedforward_.assign(joint_names_.size(), 0.0);
  residual_.assign(joint_names_.size(), 0.0);
  hold_command_.assign(joint_names_.size(), 0.0);
  RCLCPP_INFO(
    get_node()->get_logger(),
    "loaded embedded RL policy (%zu-dim observation) from %s",
    policy_.observation_dim(), policy_path.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GaitLabSilRlResidualController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::lock_guard<std::mutex> lock(sample_mutex_);
  have_feedforward_.store(false);
  have_residual_.store(false);
  residual_.assign(joint_names_.size(), 0.0);
  for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
    hold_command_[i] = command_interfaces_[i].get_value();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GaitLabSilRlResidualController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  have_feedforward_.store(false);
  have_residual_.store(false);
  return controller_interface::CallbackReturn::SUCCESS;
}

void GaitLabSilRlResidualController::on_feedforward(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != joint_names_.size()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "feedforward size %zu != joints %zu", msg->data.size(), joint_names_.size());
    return;
  }
  std::lock_guard<std::mutex> lock(sample_mutex_);
  feedforward_.assign(msg->data.begin(), msg->data.end());
  have_feedforward_.store(true);
}

void GaitLabSilRlResidualController::on_observation(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != policy_.observation_dim()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "observation size %zu != policy %zu", msg->data.size(), policy_.observation_dim());
    return;
  }
  const auto action = policy_.infer(msg->data);
  if (action.size() != joint_names_.size()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "policy action size %zu != joints %zu", action.size(), joint_names_.size());
    return;
  }
  std::lock_guard<std::mutex> lock(sample_mutex_);
  residual_.resize(action.size());
  for (std::size_t i = 0; i < action.size(); ++i) {
    residual_[i] = action_scale_ * std::clamp(action[i], -1.0, 1.0);
  }
  have_residual_.store(true);
}

controller_interface::return_type GaitLabSilRlResidualController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!have_feedforward_.load()) {
    return controller_interface::return_type::OK;
  }

  std::vector<double> command;
  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    command = feedforward_;
    if (have_residual_.load() && residual_.size() == command.size()) {
      for (std::size_t i = 0; i < command.size(); ++i) {
        command[i] += residual_[i];
      }
    }
    hold_command_ = command;
  }

  if (command.size() != command_interfaces_.size()) {
    return controller_interface::return_type::ERROR;
  }
  for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
    if (!command_interfaces_[i].set_value(command[i])) {
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace locomotion_ros2_gait_lab_sil

PLUGINLIB_EXPORT_CLASS(
  locomotion_ros2_gait_lab_sil::GaitLabSilRlResidualController,
  controller_interface::ControllerInterface)
