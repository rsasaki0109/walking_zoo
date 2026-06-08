#include "locomotion_ros2_gait_lab_sil/gait_lab_sil_joint_forward_controller.hpp"

#include <algorithm>
#include <optional>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace locomotion_ros2_gait_lab_sil
{

controller_interface::InterfaceConfiguration
GaitLabSilJointForwardController::command_interface_configuration() const
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
GaitLabSilJointForwardController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn GaitLabSilJointForwardController::on_init()
{
  try {
    joint_names_ = auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_node()->get_logger(), "declare joints failed: %s", ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  hold_command_.assign(joint_names_.size(), 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GaitLabSilJointForwardController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  commands_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    std::bind(&GaitLabSilJointForwardController::on_commands, this, std::placeholders::_1));
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GaitLabSilJointForwardController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  command_queue_.clear();
  for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
    hold_command_[i] = command_interfaces_[i].get_value();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GaitLabSilJointForwardController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  command_queue_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

void GaitLabSilJointForwardController::on_commands(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != joint_names_.size()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "commands size %zu != joints %zu", msg->data.size(), joint_names_.size());
    return;
  }

  std::lock_guard<std::mutex> lock(queue_mutex_);
  command_queue_.emplace_back(msg->data.begin(), msg->data.end());
  constexpr std::size_t kMaxQueue = 32;
  while (command_queue_.size() > kMaxQueue) {
    command_queue_.pop_front();
  }
}

controller_interface::return_type GaitLabSilJointForwardController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::optional<std::vector<double>> next;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!command_queue_.empty()) {
      next = command_queue_.front();
      command_queue_.pop_front();
      hold_command_ = *next;
    }
  }

  if (!next) {
    return controller_interface::return_type::OK;
  }

  if (next->size() != command_interfaces_.size()) {
    return controller_interface::return_type::ERROR;
  }

  for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
    if (!command_interfaces_[i].set_value((*next)[i])) {
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace locomotion_ros2_gait_lab_sil

PLUGINLIB_EXPORT_CLASS(
  locomotion_ros2_gait_lab_sil::GaitLabSilJointForwardController,
  controller_interface::ControllerInterface)
