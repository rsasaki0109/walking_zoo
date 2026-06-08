#include "locomotion_ros2_gait_lab_sil/gait_lab_sil_topic_system.hpp"

#include <algorithm>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace locomotion_ros2_gait_lab_sil
{

hardware_interface::CallbackReturn GaitLabSilTopicSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  for (const auto & joint : info.joints) {
    joint_names_.push_back(joint.name);
  }
  const auto n = joint_names_.size();
  hw_positions_.assign(n, 0.0);
  hw_velocities_.assign(n, 0.0);
  hw_commands_.assign(n, 0.0);

  for (const auto & param : info.hardware_parameters) {
    if (param.first == "joint_states_topic") {
      joint_states_topic_ = param.second;
    } else if (param.first == "joint_commands_topic") {
      joint_commands_topic_ = param.second;
    } else if (param.first == "relay_commands") {
      relay_commands_ = (param.second == "true" || param.second == "1");
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GaitLabSilTopicSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  node_ = rclcpp::Node::make_shared("gait_lab_sil_topic_system");
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, rclcpp::SensorDataQoS(),
    std::bind(&GaitLabSilTopicSystem::on_joint_states, this, std::placeholders::_1));
  joint_command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    joint_commands_topic_, rclcpp::SystemDefaultsQoS());
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
GaitLabSilTopicSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    states.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    states.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return states;
}

std::vector<hardware_interface::CommandInterface>
GaitLabSilTopicSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> commands;
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    commands.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return commands;
}

hardware_interface::CallbackReturn GaitLabSilTopicSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (std::size_t i = 0; i < hw_commands_.size(); ++i) {
    hw_commands_[i] = hw_positions_[i];
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GaitLabSilTopicSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

void GaitLabSilTopicSystem::on_joint_states(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const auto & name = joint_names_[i];
    auto it = std::find(msg->name.begin(), msg->name.end(), name);
    if (it == msg->name.end()) {
      continue;
    }
    const auto idx = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
    if (idx < msg->position.size()) {
      hw_positions_[i] = msg->position[idx];
    }
    if (idx < msg->velocity.size()) {
      hw_velocities_[i] = msg->velocity[idx];
    }
  }
  state_received_ = true;
}

hardware_interface::return_type GaitLabSilTopicSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (node_) {
    rclcpp::spin_some(node_);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GaitLabSilTopicSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Direct path: policy publishes joint_commands; hardware only reads state.
  // Forward path: GaitLabSilJointForwardController writes hw_commands and
  // relay_commands=true relays them to the sim topic each update.
  if (!relay_commands_ || !joint_command_pub_ || joint_names_.empty()) {
    return hardware_interface::return_type::OK;
  }

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->now();
  msg.name = joint_names_;
  msg.position = hw_commands_;
  joint_command_pub_->publish(msg);
  return hardware_interface::return_type::OK;
}

}  // namespace locomotion_ros2_gait_lab_sil

PLUGINLIB_EXPORT_CLASS(
  locomotion_ros2_gait_lab_sil::GaitLabSilTopicSystem,
  hardware_interface::SystemInterface)
