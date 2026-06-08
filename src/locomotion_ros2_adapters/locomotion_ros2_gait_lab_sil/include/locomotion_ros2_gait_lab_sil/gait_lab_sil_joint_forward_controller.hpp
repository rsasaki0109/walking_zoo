#pragma once

#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace locomotion_ros2_gait_lab_sil
{

/**
 * Forwards queued leg-joint position targets from the gait_lab policy node into
 * ros2_control command interfaces (B3 deep first rung).
 *
 * Subscribes to ~/commands (std_msgs/Float64MultiArray) with one position per
 * configured joint. Each update() consumes at most one queued sample so burst
 * policy output (substeps per physics tick) maps 1:1 to hardware write() cycles.
 */
class GaitLabSilJointForwardController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void on_commands(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  std::vector<std::string> joint_names_;
  std::vector<double> hold_command_;

  std::mutex queue_mutex_;
  std::deque<std::vector<double>> command_queue_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr commands_sub_;
};

}  // namespace locomotion_ros2_gait_lab_sil
