#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "locomotion_ros2_gait_lab_sil/gait_lab_rl_policy.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace locomotion_ros2_gait_lab_sil
{

/**
 * Runs gait_lab RL residual inference inside ros2_control (B3 deep first rung).
 *
 * Subscribes to ~/observation and ~/feedforward from the split gait node, adds the
 * learned residual to the CPG feedforward, and writes leg position commands.
 */
class GaitLabSilRlResidualController : public controller_interface::ControllerInterface
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
  void on_observation(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void on_feedforward(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  std::vector<std::string> joint_names_;
  GaitLabRlPolicy policy_;
  double action_scale_{0.25};

  std::mutex sample_mutex_;
  std::vector<double> feedforward_;
  std::vector<double> residual_;
  std::vector<double> pending_observation_;
  std::atomic<bool> have_feedforward_{false};
  std::atomic<bool> have_residual_{false};
  std::atomic<bool> have_pending_observation_{false};
  std::vector<double> hold_command_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr observation_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr feedforward_sub_;
};

}  // namespace locomotion_ros2_gait_lab_sil
