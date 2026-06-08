#ifndef LOCOMOTION_ROS2_GAIT_LAB_SIL__GAIT_LAB_SIL_ADAPTER_HPP_
#define LOCOMOTION_ROS2_GAIT_LAB_SIL__GAIT_LAB_SIL_ADAPTER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "locomotion_ros2_core/walking_adapter.hpp"
#include "locomotion_ros2_gait_lab_sil/gait_lab_sil_model.hpp"

namespace locomotion_ros2_gait_lab_sil
{

// Software-in-the-loop adapter that drives a MuJoCo Unitree G1 through a
// gait_lab controller (default: the reinforcement-learned `rl-residual` policy
// that is the only gait_lab gait to walk a full horizon). The heavy physics +
// the learned policy live in a companion Python node (`gait_lab_sil_sim.py`), so
// this C++ plugin carries NO MuJoCo/Python build dependency — it is a thin ROS
// bridge: it forwards the runtime's safety-filtered commands to the sim and
// reports the simulated robot's state back into the runtime/safety pipeline.
//
// The bridge uses its own internal node, drained non-blockingly inside
// `read_state` via `spin_some` (no background executor thread). All command
// gating / state fusion lives in the ROS-free GaitLabSilModel.
class GaitLabSilAdapter : public locomotion_ros2_core::WalkingAdapter
{
public:
  GaitLabSilAdapter();
  ~GaitLabSilAdapter() override;

  locomotion_ros2_core::CallbackReturn configure(
    const locomotion_ros2_core::AdapterContext & context) override;
  locomotion_ros2_core::CallbackReturn activate() override;
  locomotion_ros2_core::CallbackReturn deactivate() override;
  locomotion_ros2_core::CallbackReturn cleanup() override;

  locomotion_ros2_core::RobotProfile get_robot_profile() const override;
  locomotion_ros2_msgs::msg::AdapterStatus get_status() const override;
  locomotion_ros2_msgs::msg::WalkingState read_state() override;

  locomotion_ros2_core::CommandResult command_velocity(
    const geometry_msgs::msg::TwistStamped & cmd) override;
  locomotion_ros2_core::CommandResult command_body_pose(
    const locomotion_ros2_msgs::msg::BodyPoseCommand & cmd) override;
  locomotion_ros2_core::CommandResult execute_footstep_plan(
    const locomotion_ros2_msgs::msg::FootstepPlan & plan) override;

  locomotion_ros2_core::CommandResult stop(locomotion_ros2_core::StopMode mode) override;
  locomotion_ros2_core::CommandResult emergency_stop() override;
  locomotion_ros2_core::CommandResult clear_fault() override;

private:
  double now_sec() const;
  void drain();                         // spin_some the bridge node
  void publish_control(const std::string & signal);

  GaitLabSilModel model_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_pub_;
  rclcpp::Subscription<locomotion_ros2_msgs::msg::WalkingState>::SharedPtr state_sub_;
};

}  // namespace locomotion_ros2_gait_lab_sil

#endif  // LOCOMOTION_ROS2_GAIT_LAB_SIL__GAIT_LAB_SIL_ADAPTER_HPP_
