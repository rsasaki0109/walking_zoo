#ifndef LOCOMOTION_ROS2_RUNTIME__WALKING_RUNTIME_MANAGER_HPP_
#define LOCOMOTION_ROS2_RUNTIME__WALKING_RUNTIME_MANAGER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "locomotion_ros2_core/robot_profile.hpp"
#include "locomotion_ros2_core/walking_adapter.hpp"
#include "locomotion_ros2_msgs/action/execute_body_pose.hpp"
#include "locomotion_ros2_msgs/action/execute_footstep_plan.hpp"
#include "locomotion_ros2_msgs/action/execute_velocity.hpp"
#include "locomotion_ros2_msgs/msg/adapter_status.hpp"
#include "locomotion_ros2_msgs/msg/safety_state.hpp"
#include "locomotion_ros2_msgs/msg/walking_state.hpp"
#include "locomotion_ros2_msgs/srv/clear_fault.hpp"
#include "locomotion_ros2_msgs/srv/emergency_stop.hpp"
#include "locomotion_ros2_msgs/srv/set_locomotion_mode.hpp"
#include "locomotion_ros2_runtime/adapter_loader.hpp"
#include "locomotion_ros2_runtime/command_arbiter.hpp"
#include "locomotion_ros2_runtime/mode_manager.hpp"
#include "locomotion_ros2_runtime/step_feasibility_checker.hpp"
#include "locomotion_ros2_safety/safety_pipeline.hpp"

namespace locomotion_ros2_runtime
{

class WalkingRuntimeManager : public rclcpp_lifecycle::LifecycleNode
{
public:
  using ExecuteVelocity = locomotion_ros2_msgs::action::ExecuteVelocity;
  using GoalHandleExecuteVelocity = rclcpp_action::ServerGoalHandle<ExecuteVelocity>;
  using ExecuteFootstepPlan = locomotion_ros2_msgs::action::ExecuteFootstepPlan;
  using GoalHandleExecuteFootstepPlan = rclcpp_action::ServerGoalHandle<ExecuteFootstepPlan>;
  using ExecuteBodyPose = locomotion_ros2_msgs::action::ExecuteBodyPose;
  using GoalHandleExecuteBodyPose = rclcpp_action::ServerGoalHandle<ExecuteBodyPose>;
  using LifecycleCallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit WalkingRuntimeManager(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

private:
  void declare_parameters();
  locomotion_ros2_core::RobotProfile profile_from_parameters();
  bool is_active() const;

  void handle_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void publish_state();

  void handle_estop(
    const std::shared_ptr<locomotion_ros2_msgs::srv::EmergencyStop::Request> request,
    std::shared_ptr<locomotion_ros2_msgs::srv::EmergencyStop::Response> response);
  void handle_clear_fault(
    const std::shared_ptr<locomotion_ros2_msgs::srv::ClearFault::Request> request,
    std::shared_ptr<locomotion_ros2_msgs::srv::ClearFault::Response> response);
  void handle_set_locomotion_mode(
    const std::shared_ptr<locomotion_ros2_msgs::srv::SetLocomotionMode::Request> request,
    std::shared_ptr<locomotion_ros2_msgs::srv::SetLocomotionMode::Response> response);

  rclcpp_action::GoalResponse handle_velocity_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteVelocity::Goal> goal);
  rclcpp_action::CancelResponse handle_velocity_cancel(
    const std::shared_ptr<GoalHandleExecuteVelocity> goal_handle);
  void handle_velocity_accepted(
    const std::shared_ptr<GoalHandleExecuteVelocity> goal_handle);
  void execute_velocity_goal(
    const std::shared_ptr<GoalHandleExecuteVelocity> goal_handle);

  rclcpp_action::GoalResponse handle_footstep_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteFootstepPlan::Goal> goal);
  rclcpp_action::CancelResponse handle_footstep_cancel(
    const std::shared_ptr<GoalHandleExecuteFootstepPlan> goal_handle);
  void handle_footstep_accepted(
    const std::shared_ptr<GoalHandleExecuteFootstepPlan> goal_handle);
  void execute_footstep_goal(
    const std::shared_ptr<GoalHandleExecuteFootstepPlan> goal_handle);

  rclcpp_action::GoalResponse handle_body_pose_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteBodyPose::Goal> goal);
  rclcpp_action::CancelResponse handle_body_pose_cancel(
    const std::shared_ptr<GoalHandleExecuteBodyPose> goal_handle);
  void handle_body_pose_accepted(
    const std::shared_ptr<GoalHandleExecuteBodyPose> goal_handle);
  void execute_body_pose_goal(
    const std::shared_ptr<GoalHandleExecuteBodyPose> goal_handle);

  std::unique_ptr<AdapterLoader> adapter_loader_;
  std::shared_ptr<locomotion_ros2_core::WalkingAdapter> adapter_;
  locomotion_ros2_core::RobotProfile robot_profile_;
  locomotion_ros2_safety::SafetyPipeline safety_pipeline_;
  CommandArbiter command_arbiter_;
  ModeManager mode_manager_;
  StepFeasibilityChecker feasibility_checker_;

  rclcpp_lifecycle::LifecyclePublisher<locomotion_ros2_msgs::msg::WalkingState>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<locomotion_ros2_msgs::msg::AdapterStatus>::SharedPtr adapter_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<locomotion_ros2_msgs::msg::SafetyState>::SharedPtr safety_state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Service<locomotion_ros2_msgs::srv::EmergencyStop>::SharedPtr estop_srv_;
  rclcpp::Service<locomotion_ros2_msgs::srv::ClearFault>::SharedPtr clear_fault_srv_;
  rclcpp::Service<locomotion_ros2_msgs::srv::SetLocomotionMode>::SharedPtr set_mode_srv_;
  rclcpp_action::Server<ExecuteVelocity>::SharedPtr execute_velocity_server_;
  rclcpp_action::Server<ExecuteFootstepPlan>::SharedPtr execute_footstep_server_;
  rclcpp_action::Server<ExecuteBodyPose>::SharedPtr execute_body_pose_server_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  locomotion_ros2_msgs::msg::WalkingState last_state_;
  std::string robot_profile_path_;
  std::string adapter_plugin_;
  bool allow_motion_{false};
};

}  // namespace locomotion_ros2_runtime

#endif  // LOCOMOTION_ROS2_RUNTIME__WALKING_RUNTIME_MANAGER_HPP_
