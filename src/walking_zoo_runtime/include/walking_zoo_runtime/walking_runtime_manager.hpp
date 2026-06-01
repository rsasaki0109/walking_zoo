#ifndef WALKING_ZOO_RUNTIME__WALKING_RUNTIME_MANAGER_HPP_
#define WALKING_ZOO_RUNTIME__WALKING_RUNTIME_MANAGER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "walking_zoo_core/robot_profile.hpp"
#include "walking_zoo_core/walking_adapter.hpp"
#include "walking_zoo_msgs/action/execute_velocity.hpp"
#include "walking_zoo_msgs/msg/adapter_status.hpp"
#include "walking_zoo_msgs/msg/safety_state.hpp"
#include "walking_zoo_msgs/msg/walking_state.hpp"
#include "walking_zoo_msgs/srv/clear_fault.hpp"
#include "walking_zoo_msgs/srv/emergency_stop.hpp"
#include "walking_zoo_msgs/srv/set_locomotion_mode.hpp"
#include "walking_zoo_runtime/adapter_loader.hpp"
#include "walking_zoo_runtime/command_arbiter.hpp"
#include "walking_zoo_runtime/mode_manager.hpp"
#include "walking_zoo_safety/safety_pipeline.hpp"

namespace walking_zoo_runtime
{

class WalkingRuntimeManager : public rclcpp_lifecycle::LifecycleNode
{
public:
  using ExecuteVelocity = walking_zoo_msgs::action::ExecuteVelocity;
  using GoalHandleExecuteVelocity = rclcpp_action::ServerGoalHandle<ExecuteVelocity>;
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
  walking_zoo_core::RobotProfile profile_from_parameters();
  bool is_active() const;

  void handle_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void publish_state();

  void handle_estop(
    const std::shared_ptr<walking_zoo_msgs::srv::EmergencyStop::Request> request,
    std::shared_ptr<walking_zoo_msgs::srv::EmergencyStop::Response> response);
  void handle_clear_fault(
    const std::shared_ptr<walking_zoo_msgs::srv::ClearFault::Request> request,
    std::shared_ptr<walking_zoo_msgs::srv::ClearFault::Response> response);
  void handle_set_locomotion_mode(
    const std::shared_ptr<walking_zoo_msgs::srv::SetLocomotionMode::Request> request,
    std::shared_ptr<walking_zoo_msgs::srv::SetLocomotionMode::Response> response);

  rclcpp_action::GoalResponse handle_velocity_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteVelocity::Goal> goal);
  rclcpp_action::CancelResponse handle_velocity_cancel(
    const std::shared_ptr<GoalHandleExecuteVelocity> goal_handle);
  void handle_velocity_accepted(
    const std::shared_ptr<GoalHandleExecuteVelocity> goal_handle);
  void execute_velocity_goal(
    const std::shared_ptr<GoalHandleExecuteVelocity> goal_handle);

  std::unique_ptr<AdapterLoader> adapter_loader_;
  std::shared_ptr<walking_zoo_core::WalkingAdapter> adapter_;
  walking_zoo_core::RobotProfile robot_profile_;
  walking_zoo_safety::SafetyPipeline safety_pipeline_;
  CommandArbiter command_arbiter_;
  ModeManager mode_manager_;

  rclcpp_lifecycle::LifecyclePublisher<walking_zoo_msgs::msg::WalkingState>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<walking_zoo_msgs::msg::AdapterStatus>::SharedPtr adapter_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<walking_zoo_msgs::msg::SafetyState>::SharedPtr safety_state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Service<walking_zoo_msgs::srv::EmergencyStop>::SharedPtr estop_srv_;
  rclcpp::Service<walking_zoo_msgs::srv::ClearFault>::SharedPtr clear_fault_srv_;
  rclcpp::Service<walking_zoo_msgs::srv::SetLocomotionMode>::SharedPtr set_mode_srv_;
  rclcpp_action::Server<ExecuteVelocity>::SharedPtr execute_velocity_server_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  walking_zoo_msgs::msg::WalkingState last_state_;
  std::string robot_profile_path_;
  std::string adapter_plugin_;
  bool allow_motion_{false};
};

}  // namespace walking_zoo_runtime

#endif  // WALKING_ZOO_RUNTIME__WALKING_RUNTIME_MANAGER_HPP_
