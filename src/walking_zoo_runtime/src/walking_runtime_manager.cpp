#include "walking_zoo_runtime/walking_runtime_manager.hpp"

#include <chrono>
#include <memory>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"
#include "walking_zoo_core/adapter_context.hpp"
#include "walking_zoo_core/robot_profile.hpp"

using namespace std::chrono_literals;

namespace walking_zoo_runtime
{

WalkingRuntimeManager::WalkingRuntimeManager(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("walking_zoo_runtime_manager", options),
  adapter_loader_(std::make_unique<AdapterLoader>())
{
  declare_parameters();
}

void WalkingRuntimeManager::declare_parameters()
{
  declare_parameter<bool>("autostart", false);
  declare_parameter<std::string>("adapter_plugin", "walking_zoo_mock_adapter/MockWalkingAdapter");
  declare_parameter<std::string>("robot_profile", "");
  declare_parameter<bool>("allow_motion", false);

  declare_parameter<std::string>("robot_model", "mock_legged_robot");
  declare_parameter<std::string>("robot_family", "mock");
  declare_parameter<std::string>("base_frame", "base_link");
  declare_parameter<std::string>("odom_frame", "odom");
  declare_parameter<std::string>("map_frame", "map");

  declare_parameter<double>("limits.max_linear_x", 0.3);
  declare_parameter<double>("limits.max_linear_y", 0.2);
  declare_parameter<double>("limits.max_angular_z", 0.5);
  declare_parameter<double>("limits.max_body_roll", 0.2);
  declare_parameter<double>("limits.max_body_pitch", 0.2);
  declare_parameter<double>("limits.command_timeout_sec", 0.25);
  declare_parameter<double>("limits.body_tilt_warn_rad", 0.35);
  declare_parameter<double>("limits.body_tilt_fall_rad", 0.70);
}

walking_zoo_core::RobotProfile WalkingRuntimeManager::profile_from_parameters()
{
  walking_zoo_core::RobotProfile profile;
  profile.robot_model = get_parameter("robot_model").as_string();
  profile.robot_family = get_parameter("robot_family").as_string();
  profile.adapter_plugin = get_parameter("adapter_plugin").as_string();
  profile.base_frame = get_parameter("base_frame").as_string();
  profile.odom_frame = get_parameter("odom_frame").as_string();
  profile.map_frame = get_parameter("map_frame").as_string();
  profile.max_linear_x = get_parameter("limits.max_linear_x").as_double();
  profile.max_linear_y = get_parameter("limits.max_linear_y").as_double();
  profile.max_angular_z = get_parameter("limits.max_angular_z").as_double();
  profile.max_body_roll = get_parameter("limits.max_body_roll").as_double();
  profile.max_body_pitch = get_parameter("limits.max_body_pitch").as_double();
  profile.command_timeout_sec = get_parameter("limits.command_timeout_sec").as_double();
  profile.real_robot_motion_allowed = get_parameter("allow_motion").as_bool();
  profile.status_text = "runtime profile from ROS parameters";

  const auto robot_profile_path = get_parameter("robot_profile").as_string();
  if (!robot_profile_path.empty() && robot_profile_path != "mock") {
    profile = walking_zoo_core::load_robot_profile_from_yaml(robot_profile_path, profile);
  }

  profile.real_robot_motion_allowed = get_parameter("allow_motion").as_bool();
  return profile;
}

WalkingRuntimeManager::LifecycleCallbackReturn WalkingRuntimeManager::on_configure(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  robot_profile_path_ = get_parameter("robot_profile").as_string();
  allow_motion_ = get_parameter("allow_motion").as_bool();
  robot_profile_ = profile_from_parameters();
  adapter_plugin_ = robot_profile_.adapter_plugin;

  safety_pipeline_.set_limits(
    {robot_profile_.max_linear_x, robot_profile_.max_linear_y, robot_profile_.max_angular_z});
  safety_pipeline_.set_command_timeout_sec(robot_profile_.command_timeout_sec);
  safety_pipeline_.set_body_pose_limits(robot_profile_.max_body_roll, robot_profile_.max_body_pitch);
  safety_pipeline_.set_fall_thresholds(
    get_parameter("limits.body_tilt_warn_rad").as_double(),
    get_parameter("limits.body_tilt_fall_rad").as_double());

  state_pub_ = create_publisher<walking_zoo_msgs::msg::WalkingState>(
    "/walking_zoo/state", rclcpp::SystemDefaultsQoS());
  adapter_status_pub_ = create_publisher<walking_zoo_msgs::msg::AdapterStatus>(
    "/walking_zoo/adapter_status", rclcpp::SystemDefaultsQoS());
  safety_state_pub_ = create_publisher<walking_zoo_msgs::msg::SafetyState>(
    "/walking_zoo/safety_state", rclcpp::SystemDefaultsQoS());

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "/walking_zoo/cmd_vel",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&WalkingRuntimeManager::handle_cmd_vel, this, std::placeholders::_1));

  estop_srv_ = create_service<walking_zoo_msgs::srv::EmergencyStop>(
    "/walking_zoo/estop",
    std::bind(
      &WalkingRuntimeManager::handle_estop,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  clear_fault_srv_ = create_service<walking_zoo_msgs::srv::ClearFault>(
    "/walking_zoo/clear_fault",
    std::bind(
      &WalkingRuntimeManager::handle_clear_fault,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  set_mode_srv_ = create_service<walking_zoo_msgs::srv::SetLocomotionMode>(
    "/walking_zoo/set_locomotion_mode",
    std::bind(
      &WalkingRuntimeManager::handle_set_locomotion_mode,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  execute_velocity_server_ = rclcpp_action::create_server<ExecuteVelocity>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "/walking_zoo/execute_velocity",
    std::bind(
      &WalkingRuntimeManager::handle_velocity_goal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &WalkingRuntimeManager::handle_velocity_cancel,
      this,
      std::placeholders::_1),
    std::bind(
      &WalkingRuntimeManager::handle_velocity_accepted,
      this,
      std::placeholders::_1));

  execute_footstep_server_ = rclcpp_action::create_server<ExecuteFootstepPlan>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "/walking_zoo/execute_footstep_plan",
    std::bind(
      &WalkingRuntimeManager::handle_footstep_goal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &WalkingRuntimeManager::handle_footstep_cancel,
      this,
      std::placeholders::_1),
    std::bind(
      &WalkingRuntimeManager::handle_footstep_accepted,
      this,
      std::placeholders::_1));

  execute_body_pose_server_ = rclcpp_action::create_server<ExecuteBodyPose>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "/walking_zoo/execute_body_pose",
    std::bind(
      &WalkingRuntimeManager::handle_body_pose_goal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &WalkingRuntimeManager::handle_body_pose_cancel,
      this,
      std::placeholders::_1),
    std::bind(
      &WalkingRuntimeManager::handle_body_pose_accepted,
      this,
      std::placeholders::_1));

  try {
    adapter_ = adapter_loader_->load(adapter_plugin_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to load walking adapter '%s': %s", adapter_plugin_.c_str(), error.what());
    return LifecycleCallbackReturn::FAILURE;
  }

  walking_zoo_core::AdapterContext context(get_logger(), get_clock());
  context.robot_profile = robot_profile_;
  context.allow_motion = allow_motion_;
  context.robot_profile_path = robot_profile_path_;
  if (adapter_->configure(context) != walking_zoo_core::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Walking adapter configure failed");
    return LifecycleCallbackReturn::FAILURE;
  }

  last_state_ = adapter_->read_state();
  state_timer_ = create_wall_timer(100ms, std::bind(&WalkingRuntimeManager::publish_state, this));

  RCLCPP_INFO(
    get_logger(),
    "Configured walking_zoo runtime with adapter '%s' (allow_motion=%s)",
    adapter_plugin_.c_str(),
    allow_motion_ ? "true" : "false");
  return LifecycleCallbackReturn::SUCCESS;
}

WalkingRuntimeManager::LifecycleCallbackReturn WalkingRuntimeManager::on_activate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  state_pub_->on_activate();
  adapter_status_pub_->on_activate();
  safety_state_pub_->on_activate();

  if (adapter_ && adapter_->activate() != walking_zoo_core::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Walking adapter activate failed");
    return LifecycleCallbackReturn::FAILURE;
  }
  publish_state();
  RCLCPP_INFO(get_logger(), "walking_zoo runtime active");
  return LifecycleCallbackReturn::SUCCESS;
}

WalkingRuntimeManager::LifecycleCallbackReturn WalkingRuntimeManager::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (adapter_) {
    adapter_->stop(walking_zoo_core::StopMode::NORMAL);
    adapter_->deactivate();
  }
  state_pub_->on_deactivate();
  adapter_status_pub_->on_deactivate();
  safety_state_pub_->on_deactivate();
  return LifecycleCallbackReturn::SUCCESS;
}

WalkingRuntimeManager::LifecycleCallbackReturn WalkingRuntimeManager::on_cleanup(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (adapter_) {
    adapter_->cleanup();
  }
  adapter_.reset();
  state_timer_.reset();
  execute_velocity_server_.reset();
  execute_footstep_server_.reset();
  execute_body_pose_server_.reset();
  cmd_vel_sub_.reset();
  estop_srv_.reset();
  clear_fault_srv_.reset();
  set_mode_srv_.reset();
  state_pub_.reset();
  adapter_status_pub_.reset();
  safety_state_pub_.reset();
  return LifecycleCallbackReturn::SUCCESS;
}

WalkingRuntimeManager::LifecycleCallbackReturn WalkingRuntimeManager::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (adapter_) {
    adapter_->stop(walking_zoo_core::StopMode::NORMAL);
    adapter_->deactivate();
  }
  return LifecycleCallbackReturn::SUCCESS;
}

WalkingRuntimeManager::LifecycleCallbackReturn WalkingRuntimeManager::on_error(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (adapter_) {
    adapter_->stop(walking_zoo_core::StopMode::EMERGENCY);
  }
  return LifecycleCallbackReturn::SUCCESS;
}

bool WalkingRuntimeManager::is_active() const
{
  return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

void WalkingRuntimeManager::handle_cmd_vel(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (!adapter_ || !is_active()) {
    return;
  }

  auto safety_result = safety_pipeline_.filter_velocity(*msg, now());
  if (!safety_result.result.accepted) {
    last_state_ = adapter_->read_state();
    last_state_.status_text = safety_result.result.message;
    publish_state();
    return;
  }

  const auto adapter_result = adapter_->command_velocity(safety_result.command);
  mode_manager_.set_mode(walking_zoo_msgs::msg::WalkingState::MODE_WALK);
  last_state_ = adapter_->read_state();
  if (!adapter_result.accepted) {
    last_state_.status_text = adapter_result.message;
  } else if (safety_result.result.status == walking_zoo_core::CommandStatus::LIMITED) {
    last_state_.status_text = safety_result.result.message;
  }
  publish_state();
}

void WalkingRuntimeManager::publish_state()
{
  if (!state_pub_ || !state_pub_->is_activated()) {
    return;
  }

  auto state = adapter_ ? adapter_->read_state() : last_state_;
  state.header.stamp = now();
  state.lifecycle_state = is_active() ?
    walking_zoo_msgs::msg::WalkingState::LIFECYCLE_ACTIVE :
    walking_zoo_msgs::msg::WalkingState::LIFECYCLE_INACTIVE;
  state.locomotion_mode = mode_manager_.mode();
  if (safety_pipeline_.estop_active()) {
    state.estop_active = true;
    state.locomotion_state = walking_zoo_msgs::msg::WalkingState::STATE_ESTOPPED;
  }
  state_pub_->publish(state);
  last_state_ = state;

  if (adapter_status_pub_ && adapter_status_pub_->is_activated() && adapter_) {
    auto adapter_status = adapter_->get_status();
    adapter_status.header.stamp = now();
    adapter_status_pub_->publish(adapter_status);
  }

  if (safety_state_pub_ && safety_state_pub_->is_activated()) {
    auto safety_state = safety_pipeline_.make_state_msg();
    safety_state.header.stamp = now();
    safety_state.fall_detected = state.is_fallen;
    if (state.is_fallen && !safety_state.estop_active) {
      safety_state.state = walking_zoo_msgs::msg::SafetyState::STATE_FAULT;
      safety_state.status_text = "fall detected";
    }
    safety_state_pub_->publish(safety_state);
  }
}

void WalkingRuntimeManager::handle_estop(
  const std::shared_ptr<walking_zoo_msgs::srv::EmergencyStop::Request> request,
  std::shared_ptr<walking_zoo_msgs::srv::EmergencyStop::Response> response)
{
  safety_pipeline_.set_estop_active(request->stop);
  if (request->stop && adapter_) {
    adapter_->emergency_stop();
  }

  response->success = true;
  response->estop_active = safety_pipeline_.estop_active();
  response->status_text = request->stop ? "estop active" : "estop gate released";
  publish_state();
}

void WalkingRuntimeManager::handle_clear_fault(
  const std::shared_ptr<walking_zoo_msgs::srv::ClearFault::Request> request,
  std::shared_ptr<walking_zoo_msgs::srv::ClearFault::Response> response)
{
  (void)request;
  if (!adapter_) {
    response->success = false;
    response->status_text = "adapter not loaded";
    return;
  }
  const auto result = adapter_->clear_fault();
  response->success = result.accepted;
  response->status_text = result.message;
  publish_state();
}

void WalkingRuntimeManager::handle_set_locomotion_mode(
  const std::shared_ptr<walking_zoo_msgs::srv::SetLocomotionMode::Request> request,
  std::shared_ptr<walking_zoo_msgs::srv::SetLocomotionMode::Response> response)
{
  response->accepted = mode_manager_.set_mode(request->locomotion_mode);
  response->active_locomotion_mode = mode_manager_.mode();
  response->status_text = response->accepted ? "locomotion mode updated" : "invalid locomotion mode";
  publish_state();
}

rclcpp_action::GoalResponse WalkingRuntimeManager::handle_velocity_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ExecuteVelocity::Goal> goal)
{
  (void)uuid;
  (void)goal;
  if (!is_active()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WalkingRuntimeManager::handle_velocity_cancel(
  const std::shared_ptr<GoalHandleExecuteVelocity> goal_handle)
{
  (void)goal_handle;
  if (adapter_) {
    adapter_->stop(walking_zoo_core::StopMode::QUICK);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WalkingRuntimeManager::handle_velocity_accepted(
  const std::shared_ptr<GoalHandleExecuteVelocity> goal_handle)
{
  std::thread{std::bind(&WalkingRuntimeManager::execute_velocity_goal, this, goal_handle)}.detach();
}

void WalkingRuntimeManager::execute_velocity_goal(
  const std::shared_ptr<GoalHandleExecuteVelocity> goal_handle)
{
  auto result = std::make_shared<ExecuteVelocity::Result>();
  if (!adapter_ || !is_active()) {
    result->success = false;
    result->status_text = "runtime inactive";
    goal_handle->abort(result);
    return;
  }

  const auto goal = goal_handle->get_goal();
  const auto safety_result = safety_pipeline_.filter_velocity(goal->command, now());
  if (!safety_result.result.accepted) {
    result->success = false;
    result->status_text = safety_result.result.message;
    goal_handle->abort(result);
    return;
  }

  const auto adapter_result = adapter_->command_velocity(safety_result.command);
  if (!adapter_result.accepted) {
    result->success = false;
    result->status_text = adapter_result.message;
    goal_handle->abort(result);
    return;
  }

  const auto start = now();
  const double duration_sec = std::max(0.0F, goal->duration_sec);
  rclcpp::Rate rate(10.0);
  while (rclcpp::ok() && (now() - start).seconds() < duration_sec) {
    if (goal_handle->is_canceling()) {
      if (adapter_) {
        adapter_->stop(walking_zoo_core::StopMode::QUICK);
      }
      result->success = false;
      result->status_text = "velocity goal canceled";
      goal_handle->canceled(result);
      return;
    }
    auto feedback = std::make_shared<ExecuteVelocity::Feedback>();
    feedback->header.stamp = now();
    feedback->state = adapter_->read_state();
    feedback->elapsed_sec = static_cast<float>((now() - start).seconds());
    goal_handle->publish_feedback(feedback);
    rate.sleep();
  }

  if (adapter_) {
    adapter_->stop(walking_zoo_core::StopMode::NORMAL);
  }
  result->success = true;
  result->status_text = "velocity goal complete";
  goal_handle->succeed(result);
}

rclcpp_action::GoalResponse WalkingRuntimeManager::handle_footstep_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ExecuteFootstepPlan::Goal> goal)
{
  (void)uuid;
  if (!is_active() || goal->plan.footsteps.empty()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WalkingRuntimeManager::handle_footstep_cancel(
  const std::shared_ptr<GoalHandleExecuteFootstepPlan> goal_handle)
{
  (void)goal_handle;
  if (adapter_) {
    adapter_->stop(walking_zoo_core::StopMode::QUICK);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WalkingRuntimeManager::handle_footstep_accepted(
  const std::shared_ptr<GoalHandleExecuteFootstepPlan> goal_handle)
{
  std::thread{std::bind(&WalkingRuntimeManager::execute_footstep_goal, this, goal_handle)}.detach();
}

void WalkingRuntimeManager::execute_footstep_goal(
  const std::shared_ptr<GoalHandleExecuteFootstepPlan> goal_handle)
{
  auto result = std::make_shared<ExecuteFootstepPlan::Result>();
  if (!adapter_ || !is_active()) {
    result->success = false;
    result->status_text = "runtime inactive";
    goal_handle->abort(result);
    return;
  }

  if (safety_pipeline_.estop_active()) {
    result->success = false;
    result->status_text = "footstep plan blocked: estop active";
    goal_handle->abort(result);
    return;
  }

  const auto goal = goal_handle->get_goal();
  const auto & plan = goal->plan;

  // Reject obviously out-of-range plans before touching the adapter.
  const auto feasibility = feasibility_checker_.evaluate(plan, StepFeasibilityLimits{});
  if (!feasibility.feasible) {
    std::string reason = "footstep plan infeasible";
    for (std::size_t i = 0; i < feasibility.steps.size(); ++i) {
      if (!feasibility.steps[i].feasible) {
        reason += " (step " + std::to_string(i) + ": " + feasibility.steps[i].reason + ")";
        break;
      }
    }
    result->success = false;
    result->status_text = reason;
    goal_handle->abort(result);
    return;
  }

  const auto adapter_result = adapter_->execute_footstep_plan(plan);
  if (!adapter_result.accepted) {
    result->success = false;
    result->status_text = adapter_result.message;
    goal_handle->abort(result);
    return;
  }

  mode_manager_.set_mode(walking_zoo_msgs::msg::WalkingState::MODE_FOOTSTEP);

  const std::size_t total_steps = plan.footsteps.size();
  const double per_step_sec = plan.footsteps.front().duration > 0.0F ?
    static_cast<double>(plan.footsteps.front().duration) : 0.5;

  for (std::size_t completed = 0; completed < total_steps; ++completed) {
    const auto step_start = now();
    while (rclcpp::ok() && (now() - step_start).seconds() < per_step_sec) {
      if (goal_handle->is_canceling()) {
        if (adapter_) {
          adapter_->stop(walking_zoo_core::StopMode::QUICK);
        }
        result->success = false;
        result->status_text = "footstep plan canceled";
        goal_handle->canceled(result);
        return;
      }
      std::this_thread::sleep_for(20ms);
    }

    auto feedback = std::make_shared<ExecuteFootstepPlan::Feedback>();
    feedback->header.stamp = now();
    feedback->state = adapter_->read_state();
    feedback->completed_steps = static_cast<std::uint32_t>(completed + 1);
    goal_handle->publish_feedback(feedback);
  }

  if (adapter_) {
    adapter_->stop(walking_zoo_core::StopMode::NORMAL);
  }
  result->success = true;
  result->status_text = "footstep plan complete";
  goal_handle->succeed(result);
}

rclcpp_action::GoalResponse WalkingRuntimeManager::handle_body_pose_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ExecuteBodyPose::Goal> goal)
{
  (void)uuid;
  (void)goal;
  if (!is_active()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WalkingRuntimeManager::handle_body_pose_cancel(
  const std::shared_ptr<GoalHandleExecuteBodyPose> goal_handle)
{
  (void)goal_handle;
  if (adapter_) {
    adapter_->stop(walking_zoo_core::StopMode::QUICK);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WalkingRuntimeManager::handle_body_pose_accepted(
  const std::shared_ptr<GoalHandleExecuteBodyPose> goal_handle)
{
  std::thread{std::bind(&WalkingRuntimeManager::execute_body_pose_goal, this, goal_handle)}.detach();
}

void WalkingRuntimeManager::execute_body_pose_goal(
  const std::shared_ptr<GoalHandleExecuteBodyPose> goal_handle)
{
  auto result = std::make_shared<ExecuteBodyPose::Result>();
  if (!adapter_ || !is_active()) {
    result->success = false;
    result->status_text = "runtime inactive";
    goal_handle->abort(result);
    return;
  }

  if (safety_pipeline_.estop_active()) {
    result->success = false;
    result->status_text = "body pose blocked: estop active";
    goal_handle->abort(result);
    return;
  }

  const auto goal = goal_handle->get_goal();

  // Fall-aware safety gate: reject over-tilt that would topple the torso, clamp
  // anything still beyond the per-axis body limits.
  const auto safety = safety_pipeline_.filter_body_pose(goal->command);
  if (!safety.result.accepted) {
    result->success = false;
    result->status_text = safety.result.message;
    goal_handle->abort(result);
    return;
  }

  const auto adapter_result = adapter_->command_body_pose(safety.command);
  if (!adapter_result.accepted) {
    result->success = false;
    result->status_text = adapter_result.message;
    goal_handle->abort(result);
    return;
  }

  mode_manager_.set_mode(walking_zoo_msgs::msg::WalkingState::MODE_BODY_POSE);

  const auto start = now();
  const double duration_sec = std::max(0.0F, goal->command.duration_sec);
  rclcpp::Rate rate(10.0);
  bool first = true;
  while (rclcpp::ok() && (first || (now() - start).seconds() < duration_sec)) {
    first = false;
    if (goal_handle->is_canceling()) {
      if (adapter_) {
        adapter_->stop(walking_zoo_core::StopMode::QUICK);
      }
      result->success = false;
      result->status_text = "body pose canceled";
      goal_handle->canceled(result);
      return;
    }
    auto feedback = std::make_shared<ExecuteBodyPose::Feedback>();
    feedback->header.stamp = now();
    feedback->state = adapter_->read_state();
    feedback->elapsed_sec = static_cast<float>((now() - start).seconds());
    goal_handle->publish_feedback(feedback);
    if ((now() - start).seconds() >= duration_sec) {
      break;
    }
    rate.sleep();
  }

  if (adapter_) {
    adapter_->stop(walking_zoo_core::StopMode::NORMAL);
  }
  result->success = true;
  result->status_text = safety.result.status == walking_zoo_core::CommandStatus::LIMITED ?
    safety.result.message : "body pose complete";
  goal_handle->succeed(result);
}

}  // namespace walking_zoo_runtime
