#include "locomotion_ros2_bt/recovery_bt.hpp"

#include <future>

#include "locomotion_ros2_bt/locomotion_ros2_bt_nodes.hpp"

namespace locomotion_ros2_bt
{

ClearWalkingFaultService::ClearWalkingFaultService(
  const std::string & name, const BT::NodeConfig & config,
  std::shared_ptr<RecoveryContext> context)
: BT::SyncActionNode(name, config),
  context_(std::move(context))
{
}

BT::PortsList ClearWalkingFaultService::providedPorts()
{
  return {
    BT::InputPort<std::string>("reason", "bt_recovery", "Reason sent to clear_fault"),
    BT::OutputPort<std::string>("status_text", "Status text returned by the runtime")
  };
}

BT::NodeStatus ClearWalkingFaultService::tick()
{
  if (!context_ || !context_->clear_fault_client) {
    return BT::NodeStatus::FAILURE;
  }
  auto & client = context_->clear_fault_client;
  auto logger = context_->node->get_logger();

  if (!client->wait_for_service(
      std::chrono::duration_cast<std::chrono::nanoseconds>(context_->discovery_timeout)))
  {
    RCLCPP_WARN(logger, "clear_fault service unavailable");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<locomotion_ros2_msgs::srv::ClearFault::Request>();
  getInput("reason", request->reason);

  auto future = client->async_send_request(request);
  // The owning node is spun by a background executor, so waiting here lets the
  // response arrive without re-entering the tick thread.
  if (future.wait_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(context_->service_timeout)) !=
    std::future_status::ready)
  {
    RCLCPP_WARN(logger, "clear_fault service call timed out");
    return BT::NodeStatus::FAILURE;
  }

  auto response = future.get();
  setOutput("status_text", response->status_text);
  if (!response->success) {
    RCLCPP_WARN(logger, "clear_fault rejected: %s", response->status_text.c_str());
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(logger, "clear_fault succeeded: %s", response->status_text.c_str());
  return BT::NodeStatus::SUCCESS;
}

void register_recovery_bt_nodes(
  BT::BehaviorTreeFactory & factory, std::shared_ptr<RecoveryContext> context)
{
  // The readiness condition is the same port-driven node used everywhere else;
  // the host refreshes the `walking_state` blackboard entry before each tick.
  factory.registerNodeType<CheckWalkingReadyCondition>("CheckWalkingReady");
  factory.registerNodeType<ClearWalkingFaultService>("ClearWalkingFaultService", context);
}

}  // namespace locomotion_ros2_bt
