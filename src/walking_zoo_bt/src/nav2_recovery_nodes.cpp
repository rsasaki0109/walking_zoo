#include "walking_zoo_bt/nav2_recovery_nodes.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"

namespace walking_zoo_bt
{

IsWalkingReadyCondition::IsWalkingReadyCondition(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  state_topic_ = "/walking_zoo/state";
  getInput("state_topic", state_topic_);

  // A dedicated callback group (not auto-added to the bt_navigator executor) that
  // we spin ourselves on each tick, so reading the latest state never races the
  // tree's main executor -- the standard Nav2 topic-condition pattern.
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;
  state_sub_ = node_->create_subscription<walking_zoo_msgs::msg::WalkingState>(
    state_topic_, rclcpp::QoS(10),
    std::bind(&IsWalkingReadyCondition::stateCallback, this, std::placeholders::_1),
    sub_options);
}

BT::PortsList IsWalkingReadyCondition::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "state_topic", "/walking_zoo/state", "WalkingState topic to monitor")
  };
}

void IsWalkingReadyCondition::stateCallback(walking_zoo_msgs::msg::WalkingState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_state_ = *msg;
  has_state_ = true;
}

BT::NodeStatus IsWalkingReadyCondition::tick()
{
  callback_group_executor_.spin_some();

  // The first time we are ticked the subscription may not have delivered a
  // sample yet. Reporting "not ready" then would trigger a spurious recovery, so
  // give the very first state a short, bounded chance to arrive (one-time only;
  // every subsequent tick reads the cached latest state without blocking).
  {
    std::unique_lock<std::mutex> lock(state_mutex_);
    if (!has_state_) {
      lock.unlock();
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
      while (std::chrono::steady_clock::now() < deadline) {
        callback_group_executor_.spin_some();
        {
          std::lock_guard<std::mutex> have(state_mutex_);
          if (has_state_) {
            break;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }

  walking_zoo_msgs::msg::WalkingState state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_state_) {
      return BT::NodeStatus::FAILURE;
    }
    state = latest_state_;
  }
  return checker_.tick(state) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

ClearWalkingFaultBtNode::ClearWalkingFaultBtNode(
  const std::string & name, const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtServiceNode<walking_zoo_msgs::srv::ClearFault>(
    name, conf, "/walking_zoo/clear_fault")
{
}

BT::PortsList ClearWalkingFaultBtNode::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<std::string>("reason", "nav2_bt_recovery", "Reason sent to clear_fault")
  });
}

void ClearWalkingFaultBtNode::on_tick()
{
  std::string reason = "nav2_bt_recovery";
  getInput("reason", reason);
  request_->reason = reason;
}

BT::NodeStatus ClearWalkingFaultBtNode::on_completion(
  std::shared_ptr<walking_zoo_msgs::srv::ClearFault::Response> response)
{
  if (response && response->success) {
    RCLCPP_INFO(
      node_->get_logger(), "clear_fault succeeded: %s", response->status_text.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(
    node_->get_logger(), "clear_fault rejected: %s",
    response ? response->status_text.c_str() : "no response");
  return BT::NodeStatus::FAILURE;
}

}  // namespace walking_zoo_bt

// Make the walking recovery nodes loadable as Nav2 bt_navigator plugins. Listing
// `walking_zoo_nav2_bt_nodes` in the bt_navigator `plugin_lib_names` parameter is
// enough for Nav2 to register IsWalkingReady and ClearWalkingFault into the tree.
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<walking_zoo_bt::IsWalkingReadyCondition>("IsWalkingReady");
  factory.registerNodeType<walking_zoo_bt::ClearWalkingFaultBtNode>("ClearWalkingFault");
}
