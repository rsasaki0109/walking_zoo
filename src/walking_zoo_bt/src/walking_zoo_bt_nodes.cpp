#include "walking_zoo_bt/walking_zoo_bt_nodes.hpp"

namespace walking_zoo_bt
{

CheckWalkingReadyCondition::CheckWalkingReadyCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckWalkingReadyCondition::providedPorts()
{
  return {
    BT::InputPort<walking_zoo_msgs::msg::WalkingState>(
      "walking_state", "Latest WalkingState read from /walking_zoo/state")
  };
}

BT::NodeStatus CheckWalkingReadyCondition::tick()
{
  walking_zoo_msgs::msg::WalkingState state;
  if (!getInput("walking_state", state)) {
    return BT::NodeStatus::FAILURE;
  }
  return checker_.tick(state) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

ClearWalkingFaultAction::ClearWalkingFaultAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList ClearWalkingFaultAction::providedPorts()
{
  return {
    BT::InputPort<bool>(
      "clear_succeeded", false, "Whether the clear-fault service call succeeded")
  };
}

BT::NodeStatus ClearWalkingFaultAction::tick()
{
  bool clear_succeeded = false;
  getInput("clear_succeeded", clear_succeeded);
  return clearer_.tick(clear_succeeded) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void register_walking_zoo_bt_nodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<CheckWalkingReadyCondition>("CheckWalkingReady");
  factory.registerNodeType<ClearWalkingFaultAction>("ClearWalkingFault");
}

}  // namespace walking_zoo_bt

// Make this library loadable as a BehaviorTree.CPP plugin via
// `factory.registerFromPlugin(...)`.
BT_REGISTER_NODES(factory)
{
  walking_zoo_bt::register_walking_zoo_bt_nodes(factory);
}
