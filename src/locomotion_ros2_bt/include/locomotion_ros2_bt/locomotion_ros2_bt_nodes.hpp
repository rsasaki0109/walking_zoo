#ifndef LOCOMOTION_ROS2_BT__LOCOMOTION_ROS2_BT_NODES_HPP_
#define LOCOMOTION_ROS2_BT__LOCOMOTION_ROS2_BT_NODES_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"
#include "locomotion_ros2_bt/check_walking_ready.hpp"
#include "locomotion_ros2_bt/clear_walking_fault.hpp"
#include "locomotion_ros2_msgs/msg/walking_state.hpp"

namespace locomotion_ros2_bt
{

// BehaviorTree.CPP condition node: SUCCESS when the latest WalkingState (read
// from the `walking_state` input port) reports the robot ready to walk. The
// readiness decision is delegated to the reusable CheckWalkingReady core so the
// same rule is covered by both plain and BT tests.
class CheckWalkingReadyCondition : public BT::ConditionNode
{
public:
  CheckWalkingReadyCondition(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  CheckWalkingReady checker_;
};

// BehaviorTree.CPP action node: SUCCESS when a clear-fault attempt (reported via
// the `clear_succeeded` input port) succeeded, FAILURE otherwise. In a full
// deployment the port is wired to the result of the `/locomotion_ros2/clear_fault`
// service call performed by a ROS action/service node.
class ClearWalkingFaultAction : public BT::SyncActionNode
{
public:
  ClearWalkingFaultAction(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  ClearWalkingFault clearer_;
};

// Register the locomotion_ros2 node types on a factory (used by tests and by any
// host that links this library directly).
void register_locomotion_ros2_bt_nodes(BT::BehaviorTreeFactory & factory);

}  // namespace locomotion_ros2_bt

#endif  // LOCOMOTION_ROS2_BT__LOCOMOTION_ROS2_BT_NODES_HPP_
