#ifndef LOCOMOTION_ROS2_BT__RECOVERY_BT_HPP_
#define LOCOMOTION_ROS2_BT__RECOVERY_BT_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "locomotion_ros2_msgs/srv/clear_fault.hpp"

namespace locomotion_ros2_bt
{

// Shared ROS handles the live recovery tree needs. The owning node spins on a
// background executor so the BT action node below can block on the service
// future without deadlocking the tick thread.
struct RecoveryContext
{
  rclcpp::Node::SharedPtr node;
  rclcpp::Client<locomotion_ros2_msgs::srv::ClearFault>::SharedPtr clear_fault_client;
  std::chrono::duration<double> service_timeout{std::chrono::seconds(5)};
  std::chrono::duration<double> discovery_timeout{std::chrono::milliseconds(500)};
};

// BehaviorTree.CPP action node that actually calls /locomotion_ros2/clear_fault and
// returns SUCCESS only when the runtime reports the fault cleared. This is the
// ROS-integrated counterpart of the port-driven ClearWalkingFaultAction: it is
// what turns the recovery tree from a pure decision tree into a live actor.
class ClearWalkingFaultService : public BT::SyncActionNode
{
public:
  ClearWalkingFaultService(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<RecoveryContext> context);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<RecoveryContext> context_;
};

// Register the live recovery node types (CheckWalkingReady condition + the
// service-backed clear-fault action) on a factory bound to a RecoveryContext.
void register_recovery_bt_nodes(
  BT::BehaviorTreeFactory & factory, std::shared_ptr<RecoveryContext> context);

}  // namespace locomotion_ros2_bt

#endif  // LOCOMOTION_ROS2_BT__RECOVERY_BT_HPP_
