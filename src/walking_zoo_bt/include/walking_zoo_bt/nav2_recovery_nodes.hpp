#ifndef WALKING_ZOO_BT__NAV2_RECOVERY_NODES_HPP_
#define WALKING_ZOO_BT__NAV2_RECOVERY_NODES_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include "walking_zoo_bt/check_walking_ready.hpp"
#include "walking_zoo_msgs/msg/walking_state.hpp"
#include "walking_zoo_msgs/srv/clear_fault.hpp"

namespace walking_zoo_bt
{

// Nav2-loadable condition node. Follows the Nav2 topic-condition convention: it
// pulls the ROS node from the "node" blackboard entry that the Nav2 bt_navigator
// sets, subscribes to /walking_zoo/state on its own callback group, and reports
// SUCCESS when the latest state says the robot is ready to walk. The readiness
// decision is delegated to the shared CheckWalkingReady core so the rule stays
// identical to the standalone recovery node and the unit tests.
class IsWalkingReadyCondition : public BT::ConditionNode
{
public:
  IsWalkingReadyCondition(const std::string & name, const BT::NodeConfiguration & conf);
  IsWalkingReadyCondition() = delete;

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

private:
  void stateCallback(walking_zoo_msgs::msg::WalkingState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<walking_zoo_msgs::msg::WalkingState>::SharedPtr state_sub_;
  CheckWalkingReady checker_;
  std::mutex state_mutex_;
  walking_zoo_msgs::msg::WalkingState latest_state_;
  bool has_state_{false};
  std::string state_topic_;
};

// Nav2-loadable service action node built on nav2_behavior_tree::BtServiceNode.
// It calls /walking_zoo/clear_fault through the very same machinery every Nav2
// service BT node uses (the shared "node", server_timeout, and
// wait_for_service_timeout from the blackboard), and reports SUCCESS only when
// the runtime confirms the fault is cleared.
class ClearWalkingFaultBtNode
  : public nav2_behavior_tree::BtServiceNode<walking_zoo_msgs::srv::ClearFault>
{
public:
  ClearWalkingFaultBtNode(const std::string & name, const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_completion(
    std::shared_ptr<walking_zoo_msgs::srv::ClearFault::Response> response) override;
  static BT::PortsList providedPorts();
};

}  // namespace walking_zoo_bt

#endif  // WALKING_ZOO_BT__NAV2_RECOVERY_NODES_HPP_
