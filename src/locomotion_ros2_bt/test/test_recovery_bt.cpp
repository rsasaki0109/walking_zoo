#include <chrono>
#include <memory>

#include <gtest/gtest.h>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "locomotion_ros2_bt/recovery_bt.hpp"
#include "locomotion_ros2_msgs/msg/walking_state.hpp"

namespace
{

locomotion_ros2_msgs::msg::WalkingState ready_state()
{
  locomotion_ros2_msgs::msg::WalkingState state;
  state.adapter_connected = true;
  state.is_balanced = true;
  state.is_fallen = false;
  state.estop_active = false;
  state.locomotion_state = locomotion_ros2_msgs::msg::WalkingState::STATE_STANDING;
  return state;
}

locomotion_ros2_msgs::msg::WalkingState estopped_state()
{
  auto state = ready_state();
  state.estop_active = true;
  state.is_balanced = false;
  state.locomotion_state = locomotion_ros2_msgs::msg::WalkingState::STATE_ESTOPPED;
  return state;
}

std::shared_ptr<locomotion_ros2_bt::RecoveryContext> make_context(const std::string & node_name)
{
  auto context = std::make_shared<locomotion_ros2_bt::RecoveryContext>();
  context->node = std::make_shared<rclcpp::Node>(node_name);
  context->clear_fault_client =
    context->node->create_client<locomotion_ros2_msgs::srv::ClearFault>("/locomotion_ros2/clear_fault");
  // Keep the no-server path fast for the unit test.
  context->discovery_timeout = std::chrono::milliseconds(100);
  context->service_timeout = std::chrono::milliseconds(200);
  return context;
}

BT::Tree build_tree(const std::shared_ptr<locomotion_ros2_bt::RecoveryContext> & context)
{
  BT::BehaviorTreeFactory factory;
  locomotion_ros2_bt::register_recovery_bt_nodes(factory, context);
  return factory.createTreeFromFile(LOCOMOTION_ROS2_RECOVERY_XML_PATH);
}

}  // namespace

TEST(RecoveryBt, LiveTreeBuildsFromFile)
{
  auto context = make_context("recovery_build");
  EXPECT_NO_THROW({auto tree = build_tree(context);});
}

TEST(RecoveryBt, ReadyStateShortCircuitsWithoutCallingService)
{
  auto context = make_context("recovery_ready");
  auto tree = build_tree(context);

  tree.rootBlackboard()->set("walking_state", ready_state());
  // No clear_fault server exists; a ready state must succeed without touching it.
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST(RecoveryBt, NotReadyWithoutServiceFailsGracefully)
{
  auto context = make_context("recovery_not_ready");
  auto tree = build_tree(context);

  tree.rootBlackboard()->set("walking_state", estopped_state());
  // CheckWalkingReady fails, ClearWalkingFaultService finds no server and the
  // whole tree fails without crashing.
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
