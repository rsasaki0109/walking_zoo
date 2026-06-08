#include <gtest/gtest.h>

#include "behaviortree_cpp/bt_factory.h"
#include "locomotion_ros2_bt/locomotion_ros2_bt_nodes.hpp"
#include "locomotion_ros2_msgs/msg/walking_state.hpp"

using WalkingState = locomotion_ros2_msgs::msg::WalkingState;

namespace
{

WalkingState ready_state()
{
  WalkingState state;
  state.adapter_connected = true;
  state.is_balanced = true;
  state.is_fallen = false;
  state.estop_active = false;
  state.locomotion_state = WalkingState::STATE_STANDING;
  return state;
}

WalkingState fallen_state()
{
  WalkingState state;
  state.adapter_connected = true;
  state.is_balanced = false;
  state.is_fallen = true;
  state.estop_active = false;
  state.locomotion_state = WalkingState::STATE_FALLEN;
  return state;
}

constexpr const char * kRecoveryXml = R"(
<root BTCPP_format="4" main_tree_to_execute="LocomotionRos2Recovery">
  <BehaviorTree ID="LocomotionRos2Recovery">
    <Fallback name="locomotion_ros2_recovery">
      <CheckWalkingReady walking_state="{walking_state}"/>
      <Sequence name="clear_and_recheck">
        <ClearWalkingFault clear_succeeded="{clear_succeeded}"/>
        <CheckWalkingReady walking_state="{walking_state}"/>
      </Sequence>
    </Fallback>
  </BehaviorTree>
</root>
)";

BT::BehaviorTreeFactory make_factory()
{
  BT::BehaviorTreeFactory factory;
  locomotion_ros2_bt::register_locomotion_ros2_bt_nodes(factory);
  return factory;
}

}  // namespace

TEST(LocomotionRos2BtNodes, RegistersNodeTypes)
{
  auto factory = make_factory();
  const auto & manifests = factory.manifests();
  EXPECT_TRUE(manifests.count("CheckWalkingReady"));
  EXPECT_TRUE(manifests.count("ClearWalkingFault"));
}

TEST(LocomotionRos2BtNodes, RecoveryTreeSucceedsWhenReady)
{
  auto factory = make_factory();
  auto tree = factory.createTreeFromText(kRecoveryXml);
  tree.rootBlackboard()->set("walking_state", ready_state());
  tree.rootBlackboard()->set("clear_succeeded", false);
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
}

TEST(LocomotionRos2BtNodes, RecoveryTreeFailsWhenFallenAndClearFails)
{
  auto factory = make_factory();
  auto tree = factory.createTreeFromText(kRecoveryXml);
  tree.rootBlackboard()->set("walking_state", fallen_state());
  tree.rootBlackboard()->set("clear_succeeded", false);
  // First check fails; clear fails, so the sequence fails and the fallback fails.
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::FAILURE);
}

TEST(LocomotionRos2BtNodes, RecoveryTreeRunsClearBranchWhenNotReady)
{
  auto factory = make_factory();
  auto tree = factory.createTreeFromText(kRecoveryXml);
  // Not ready (estopped), but the clear-fault attempt succeeds; the re-check
  // still uses the same (not-ready) state, so overall recovery is incomplete.
  WalkingState estopped = ready_state();
  estopped.estop_active = true;
  estopped.is_balanced = false;
  tree.rootBlackboard()->set("walking_state", estopped);
  tree.rootBlackboard()->set("clear_succeeded", true);
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::FAILURE);
}

#ifdef LOCOMOTION_ROS2_BT_XML_PATH
TEST(LocomotionRos2BtNodes, ShippedRecoveryXmlIsValidAndTicks)
{
  auto factory = make_factory();
  // The bt_xml file installed with the package must parse against the
  // registered nodes and tick to SUCCESS for a ready robot.
  auto tree = factory.createTreeFromFile(LOCOMOTION_ROS2_BT_XML_PATH);
  tree.rootBlackboard()->set("walking_state", ready_state());
  tree.rootBlackboard()->set("clear_succeeded", false);
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
}
#endif

TEST(LocomotionRos2BtNodes, ConditionFailsWithoutWalkingStatePort)
{
  auto factory = make_factory();
  // A bare condition with no blackboard entry must fail closed, not throw.
  auto tree = factory.createTreeFromText(R"(
    <root BTCPP_format="4" main_tree_to_execute="T">
      <BehaviorTree ID="T">
        <CheckWalkingReady walking_state="{walking_state}"/>
      </BehaviorTree>
    </root>
  )");
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::FAILURE);
}
