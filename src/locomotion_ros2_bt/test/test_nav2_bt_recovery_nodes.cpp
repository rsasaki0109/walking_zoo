// ROS-integrated tests for the Nav2-loadable locomotion_ros2 recovery nodes. The
// plugin library is loaded exactly the way the Nav2 bt_navigator loads it
// (BT::BehaviorTreeFactory::registerFromPlugin on the built .so), and the tree is
// driven against a fake /locomotion_ros2/clear_fault service plus a /locomotion_ros2/state
// publisher so the Nav2-loaded IsWalkingReady / ClearWalkingFault nodes are
// exercised through the real Nav2 BtServiceNode machinery.

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "locomotion_ros2_msgs/msg/walking_state.hpp"
#include "locomotion_ros2_msgs/srv/clear_fault.hpp"

using namespace std::chrono_literals;
using locomotion_ros2_msgs::msg::WalkingState;
using locomotion_ros2_msgs::srv::ClearFault;

namespace
{

const char * kBranchXml = R"(
<root BTCPP_format="4" main_tree_to_execute="WalkingRecoveryBranch">
  <BehaviorTree ID="WalkingRecoveryBranch">
    <Sequence name="WalkingFaultRecovery">
      <Fallback name="ReadyOrClear">
        <IsWalkingReady state_topic="/locomotion_ros2/state"/>
        <ClearWalkingFault service_name="/locomotion_ros2/clear_fault" reason="nav2_bt_recovery"/>
      </Fallback>
      <IsWalkingReady state_topic="/locomotion_ros2/state"/>
    </Sequence>
  </BehaviorTree>
</root>
)";

WalkingState make_ready_state()
{
  WalkingState state;
  state.adapter_connected = true;
  state.is_balanced = true;
  state.is_fallen = false;
  state.estop_active = false;
  state.locomotion_state = WalkingState::STATE_STANDING;
  return state;
}

WalkingState make_faulted_state()
{
  WalkingState state = make_ready_state();
  state.estop_active = true;
  state.locomotion_state = WalkingState::STATE_ESTOPPED;
  return state;
}

// A stand-in walking runtime: continuously publishes a WalkingState and answers
// /locomotion_ros2/clear_fault. When configured to recover, a successful clear flips
// the published state to ready -- modelling the real runtime's clear_fault.
class FakeRuntime
{
public:
  FakeRuntime()
  : node_(std::make_shared<rclcpp::Node>("fake_walking_runtime"))
  {
    state_pub_ = node_->create_publisher<WalkingState>("/locomotion_ros2/state", 10);
    service_ = node_->create_service<ClearFault>(
      "/locomotion_ros2/clear_fault",
      [this](
        const std::shared_ptr<ClearFault::Request>,
        std::shared_ptr<ClearFault::Response> response) {
        clear_calls_.fetch_add(1);
        response->success = clear_succeeds_;
        response->status_text = clear_succeeds_ ? "cleared" : "rejected";
        if (clear_succeeds_ && recover_on_clear_) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          state_ = make_ready_state();
        }
      });
    timer_ = node_->create_wall_timer(
      20ms, [this]() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_pub_->publish(state_);
      });

    executor_.add_node(node_);
    spin_thread_ = std::thread([this]() {executor_.spin();});
  }

  ~FakeRuntime()
  {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  void set_state(const WalkingState & state)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = state;
  }

  void configure_clear(bool succeeds, bool recover_on_clear)
  {
    clear_succeeds_ = succeeds;
    recover_on_clear_ = recover_on_clear;
  }

  int clear_calls() const {return clear_calls_.load();}
  rclcpp::Node::SharedPtr node() {return node_;}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<WalkingState>::SharedPtr state_pub_;
  rclcpp::Service<ClearFault>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spin_thread_;

  std::mutex state_mutex_;
  WalkingState state_;
  std::atomic<bool> clear_succeeds_{true};
  std::atomic<bool> recover_on_clear_{true};
  std::atomic<int> clear_calls_{0};
};

class Nav2RecoveryNodesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    runtime_ = std::make_unique<FakeRuntime>();
    bt_node_ = std::make_shared<rclcpp::Node>("bt_host_node");
    // Give DDS a moment so the clear_fault service is discoverable before the
    // BtServiceNode constructor waits on it.
    std::this_thread::sleep_for(500ms);
  }

  void TearDown() override
  {
    runtime_.reset();
    bt_node_.reset();
  }

  BT::Tree make_tree()
  {
    BT::BehaviorTreeFactory factory;
    // Load the plugin exactly as Nav2's bt_navigator does.
    factory.registerFromPlugin(LOCOMOTION_ROS2_NAV2_BT_NODES_LIB);

    auto blackboard = BT::Blackboard::create();
    blackboard->set<rclcpp::Node::SharedPtr>("node", bt_node_);
    blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
    blackboard->set<std::chrono::milliseconds>("server_timeout", 1000ms);
    blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", 5000ms);
    blackboard->set<int>("number_recoveries", 0);
    return factory.createTreeFromText(kBranchXml, blackboard);
  }

  // Tick the tree until SUCCESS or the deadline; returns the final status.
  BT::NodeStatus tick_until(BT::Tree & tree, std::chrono::seconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (std::chrono::steady_clock::now() < deadline) {
      status = tree.tickOnce();
      if (status == BT::NodeStatus::SUCCESS) {
        break;
      }
      std::this_thread::sleep_for(50ms);
    }
    return status;
  }

  std::unique_ptr<FakeRuntime> runtime_;
  rclcpp::Node::SharedPtr bt_node_;
};

TEST_F(Nav2RecoveryNodesTest, AlreadyReadySucceedsWithoutClearing)
{
  runtime_->set_state(make_ready_state());
  auto tree = make_tree();

  EXPECT_EQ(tick_until(tree, 5s), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(runtime_->clear_calls(), 0);
}

TEST_F(Nav2RecoveryNodesTest, FaultedStateClearsAndRecovers)
{
  runtime_->set_state(make_faulted_state());
  runtime_->configure_clear(/*succeeds=*/true, /*recover_on_clear=*/true);
  auto tree = make_tree();

  EXPECT_EQ(tick_until(tree, 8s), BT::NodeStatus::SUCCESS);
  EXPECT_GE(runtime_->clear_calls(), 1);
}

TEST_F(Nav2RecoveryNodesTest, RejectedClearDoesNotRecover)
{
  runtime_->set_state(make_faulted_state());
  runtime_->configure_clear(/*succeeds=*/false, /*recover_on_clear=*/false);
  auto tree = make_tree();

  EXPECT_NE(tick_until(tree, 3s), BT::NodeStatus::SUCCESS);
  EXPECT_GE(runtime_->clear_calls(), 1);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
