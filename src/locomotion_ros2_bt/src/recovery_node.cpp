#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "locomotion_ros2_bt/recovery_bt.hpp"
#include "locomotion_ros2_msgs/msg/walking_state.hpp"

// A live locomotion_ros2 recovery node. It subscribes to /locomotion_ros2/state, ticks
// the recovery behavior tree, and lets the ClearWalkingFaultService BT node call
// /locomotion_ros2/clear_fault when the robot is not ready. This is the missing wire
// that turns the BehaviorTree.CPP skeleton into something that actually drives a
// running runtime: state in, service call out, readiness re-checked.
class LocomotionRos2RecoveryNode
{
public:
  explicit LocomotionRos2RecoveryNode(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    std::string default_xml;
    try {
      default_xml = ament_index_cpp::get_package_share_directory("locomotion_ros2_bt") +
        "/bt_xml/locomotion_ros2_recovery_live.xml";
    } catch (const std::exception &) {
      default_xml = "";
    }
    const std::string xml_path = node_->declare_parameter<std::string>("bt_xml_path", default_xml);
    const std::string state_topic =
      node_->declare_parameter<std::string>("state_topic", "/locomotion_ros2/state");
    const std::string clear_fault_service =
      node_->declare_parameter<std::string>("clear_fault_service", "/locomotion_ros2/clear_fault");
    tick_period_ = node_->declare_parameter<double>("tick_period_sec", 0.5);
    const double service_timeout = node_->declare_parameter<double>("service_timeout_sec", 5.0);

    context_ = std::make_shared<locomotion_ros2_bt::RecoveryContext>();
    context_->node = node_;
    context_->clear_fault_client =
      node_->create_client<locomotion_ros2_msgs::srv::ClearFault>(clear_fault_service);
    context_->service_timeout = std::chrono::duration<double>(service_timeout);

    BT::BehaviorTreeFactory factory;
    locomotion_ros2_bt::register_recovery_bt_nodes(factory, context_);
    tree_ = factory.createTreeFromFile(xml_path);

    state_sub_ = node_->create_subscription<locomotion_ros2_msgs::msg::WalkingState>(
      state_topic, rclcpp::QoS(10),
      [this](locomotion_ros2_msgs::msg::WalkingState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_state_ = *msg;
        has_state_ = true;
      });

    RCLCPP_INFO(
      node_->get_logger(),
      "locomotion_ros2 recovery node ticking '%s' every %.2fs", xml_path.c_str(), tick_period_);
  }

  // Tick the tree once against the latest state. Returns the tree status, or
  // nullopt when no state has been received yet.
  std::optional<BT::NodeStatus> tick_once()
  {
    locomotion_ros2_msgs::msg::WalkingState state;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!has_state_) {
        return std::nullopt;
      }
      state = latest_state_;
    }
    tree_.rootBlackboard()->set("walking_state", state);
    const auto status = tree_.tickOnce();

    if (status == BT::NodeStatus::SUCCESS && last_status_ != BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "locomotion_ros2 recovery: robot is ready");
    } else if (status == BT::NodeStatus::FAILURE && last_status_ != BT::NodeStatus::FAILURE) {
      RCLCPP_WARN(node_->get_logger(), "locomotion_ros2 recovery: robot not ready, recovering");
    }
    last_status_ = status;
    return status;
  }

  double tick_period() const {return tick_period_;}

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<locomotion_ros2_bt::RecoveryContext> context_;
  BT::Tree tree_;
  rclcpp::Subscription<locomotion_ros2_msgs::msg::WalkingState>::SharedPtr state_sub_;

  std::mutex state_mutex_;
  locomotion_ros2_msgs::msg::WalkingState latest_state_;
  bool has_state_{false};
  BT::NodeStatus last_status_{BT::NodeStatus::IDLE};
  double tick_period_{0.5};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("locomotion_ros2_bt_recovery");
  auto recovery = std::make_shared<LocomotionRos2RecoveryNode>(node);

  // Spin the node on a background executor so the BT service call can block on
  // its response from the tick loop below without deadlocking.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  const auto period = std::chrono::duration<double>(recovery->tick_period());
  while (rclcpp::ok()) {
    recovery->tick_once();
    std::this_thread::sleep_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period));
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
