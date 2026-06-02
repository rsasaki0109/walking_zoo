#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "rclcpp/rclcpp.hpp"

// Harness that exercises the walking_zoo Nav2 recovery nodes through the *real*
// Nav2 BT loader (nav2_behavior_tree::BehaviorTreeEngine) -- the exact class the
// Nav2 bt_navigator uses to load `plugin_lib_names` and build a tree. It ticks
// only the walking recovery branch (not a full navigate tree, which would need
// costmaps and planner/controller servers), so the integration can be proven
// hardware- and map-free: the Nav2-loaded IsWalkingReady / ClearWalkingFault
// nodes drive a live runtime back to readiness.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("walking_zoo_nav2_recovery_harness");

  std::string default_xml;
  try {
    default_xml = ament_index_cpp::get_package_share_directory("walking_zoo_bt") +
      "/bt_xml/walking_zoo_nav2_recovery_branch.xml";
  } catch (const std::exception &) {
    default_xml = "";
  }
  const std::string xml_path = node->declare_parameter<std::string>("bt_xml_path", default_xml);
  const double timeout_sec = node->declare_parameter<double>("timeout_sec", 20.0);
  const double tick_period_sec = node->declare_parameter<double>("tick_period_sec", 0.2);
  const int server_timeout_ms = node->declare_parameter<int>("server_timeout_ms", 2000);

  // Spin the node so DDS discovery and any default-group work progress while the
  // BT (which uses its own per-node callback groups) ticks.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  int exit_code = 1;
  try {
    // The blackboard entries the Nav2 BtServiceNode machinery expects. The Nav2
    // bt_action_server sets these for every navigate tree; we set them directly.
    auto blackboard = BT::Blackboard::create();
    blackboard->set<rclcpp::Node::SharedPtr>("node", node);
    blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
    blackboard->set<std::chrono::milliseconds>(
      "server_timeout", std::chrono::milliseconds(server_timeout_ms));
    blackboard->set<std::chrono::milliseconds>(
      "wait_for_service_timeout", std::chrono::milliseconds(1000));
    blackboard->set<int>("number_recoveries", 0);

    nav2_behavior_tree::BehaviorTreeEngine engine({"walking_zoo_nav2_bt_nodes"}, node);
    auto tree = engine.createTreeFromFile(xml_path, blackboard);

    RCLCPP_INFO(
      node->get_logger(), "ticking Nav2 walking recovery branch '%s'", xml_path.c_str());

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timeout_sec));
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(tick_period_sec));

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      status = tree.tickOnce();
      if (status == BT::NodeStatus::SUCCESS) {
        break;
      }
      std::this_thread::sleep_for(period);
    }

    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "walking recovery branch SUCCEEDED: robot is ready");
      exit_code = 0;
    } else {
      RCLCPP_WARN(node->get_logger(), "walking recovery branch did not reach readiness");
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "harness error: %s", error.what());
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return exit_code;
}
