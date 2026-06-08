#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "locomotion_ros2_mock_adapter/mock_walking_adapter.hpp"

TEST(MockWalkingAdapter, StateTransitions)
{
  rclcpp::init(0, nullptr);
  locomotion_ros2_mock_adapter::MockWalkingAdapter adapter;
  locomotion_ros2_core::AdapterContext context(
    rclcpp::get_logger("test_mock_adapter"),
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));

  EXPECT_EQ(adapter.configure(context), locomotion_ros2_core::CallbackReturn::SUCCESS);
  EXPECT_EQ(adapter.activate(), locomotion_ros2_core::CallbackReturn::SUCCESS);

  geometry_msgs::msg::TwistStamped cmd;
  cmd.twist.linear.x = 0.2;
  const auto result = adapter.command_velocity(cmd);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(
    adapter.read_state().locomotion_state,
    locomotion_ros2_msgs::msg::WalkingState::STATE_WALKING);

  EXPECT_TRUE(adapter.emergency_stop().accepted);
  EXPECT_EQ(
    adapter.read_state().locomotion_state,
    locomotion_ros2_msgs::msg::WalkingState::STATE_ESTOPPED);
  EXPECT_TRUE(adapter.read_state().estop_active);

  // clear_fault re-enables the driver: it clears the estop latch and returns the
  // robot to standing. The operator-estop interlock is enforced by the runtime,
  // not the adapter, so at this layer the call succeeds.
  EXPECT_TRUE(adapter.clear_fault().accepted);
  EXPECT_FALSE(adapter.read_state().estop_active);
  EXPECT_EQ(
    adapter.read_state().locomotion_state,
    locomotion_ros2_msgs::msg::WalkingState::STATE_STANDING);

  rclcpp::shutdown();
}
