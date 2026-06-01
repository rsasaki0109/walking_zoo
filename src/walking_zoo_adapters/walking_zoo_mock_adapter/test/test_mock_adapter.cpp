#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "walking_zoo_mock_adapter/mock_walking_adapter.hpp"

TEST(MockWalkingAdapter, StateTransitions)
{
  rclcpp::init(0, nullptr);
  walking_zoo_mock_adapter::MockWalkingAdapter adapter;
  walking_zoo_core::AdapterContext context(
    rclcpp::get_logger("test_mock_adapter"),
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));

  EXPECT_EQ(adapter.configure(context), walking_zoo_core::CallbackReturn::SUCCESS);
  EXPECT_EQ(adapter.activate(), walking_zoo_core::CallbackReturn::SUCCESS);

  geometry_msgs::msg::TwistStamped cmd;
  cmd.twist.linear.x = 0.2;
  const auto result = adapter.command_velocity(cmd);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(
    adapter.read_state().locomotion_state,
    walking_zoo_msgs::msg::WalkingState::STATE_WALKING);

  EXPECT_TRUE(adapter.emergency_stop().accepted);
  EXPECT_EQ(
    adapter.read_state().locomotion_state,
    walking_zoo_msgs::msg::WalkingState::STATE_ESTOPPED);
  EXPECT_FALSE(adapter.clear_fault().accepted);

  rclcpp::shutdown();
}
