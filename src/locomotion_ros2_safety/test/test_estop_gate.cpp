#include <gtest/gtest.h>

#include "locomotion_ros2_safety/estop_gate.hpp"
#include "locomotion_ros2_safety/safety_pipeline.hpp"

TEST(EStopGate, BlocksMotionWhenActive)
{
  locomotion_ros2_safety::EStopGate gate;
  EXPECT_TRUE(gate.permits_motion());

  gate.set_active(true);

  EXPECT_TRUE(gate.active());
  EXPECT_FALSE(gate.permits_motion());
}

TEST(SafetyPipeline, EStopBlocksVelocity)
{
  locomotion_ros2_safety::SafetyPipeline pipeline;
  pipeline.set_estop_active(true);

  geometry_msgs::msg::TwistStamped command;
  command.twist.linear.x = 0.1;

  const auto result = pipeline.filter_velocity(command, rclcpp::Time(0));

  EXPECT_FALSE(result.result.accepted);
  EXPECT_EQ(result.result.status, locomotion_ros2_core::CommandStatus::BLOCKED);
}
