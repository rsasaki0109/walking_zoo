#ifndef LOCOMOTION_ROS2_MOCK_ADAPTER__MOCK_WALKING_ADAPTER_HPP_
#define LOCOMOTION_ROS2_MOCK_ADAPTER__MOCK_WALKING_ADAPTER_HPP_

#include <string>

#include "locomotion_ros2_core/walking_adapter.hpp"

namespace locomotion_ros2_mock_adapter
{

class MockWalkingAdapter : public locomotion_ros2_core::WalkingAdapter
{
public:
  MockWalkingAdapter();
  ~MockWalkingAdapter() override = default;

  locomotion_ros2_core::CallbackReturn configure(
    const locomotion_ros2_core::AdapterContext & context) override;
  locomotion_ros2_core::CallbackReturn activate() override;
  locomotion_ros2_core::CallbackReturn deactivate() override;
  locomotion_ros2_core::CallbackReturn cleanup() override;

  locomotion_ros2_core::RobotProfile get_robot_profile() const override;
  locomotion_ros2_msgs::msg::AdapterStatus get_status() const override;
  locomotion_ros2_msgs::msg::WalkingState read_state() override;

  locomotion_ros2_core::CommandResult command_velocity(
    const geometry_msgs::msg::TwistStamped & cmd) override;
  locomotion_ros2_core::CommandResult command_body_pose(
    const locomotion_ros2_msgs::msg::BodyPoseCommand & cmd) override;
  locomotion_ros2_core::CommandResult execute_footstep_plan(
    const locomotion_ros2_msgs::msg::FootstepPlan & plan) override;

  locomotion_ros2_core::CommandResult stop(locomotion_ros2_core::StopMode mode) override;
  locomotion_ros2_core::CommandResult emergency_stop() override;
  locomotion_ros2_core::CommandResult clear_fault() override;

private:
  static bool is_nonzero_velocity(const geometry_msgs::msg::TwistStamped & cmd);

  locomotion_ros2_core::RobotProfile profile_;
  bool configured_{false};
  bool active_{false};
  bool estop_active_{false};
  bool fault_active_{false};
  std::uint8_t locomotion_state_{locomotion_ros2_msgs::msg::WalkingState::STATE_UNKNOWN};
  std::string status_text_{"unconfigured"};
};

}  // namespace locomotion_ros2_mock_adapter

#endif  // LOCOMOTION_ROS2_MOCK_ADAPTER__MOCK_WALKING_ADAPTER_HPP_
