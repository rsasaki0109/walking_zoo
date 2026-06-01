#ifndef WALKING_ZOO_MOCK_ADAPTER__MOCK_WALKING_ADAPTER_HPP_
#define WALKING_ZOO_MOCK_ADAPTER__MOCK_WALKING_ADAPTER_HPP_

#include <string>

#include "walking_zoo_core/walking_adapter.hpp"

namespace walking_zoo_mock_adapter
{

class MockWalkingAdapter : public walking_zoo_core::WalkingAdapter
{
public:
  MockWalkingAdapter();
  ~MockWalkingAdapter() override = default;

  walking_zoo_core::CallbackReturn configure(
    const walking_zoo_core::AdapterContext & context) override;
  walking_zoo_core::CallbackReturn activate() override;
  walking_zoo_core::CallbackReturn deactivate() override;
  walking_zoo_core::CallbackReturn cleanup() override;

  walking_zoo_core::RobotProfile get_robot_profile() const override;
  walking_zoo_msgs::msg::AdapterStatus get_status() const override;
  walking_zoo_msgs::msg::WalkingState read_state() override;

  walking_zoo_core::CommandResult command_velocity(
    const geometry_msgs::msg::TwistStamped & cmd) override;
  walking_zoo_core::CommandResult command_body_pose(
    const walking_zoo_msgs::msg::BodyPoseCommand & cmd) override;
  walking_zoo_core::CommandResult execute_footstep_plan(
    const walking_zoo_msgs::msg::FootstepPlan & plan) override;

  walking_zoo_core::CommandResult stop(walking_zoo_core::StopMode mode) override;
  walking_zoo_core::CommandResult emergency_stop() override;
  walking_zoo_core::CommandResult clear_fault() override;

private:
  static bool is_nonzero_velocity(const geometry_msgs::msg::TwistStamped & cmd);

  walking_zoo_core::RobotProfile profile_;
  bool configured_{false};
  bool active_{false};
  bool estop_active_{false};
  bool fault_active_{false};
  std::uint8_t locomotion_state_{walking_zoo_msgs::msg::WalkingState::STATE_UNKNOWN};
  std::string status_text_{"unconfigured"};
};

}  // namespace walking_zoo_mock_adapter

#endif  // WALKING_ZOO_MOCK_ADAPTER__MOCK_WALKING_ADAPTER_HPP_
