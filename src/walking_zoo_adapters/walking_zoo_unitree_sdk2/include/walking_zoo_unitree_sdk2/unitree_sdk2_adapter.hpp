#ifndef WALKING_ZOO_UNITREE_SDK2__UNITREE_SDK2_ADAPTER_HPP_
#define WALKING_ZOO_UNITREE_SDK2__UNITREE_SDK2_ADAPTER_HPP_

#include <memory>
#include <string>

#include "walking_zoo_core/walking_adapter.hpp"
#include "walking_zoo_unitree_sdk2/loco_backend.hpp"
#include "walking_zoo_unitree_sdk2/unitree_loco_command.hpp"

namespace walking_zoo_unitree_sdk2
{

class UnitreeSdk2Adapter : public walking_zoo_core::WalkingAdapter
{
public:
  UnitreeSdk2Adapter();
  ~UnitreeSdk2Adapter() override = default;

  // Movable (it owns a unique_ptr backend); copying a live adapter is not
  // meaningful. pluginlib only ever default-constructs it on the heap.
  UnitreeSdk2Adapter(UnitreeSdk2Adapter &&) = default;
  UnitreeSdk2Adapter & operator=(UnitreeSdk2Adapter &&) = default;

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

  // Read-only access to the dispatch backend (sil or unitree_sdk2), for status
  // reporting and tests that verify what was forwarded to hardware.
  const UnitreeLocoBackend * backend() const {return backend_.get();}

private:
  // Whether commands actually reach motors: only when the backend dispatches to
  // hardware (the SDK2 backend) *and* motion is allowed. The SIL backend never
  // reports true here, but still records commands for inspection.
  bool dispatch_to_hardware() const;
  // Whether a command should be forwarded to the backend at all: always for the
  // SIL backend (so it can record), and for hardware only when motion is allowed.
  bool should_forward() const;
  void enter_mode(LocoMode mode);

  walking_zoo_core::RobotProfile profile_;
  bool configured_{false};
  bool active_{false};
  bool allow_motion_{false};
  bool estop_active_{false};
  std::string status_text_{"Unitree SDK2 support not compiled"};

  LocoMode loco_mode_{LocoMode::ZERO_TORQUE};
  G1VelocityLimits velocity_limits_;
  G1PostureLimits posture_limits_;
  LocoVelocityCommand last_velocity_;
  bool has_velocity_command_{false};

  std::unique_ptr<UnitreeLocoBackend> backend_;
  std::string network_interface_{"lo"};
};

}  // namespace walking_zoo_unitree_sdk2

#endif  // WALKING_ZOO_UNITREE_SDK2__UNITREE_SDK2_ADAPTER_HPP_
