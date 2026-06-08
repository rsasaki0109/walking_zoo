#include "locomotion_ros2_unitree_sdk2/loco_backend.hpp"

namespace locomotion_ros2_unitree_sdk2
{

bool SilLocoBackend::connect(const std::string & network_interface)
{
  (void)network_interface;
  connected_ = true;
  return true;
}

void SilLocoBackend::send_velocity(const LocoVelocityCommand & cmd)
{
  last_velocity_ = cmd;
}

void SilLocoBackend::send_posture(const LocoPostureCommand & cmd)
{
  last_posture_ = cmd;
}

void SilLocoBackend::emergency_damp()
{
  last_mode_ = LocoMode::DAMP;
  last_velocity_ = LocoVelocityCommand{};
}

#ifndef LOCOMOTION_ROS2_WITH_UNITREE_SDK2
// Default factory: with no vendor SDK linked, the adapter runs software in the
// loop. The SDK2 build provides its own make_loco_backend in sdk2_loco_backend.cpp.
std::unique_ptr<UnitreeLocoBackend> make_loco_backend()
{
  return std::make_unique<SilLocoBackend>();
}
#endif

}  // namespace locomotion_ros2_unitree_sdk2
