#include "locomotion_ros2_unitree_go2/sport_backend.hpp"

namespace locomotion_ros2_unitree_go2
{

bool SilSportBackend::connect(const std::string & network_interface)
{
  (void)network_interface;
  connected_ = true;
  return true;
}

void SilSportBackend::send_velocity(const Go2VelocityCommand & cmd)
{
  last_velocity_ = cmd;
}

void SilSportBackend::send_posture(const Go2PostureCommand & cmd)
{
  last_posture_ = cmd;
}

void SilSportBackend::emergency_damp()
{
  last_mode_ = SportMode::DAMP;
  last_velocity_ = Go2VelocityCommand{};
}

#ifndef LOCOMOTION_ROS2_WITH_UNITREE_SDK2
// Default factory: with no vendor SDK linked, the adapter runs software in the
// loop. The SDK2 build provides its own make_sport_backend in
// sdk2_sport_backend.cpp.
std::unique_ptr<Go2SportBackend> make_sport_backend()
{
  return std::make_unique<SilSportBackend>();
}
#endif

}  // namespace locomotion_ros2_unitree_go2
