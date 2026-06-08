#include "locomotion_ros2_safety/estop_gate.hpp"

namespace locomotion_ros2_safety
{

void EStopGate::set_active(bool active)
{
  active_ = active;
}

bool EStopGate::active() const
{
  return active_;
}

bool EStopGate::permits_motion() const
{
  return !active_;
}

}  // namespace locomotion_ros2_safety
