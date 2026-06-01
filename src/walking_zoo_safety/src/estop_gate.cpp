#include "walking_zoo_safety/estop_gate.hpp"

namespace walking_zoo_safety
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

}  // namespace walking_zoo_safety
