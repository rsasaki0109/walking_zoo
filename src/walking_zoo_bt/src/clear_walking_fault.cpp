#include "walking_zoo_bt/clear_walking_fault.hpp"

namespace walking_zoo_bt
{

bool ClearWalkingFault::tick(bool clear_fault_service_succeeded) const
{
  return clear_fault_service_succeeded;
}

}  // namespace walking_zoo_bt
