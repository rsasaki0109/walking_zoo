#include "locomotion_ros2_bt/clear_walking_fault.hpp"

namespace locomotion_ros2_bt
{

bool ClearWalkingFault::tick(bool clear_fault_service_succeeded) const
{
  return clear_fault_service_succeeded;
}

}  // namespace locomotion_ros2_bt
