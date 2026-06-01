#ifndef WALKING_ZOO_BT__CLEAR_WALKING_FAULT_HPP_
#define WALKING_ZOO_BT__CLEAR_WALKING_FAULT_HPP_

namespace walking_zoo_bt
{

class ClearWalkingFault
{
public:
  bool tick(bool clear_fault_service_succeeded) const;
};

}  // namespace walking_zoo_bt

#endif  // WALKING_ZOO_BT__CLEAR_WALKING_FAULT_HPP_
