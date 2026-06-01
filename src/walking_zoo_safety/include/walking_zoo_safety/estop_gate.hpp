#ifndef WALKING_ZOO_SAFETY__ESTOP_GATE_HPP_
#define WALKING_ZOO_SAFETY__ESTOP_GATE_HPP_

namespace walking_zoo_safety
{

class EStopGate
{
public:
  void set_active(bool active);
  bool active() const;
  bool permits_motion() const;

private:
  bool active_{false};
};

}  // namespace walking_zoo_safety

#endif  // WALKING_ZOO_SAFETY__ESTOP_GATE_HPP_
