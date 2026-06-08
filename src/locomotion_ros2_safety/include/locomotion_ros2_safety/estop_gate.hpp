#ifndef LOCOMOTION_ROS2_SAFETY__ESTOP_GATE_HPP_
#define LOCOMOTION_ROS2_SAFETY__ESTOP_GATE_HPP_

namespace locomotion_ros2_safety
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

}  // namespace locomotion_ros2_safety

#endif  // LOCOMOTION_ROS2_SAFETY__ESTOP_GATE_HPP_
