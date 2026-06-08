#ifndef LOCOMOTION_ROS2_SAFETY__FALL_DETECTOR_HPP_
#define LOCOMOTION_ROS2_SAFETY__FALL_DETECTOR_HPP_

namespace locomotion_ros2_safety
{

enum class FallState
{
  UPRIGHT,
  TILTED,
  FALLEN
};

// Placeholder fall detector based purely on body tilt magnitude. It does not use
// contact, IMU acceleration, or a learned model; it just flags when the torso
// orientation leaves a conservative upright band so the runtime can hold motion
// until a real estimator exists.
class FallDetector
{
public:
  FallDetector() = default;
  FallDetector(double tilt_warn_rad, double tilt_fall_rad);

  // roll and pitch are body orientation angles in radians.
  FallState classify(double roll, double pitch) const;
  bool is_fallen(double roll, double pitch) const;

  double tilt_warn_rad() const;
  double tilt_fall_rad() const;

private:
  double tilt_warn_rad_{0.35};   // ~20 degrees
  double tilt_fall_rad_{0.70};   // ~40 degrees
};

}  // namespace locomotion_ros2_safety

#endif  // LOCOMOTION_ROS2_SAFETY__FALL_DETECTOR_HPP_
