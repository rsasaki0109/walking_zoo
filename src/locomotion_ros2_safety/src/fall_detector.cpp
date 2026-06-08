#include "locomotion_ros2_safety/fall_detector.hpp"

#include <algorithm>
#include <cmath>

namespace locomotion_ros2_safety
{

FallDetector::FallDetector(double tilt_warn_rad, double tilt_fall_rad)
: tilt_warn_rad_(std::abs(tilt_warn_rad)),
  tilt_fall_rad_(std::abs(tilt_fall_rad))
{
  // Keep the warn threshold at or below the fall threshold so the bands are
  // ordered even if the caller passes them the other way around.
  if (tilt_warn_rad_ > tilt_fall_rad_) {
    std::swap(tilt_warn_rad_, tilt_fall_rad_);
  }
}

FallState FallDetector::classify(double roll, double pitch) const
{
  const double tilt = std::sqrt(roll * roll + pitch * pitch);
  if (tilt >= tilt_fall_rad_) {
    return FallState::FALLEN;
  }
  if (tilt >= tilt_warn_rad_) {
    return FallState::TILTED;
  }
  return FallState::UPRIGHT;
}

bool FallDetector::is_fallen(double roll, double pitch) const
{
  return classify(roll, pitch) == FallState::FALLEN;
}

double FallDetector::tilt_warn_rad() const
{
  return tilt_warn_rad_;
}

double FallDetector::tilt_fall_rad() const
{
  return tilt_fall_rad_;
}

}  // namespace locomotion_ros2_safety
