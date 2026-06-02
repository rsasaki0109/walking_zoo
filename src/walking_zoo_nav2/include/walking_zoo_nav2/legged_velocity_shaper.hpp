#ifndef WALKING_ZOO_NAV2__LEGGED_VELOCITY_SHAPER_HPP_
#define WALKING_ZOO_NAV2__LEGGED_VELOCITY_SHAPER_HPP_

namespace walking_zoo_nav2
{

// Motion envelope and shaping parameters for a legged base. Nav2 controllers
// assume a wheeled/holonomic plant and can demand velocities a gait controller
// cannot realize (instant reversals, tiny lateral steps, fast forward while
// spinning). These bound and smooth the command toward what a walker accepts.
struct LeggedMotionLimits
{
  double max_forward{0.6};        // m/s
  double max_backward{0.3};       // m/s (walkers back up slower)
  double max_lateral{0.3};        // m/s
  double max_yaw_rate{0.8};       // rad/s
  double max_linear_accel{1.0};   // m/s^2, per-axis rate limit
  double max_yaw_accel{2.0};      // rad/s^2
  double lateral_deadband{0.05};  // m/s, suppress tiny side-steps
  double turn_speed_coupling{0.7};  // 0..1, forward speed cut when turning hard
};

struct ShapedVelocity
{
  double vx{0.0};
  double vy{0.0};
  double vyaw{0.0};
  bool modified{false};  // true if shaping changed the raw command
};

// Stateful: tracks the previously emitted command so it can apply acceleration
// (rate) limits across calls. Construct one per stream and feed it the elapsed
// time since the last call.
class LeggedVelocityShaper
{
public:
  LeggedVelocityShaper() = default;
  explicit LeggedVelocityShaper(const LeggedMotionLimits & limits);

  void set_limits(const LeggedMotionLimits & limits);
  const LeggedMotionLimits & limits() const;

  // Forget the previous command (e.g. after a stop or estop) so the next call
  // is not rate-limited against stale state.
  void reset();

  // Shape a raw velocity. `dt` is the time since the previous call in seconds;
  // pass 0 or negative to skip acceleration limiting for this call.
  ShapedVelocity shape(double vx, double vy, double vyaw, double dt);

private:
  LeggedMotionLimits limits_;
  double last_vx_{0.0};
  double last_vy_{0.0};
  double last_vyaw_{0.0};
  bool has_last_{false};
};

}  // namespace walking_zoo_nav2

#endif  // WALKING_ZOO_NAV2__LEGGED_VELOCITY_SHAPER_HPP_
