#ifndef LOCOMOTION_ROS2_GAIT_LAB_SIL__GAIT_LAB_SIL_MODEL_HPP_
#define LOCOMOTION_ROS2_GAIT_LAB_SIL__GAIT_LAB_SIL_MODEL_HPP_

#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "locomotion_ros2_core/command_result.hpp"
#include "locomotion_ros2_core/robot_profile.hpp"
#include "locomotion_ros2_core/types.hpp"
#include "locomotion_ros2_msgs/msg/adapter_status.hpp"
#include "locomotion_ros2_msgs/msg/walking_state.hpp"

namespace locomotion_ros2_gait_lab_sil
{

// Lifecycle / command-gating / state-fusion logic for the gait_lab SIL bridge
// adapter, with NO ROS I/O so it can be unit-tested in isolation — mirroring the
// SDK-free command-translation layer the Unitree adapters separate out.
//
// The "robot" is a MuJoCo G1 driven by a gait_lab controller (default the
// reinforcement-learned `rl-residual` policy) running in a companion Python sim
// node. This model owns the adapter-side bookkeeping: it gates commands on the
// lifecycle, decides which lifecycle control signal to forward to the sim, and
// fuses the most recent WalkingState the sim reported. If the sim has not
// reported recently (it is not running, or has died), `read_state` reports a
// disconnected, synthesized state instead of stale truth.
class GaitLabSilModel
{
public:
  // Control signals forwarded to the Python sim over the bridge.
  static constexpr const char * CTRL_ACTIVATE = "activate";
  static constexpr const char * CTRL_DEACTIVATE = "deactivate";
  static constexpr const char * CTRL_STOP_NORMAL = "stop_normal";
  static constexpr const char * CTRL_STOP_QUICK = "stop_quick";
  static constexpr const char * CTRL_ESTOP = "estop";
  static constexpr const char * CTRL_CLEAR_FAULT = "clear_fault";

  static constexpr const char * PLUGIN_NAME =
    "locomotion_ros2_gait_lab_sil/GaitLabSilAdapter";

  void configure(const locomotion_ros2_core::RobotProfile & profile);
  locomotion_ros2_core::CallbackReturn activate();
  locomotion_ros2_core::CallbackReturn deactivate();
  locomotion_ros2_core::CallbackReturn cleanup();

  // Command gates. `accepted()` on the result means the adapter should forward
  // the command to the sim; a rejected/blocked result must not be forwarded.
  locomotion_ros2_core::CommandResult command_velocity_gate(
    const geometry_msgs::msg::TwistStamped & cmd);
  locomotion_ros2_core::CommandResult body_pose_gate();
  locomotion_ros2_core::CommandResult footstep_gate();
  locomotion_ros2_core::CommandResult stop_gate(locomotion_ros2_core::StopMode mode);
  locomotion_ros2_core::CommandResult emergency_stop_gate();
  locomotion_ros2_core::CommandResult clear_fault_gate();

  // The control string to forward for a lifecycle/stop transition (empty = none).
  std::string control_for_stop(locomotion_ros2_core::StopMode mode) const;

  // Fuse a WalkingState reported by the sim, timestamped `now_sec`.
  void ingest_sim_state(const locomotion_ros2_msgs::msg::WalkingState & state, double now_sec);
  // Whether a sim state arrived within `freshness_timeout_sec()` of `now_sec`.
  bool sim_connected(double now_sec) const;
  void set_freshness_timeout(double seconds) {freshness_timeout_sec_ = seconds;}
  double freshness_timeout_sec() const {return freshness_timeout_sec_;}

  // The state the adapter should report: the fresh sim state if connected, else a
  // synthesized state reflecting the adapter's own lifecycle/estop bookkeeping.
  locomotion_ros2_msgs::msg::WalkingState read_state(double now_sec) const;
  locomotion_ros2_msgs::msg::AdapterStatus get_status(double now_sec) const;
  const locomotion_ros2_core::RobotProfile & profile() const {return profile_;}

  bool configured() const {return configured_;}
  bool active() const {return active_;}
  bool estop_active() const {return estop_active_;}

private:
  locomotion_ros2_core::RobotProfile profile_;
  bool configured_{false};
  bool active_{false};
  bool estop_active_{false};
  bool fault_active_{false};
  std::string status_text_{"unconfigured"};

  bool have_sim_state_{false};
  double last_sim_state_sec_{0.0};
  locomotion_ros2_msgs::msg::WalkingState sim_state_;
  double freshness_timeout_sec_{0.5};
};

}  // namespace locomotion_ros2_gait_lab_sil

#endif  // LOCOMOTION_ROS2_GAIT_LAB_SIL__GAIT_LAB_SIL_MODEL_HPP_
