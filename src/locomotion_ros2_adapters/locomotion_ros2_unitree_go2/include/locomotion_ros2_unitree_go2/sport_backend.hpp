#ifndef LOCOMOTION_ROS2_UNITREE_GO2__SPORT_BACKEND_HPP_
#define LOCOMOTION_ROS2_UNITREE_GO2__SPORT_BACKEND_HPP_

#include <memory>
#include <string>

#include "locomotion_ros2_unitree_go2/go2_sport_command.hpp"

namespace locomotion_ros2_unitree_go2
{

// Hardware dispatch boundary for the Unitree Go2 adapter. Everything above this
// interface (command translation, FSM gating, state reporting) is pure and SDK
// free; everything that actually talks to the robot lives behind it. Two
// implementations: a software-in-the-loop backend that records commands (always
// built, unit-tested), and an SDK2 backend that drives the vendor SportClient
// (compiled only when LOCOMOTION_ROS2_WITH_UNITREE_SDK2 is set). This mirrors the
// G1 adapter's dispatch-backend pattern, validating it across robot classes.
class Go2SportBackend
{
public:
  virtual ~Go2SportBackend() = default;

  // Short identifier for status/logging (e.g. "sil", "unitree_sdk2").
  virtual std::string name() const = 0;

  // Bring the dispatch channel up. For SIL this is a no-op that succeeds; for the
  // SDK2 backend this initialises the DDS channel factory and the SportClient on
  // the given network interface. Returns false if the channel could not be
  // brought up.
  virtual bool connect(const std::string & network_interface) = 0;

  // Whether commands reach real motors. False for SIL, true for the SDK2 backend.
  virtual bool dispatches_to_hardware() const = 0;

  virtual void set_mode(SportMode mode) = 0;
  virtual void send_velocity(const Go2VelocityCommand & cmd) = 0;
  virtual void send_posture(const Go2PostureCommand & cmd) = 0;
  virtual void emergency_damp() = 0;
};

// Software-in-the-loop backend: records the last dispatched command and mode so
// the adapter (and tests) can verify what *would* be sent to hardware, without
// linking or running the vendor SDK.
class SilSportBackend : public Go2SportBackend
{
public:
  std::string name() const override {return "sil";}
  bool connect(const std::string & network_interface) override;
  bool dispatches_to_hardware() const override {return false;}

  void set_mode(SportMode mode) override {last_mode_ = mode;}
  void send_velocity(const Go2VelocityCommand & cmd) override;
  void send_posture(const Go2PostureCommand & cmd) override;
  void emergency_damp() override;

  // Inspection helpers for tests.
  SportMode last_mode() const {return last_mode_;}
  const Go2VelocityCommand & last_velocity() const {return last_velocity_;}
  const Go2PostureCommand & last_posture() const {return last_posture_;}
  bool connected() const {return connected_;}

private:
  SportMode last_mode_{SportMode::STAND_DOWN};
  Go2VelocityCommand last_velocity_;
  Go2PostureCommand last_posture_;
  bool connected_{false};
};

// Build the backend selected at compile time: the SDK2 backend when built with
// vendor SDK support, otherwise the software-in-the-loop backend.
std::unique_ptr<Go2SportBackend> make_sport_backend();

}  // namespace locomotion_ros2_unitree_go2

#endif  // LOCOMOTION_ROS2_UNITREE_GO2__SPORT_BACKEND_HPP_
