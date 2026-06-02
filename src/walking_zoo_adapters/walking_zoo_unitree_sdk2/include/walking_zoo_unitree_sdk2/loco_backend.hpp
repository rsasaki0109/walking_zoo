#ifndef WALKING_ZOO_UNITREE_SDK2__LOCO_BACKEND_HPP_
#define WALKING_ZOO_UNITREE_SDK2__LOCO_BACKEND_HPP_

#include <memory>
#include <string>

#include "walking_zoo_unitree_sdk2/unitree_loco_command.hpp"

namespace walking_zoo_unitree_sdk2
{

// Hardware dispatch boundary for the Unitree G1 adapter. Everything above this
// interface (command translation, FSM gating, state reporting) is pure and SDK
// free; everything that actually talks to the robot lives behind it. There are
// two implementations: a software-in-the-loop backend that records commands
// (always built, unit-tested), and an SDK2 backend that drives the vendor
// LocoClient (compiled only when WALKING_ZOO_WITH_UNITREE_SDK2 is set).
class UnitreeLocoBackend
{
public:
  virtual ~UnitreeLocoBackend() = default;

  // Short identifier for status/logging (e.g. "sil", "unitree_sdk2").
  virtual std::string name() const = 0;

  // Bring the dispatch channel up. For SIL this is a no-op that succeeds; for the
  // SDK2 backend this initialises the DDS channel factory and the LocoClient on
  // the given network interface. Returns false if the channel could not be
  // brought up.
  virtual bool connect(const std::string & network_interface) = 0;

  // Whether commands reach real motors. False for SIL, true for the SDK2 backend.
  virtual bool dispatches_to_hardware() const = 0;

  virtual void set_mode(LocoMode mode) = 0;
  virtual void send_velocity(const LocoVelocityCommand & cmd) = 0;
  virtual void send_posture(const LocoPostureCommand & cmd) = 0;
  virtual void emergency_damp() = 0;
};

// Software-in-the-loop backend: records the last dispatched command and mode so
// the adapter (and tests) can verify what *would* be sent to hardware, without
// linking or running the vendor SDK.
class SilLocoBackend : public UnitreeLocoBackend
{
public:
  std::string name() const override {return "sil";}
  bool connect(const std::string & network_interface) override;
  bool dispatches_to_hardware() const override {return false;}

  void set_mode(LocoMode mode) override {last_mode_ = mode;}
  void send_velocity(const LocoVelocityCommand & cmd) override;
  void send_posture(const LocoPostureCommand & cmd) override;
  void emergency_damp() override;

  // Inspection helpers for tests.
  LocoMode last_mode() const {return last_mode_;}
  const LocoVelocityCommand & last_velocity() const {return last_velocity_;}
  const LocoPostureCommand & last_posture() const {return last_posture_;}
  bool connected() const {return connected_;}

private:
  LocoMode last_mode_{LocoMode::ZERO_TORQUE};
  LocoVelocityCommand last_velocity_;
  LocoPostureCommand last_posture_;
  bool connected_{false};
};

// Build the backend selected at compile time: the SDK2 backend when built with
// vendor SDK support, otherwise the software-in-the-loop backend.
std::unique_ptr<UnitreeLocoBackend> make_loco_backend();

}  // namespace walking_zoo_unitree_sdk2

#endif  // WALKING_ZOO_UNITREE_SDK2__LOCO_BACKEND_HPP_
