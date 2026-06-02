// SDK2 (vendor) locomotion backend for the Unitree G1. This entire translation
// unit is compiled ONLY when the package is built with
// -DWALKING_ZOO_WITH_UNITREE_SDK2=ON, which also links the vendor unitree_sdk2
// libraries (see CMakeLists.txt). When the SDK is not present this file expands
// to nothing, so default and CI builds never need the vendor headers.
#ifdef WALKING_ZOO_WITH_UNITREE_SDK2

#include "walking_zoo_unitree_sdk2/loco_backend.hpp"

#include <memory>
#include <string>

// Vendor headers (provided by the unitree_sdk2 package).
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

namespace walking_zoo_unitree_sdk2
{

// Drives the G1 high-level LocoClient. The method names below follow the
// unitree_sdk2 G1 LocoClient API; a few FSM ids differ between firmware
// revisions, so the mode mapping is kept in one place for easy adjustment.
class Sdk2LocoBackend : public UnitreeLocoBackend
{
public:
  std::string name() const override {return "unitree_sdk2";}

  bool connect(const std::string & network_interface) override
  {
    // Bring up the DDS channel on the robot network interface, then the client.
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    client_ = std::make_unique<unitree::robot::g1::LocoClient>();
    client_->Init();
    client_->SetTimeout(10.0f);
    connected_ = true;
    return connected_;
  }

  bool dispatches_to_hardware() const override {return true;}

  void set_mode(LocoMode mode) override
  {
    if (!client_) {
      return;
    }
    switch (mode) {
      case LocoMode::ZERO_TORQUE:
        client_->ZeroTorque();
        break;
      case LocoMode::DAMP:
        client_->Damp();
        break;
      case LocoMode::BALANCE_STAND:
        client_->StandUp();
        client_->BalanceStand(0);
        break;
      case LocoMode::LOCOMOTION:
        client_->Start();
        break;
    }
  }

  void send_velocity(const LocoVelocityCommand & cmd) override
  {
    if (client_) {
      client_->Move(
        static_cast<float>(cmd.vx), static_cast<float>(cmd.vy), static_cast<float>(cmd.vyaw));
    }
  }

  void send_posture(const LocoPostureCommand & cmd) override
  {
    // The G1 high-level API exposes balance-stand control; fine-grained torso
    // roll/pitch/height is firmware dependent. Command balance stand and apply
    // the foot/stand height where the API allows it.
    if (client_) {
      client_->BalanceStand(0);
      client_->SetStandHeight(static_cast<float>(cmd.height));
    }
  }

  void emergency_damp() override
  {
    if (client_) {
      client_->Damp();
    }
  }

private:
  std::unique_ptr<unitree::robot::g1::LocoClient> client_;
  bool connected_{false};
};

// SDK2 build factory: the adapter dispatches to the real robot.
std::unique_ptr<UnitreeLocoBackend> make_loco_backend()
{
  return std::make_unique<Sdk2LocoBackend>();
}

}  // namespace walking_zoo_unitree_sdk2

#endif  // WALKING_ZOO_WITH_UNITREE_SDK2
