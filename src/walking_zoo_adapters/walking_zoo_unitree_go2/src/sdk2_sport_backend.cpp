// SDK2 (vendor) sport-mode backend for the Unitree Go2. This entire translation
// unit is compiled ONLY when the package is built with
// -DWALKING_ZOO_WITH_UNITREE_SDK2=ON, which also links the vendor unitree_sdk2
// libraries (see CMakeLists.txt). When the SDK is not present this file expands
// to nothing, so default and CI builds never need the vendor headers.
#ifdef WALKING_ZOO_WITH_UNITREE_SDK2

#include "walking_zoo_unitree_go2/sport_backend.hpp"

#include <memory>
#include <string>

// Vendor headers (provided by the unitree_sdk2 package).
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

namespace walking_zoo_unitree_go2
{

// Drives the Go2 high-level SportClient. The method names below follow the
// unitree_sdk2 Go2 SportClient API. A few entries differ between firmware
// revisions, so the mode mapping is kept in one place for easy adjustment.
class Sdk2SportBackend : public Go2SportBackend
{
public:
  std::string name() const override {return "unitree_sdk2";}

  bool connect(const std::string & network_interface) override
  {
    // Bring up the DDS channel on the robot network interface, then the client.
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    client_ = std::make_unique<unitree::robot::go2::SportClient>();
    client_->SetTimeout(10.0f);
    client_->Init();
    connected_ = true;
    return connected_;
  }

  bool dispatches_to_hardware() const override {return true;}

  void set_mode(SportMode mode) override
  {
    if (!client_) {
      return;
    }
    switch (mode) {
      case SportMode::DAMP:
        client_->Damp();
        break;
      case SportMode::STAND_DOWN:
        client_->StandDown();
        break;
      case SportMode::BALANCE_STAND:
        // RecoveryStand brings the quadruped up from lying/fallen, then
        // BalanceStand holds it ready for velocity and posture commands.
        client_->RecoveryStand();
        client_->BalanceStand();
        break;
      case SportMode::LOCOMOTION:
        // Sport mode trots in response to Move(); ensure it is balance-standing.
        client_->BalanceStand();
        break;
    }
  }

  void send_velocity(const Go2VelocityCommand & cmd) override
  {
    if (client_) {
      client_->Move(
        static_cast<float>(cmd.vx), static_cast<float>(cmd.vy), static_cast<float>(cmd.vyaw));
    }
  }

  void send_posture(const Go2PostureCommand & cmd) override
  {
    // The Go2 exposes torso orientation (Euler) and a relative body height while
    // balance-standing.
    if (client_) {
      client_->Euler(
        static_cast<float>(cmd.roll), static_cast<float>(cmd.pitch),
        static_cast<float>(cmd.yaw));
      client_->BodyHeight(static_cast<float>(cmd.height));
    }
  }

  void emergency_damp() override
  {
    if (client_) {
      client_->Damp();
    }
  }

private:
  std::unique_ptr<unitree::robot::go2::SportClient> client_;
  bool connected_{false};
};

// SDK2 build factory: the adapter dispatches to the real robot.
std::unique_ptr<Go2SportBackend> make_sport_backend()
{
  return std::make_unique<Sdk2SportBackend>();
}

}  // namespace walking_zoo_unitree_go2

#endif  // WALKING_ZOO_WITH_UNITREE_SDK2
