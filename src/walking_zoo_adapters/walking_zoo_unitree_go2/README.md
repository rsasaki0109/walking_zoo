# walking_zoo_unitree_go2

Walking adapter for the **Unitree Go2 quadruped**, driving the high-level sport
mode. It is the second real-robot adapter in the hub and validates that the
`walking_zoo_core::WalkingAdapter` contract and the dispatch-backend pattern
generalise beyond the G1 humanoid to a quadruped.

## What is different from the G1 adapter

The Go2 is not a humanoid with extra legs; the model is genuinely quadruped:

- **Rest pose.** A powered-on Go2 lies on the ground (`STAND_DOWN` →
  `STATE_SITTING`). It stands up on `activate()` and lies back down on
  `deactivate()`, instead of damping in place like the G1.
- **Self-righting.** The sport FSM allows recovery-standing into balance-stand
  from any state (including a damp), reflecting the Go2's `RecoveryStand`.
- **Quick stop sits.** `stop(QUICK)` sits the quadruped down (`STAND_DOWN`);
  `stop(EMERGENCY)` damps; `stop(NORMAL)` returns to balance-stand.
- **Support phase.** Standing/trotting reports `SUPPORT_QUADRUPED` (four feet).
- **Body pose.** Torso orientation maps to the SportClient `Euler(roll, pitch,
  yaw)` + `BodyHeight(height)` calls rather than the G1 balance-stand posture.

The asymmetric velocity envelope (faster forward than back, quick yaw) is derived
from the robot profile in `configure()`.

## Dispatch backend

All hardware calls live behind a `Go2SportBackend`:

- `SilSportBackend` is **always built**. It records the mode/velocity/posture
  that *would* be sent, so the adapter and its tests run with no vendor SDK and
  CI never needs vendor headers.
- `Sdk2SportBackend` (`src/sdk2_sport_backend.cpp`) is compiled **only** with
  `-DWALKING_ZOO_WITH_UNITREE_SDK2=ON`. It drives the Go2
  `unitree::robot::go2::SportClient` (`Move` / `Euler` / `BodyHeight` /
  `StandUp` / `StandDown` / `RecoveryStand` / `Damp`) over the DDS channel.

A compile-time factory (`make_sport_backend`) selects the backend, so the default
build stays green and the hardware path is isolated to one translation unit.

## Building with the vendor SDK

```bash
colcon build --packages-select walking_zoo_unitree_go2 \
  --cmake-args -DWALKING_ZOO_WITH_UNITREE_SDK2=ON \
  -Dunitree_sdk2_DIR=/opt/unitree_sdk2/lib/cmake/unitree_sdk2
```

`find_package(unitree_sdk2 REQUIRED)` fails loudly when the SDK is missing, so an
"on" build never silently degrades to the software-in-the-loop backend.

## Real robot prerequisites and safety

- Connect over the Go2's robot network interface (set `network_interface`, e.g.
  `eth0`); the SDK2 backend brings up the DDS channel factory on it.
- Motion is **disabled by default**. Real `Move`/`Euler` commands only reach the
  motors when the profile/runtime sets `allow_motion` true.
- `emergency_stop()` damps the robot regardless of `allow_motion` — stopping is
  always permitted.
- The Go2 high-level sport API has no footstep-placement interface, so
  `execute_footstep_plan` is rejected rather than approximated.
