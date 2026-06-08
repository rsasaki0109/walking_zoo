# locomotion_ros2_unitree_sdk2

Unitree SDK2 (G1 humanoid) adapter for locomotion_ros2.

It builds without the vendor SDK by default and runs as a **software-in-the-loop
(SIL) model**: it translates locomotion_ros2 commands into the Unitree G1 high-level
loco-client representation, clamps them to the G1 velocity/posture envelope,
tracks the locomotion FSM (`zero_torque` → `damp` / `balance_stand` →
`locomotion`), and reports a faithful `WalkingState` — all without touching
hardware. This lets the full runtime, actions, and safety pipeline be exercised
against a G1-shaped adapter today.

The command translation and FSM logic live in
`include/locomotion_ros2_unitree_sdk2/unitree_loco_command.hpp` and are covered by
unit tests; adapter behaviour (stand-up on activate, velocity → walking, body
pose → balance-stand, footstep unsupported, e-stop damping) is covered by
`test/test_unitree_sdk2_adapter.cpp`, and end-to-end loading through the real
runtime by `tools/check_unitree_adapter_e2e.py`.

## Dispatch backend boundary

All hardware dispatch goes through a single `UnitreeLocoBackend` interface
(`include/locomotion_ros2_unitree_sdk2/loco_backend.hpp`). Everything above it —
command translation, FSM gating, state reporting — is pure and SDK-free. There
are two implementations:

- `SilLocoBackend` (always built): records the mode/velocity/posture that *would*
  be sent to hardware, so the adapter and its tests can verify dispatch without
  the vendor SDK. This is the default backend.
- `Sdk2LocoBackend` (`src/sdk2_loco_backend.cpp`, compiled only with
  `-DLOCOMOTION_ROS2_WITH_UNITREE_SDK2=ON`): initialises the DDS channel and the G1
  `LocoClient` and forwards `Move`/`BalanceStand`/`Damp` calls to the robot.

`make_loco_backend()` selects the backend at compile time. The vendor headers
and symbols never appear in the default build.

## Driving real hardware

Install the vendor SDK and point CMake at it, then configure with the SDK option:

```bash
colcon build --packages-select locomotion_ros2_unitree_sdk2 \
  --cmake-args -DLOCOMOTION_ROS2_WITH_UNITREE_SDK2=ON \
               -Dunitree_sdk2_DIR=/path/to/unitree_sdk2/lib/cmake/unitree_sdk2
```

`find_package(unitree_sdk2 REQUIRED)` fails loudly if the SDK is missing, so an
ON build never silently falls back to the SIL backend. Vendor SDK types must not
appear in `locomotion_ros2_core` or `locomotion_ros2_msgs`; `src/sdk2_loco_backend.cpp`
is the only translation unit that includes vendor headers.

Real robot motion is disabled by default. The SDK-backed command path requires
`allow_motion:=true` and still passes through the locomotion_ros2 safety pipeline.
Footstep plans are reported as unsupported because the G1 high-level API exposes
no footstep-placement interface.
