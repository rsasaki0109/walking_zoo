# walking_zoo_unitree_sdk2

Unitree SDK2 (G1 humanoid) adapter for walking_zoo.

It builds without the vendor SDK by default and runs as a **software-in-the-loop
(SIL) model**: it translates walking_zoo commands into the Unitree G1 high-level
loco-client representation, clamps them to the G1 velocity/posture envelope,
tracks the locomotion FSM (`zero_torque` → `damp` / `balance_stand` →
`locomotion`), and reports a faithful `WalkingState` — all without touching
hardware. This lets the full runtime, actions, and safety pipeline be exercised
against a G1-shaped adapter today.

The command translation and FSM logic live in
`include/walking_zoo_unitree_sdk2/unitree_loco_command.hpp` and are covered by
unit tests; adapter behaviour (stand-up on activate, velocity → walking, body
pose → balance-stand, footstep unsupported, e-stop damping) is covered by
`test/test_unitree_sdk2_adapter.cpp`, and end-to-end loading through the real
runtime by `tools/check_unitree_adapter_e2e.py`.

## Driving real hardware

Configure with `-DWALKING_ZOO_WITH_UNITREE_SDK2=ON` only after installing and
wiring the vendor SDK in this package. Vendor SDK types must not appear in
`walking_zoo_core` or `walking_zoo_msgs`; the SDK-gated dispatch sites in
`unitree_sdk2_adapter.cpp` (marked with the vendor `LocoClient` calls) are the
only places that talk to hardware.

Real robot motion is disabled by default. The SDK-backed command path requires
`allow_motion:=true` and still passes through the walking_zoo safety pipeline.
Footstep plans are reported as unsupported because the G1 high-level API exposes
no footstep-placement interface.
