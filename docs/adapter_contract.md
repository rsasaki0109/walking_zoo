# Adapter Contract

Robot adapters implement `walking_zoo_core::WalkingAdapter` and are loaded with
pluginlib.

Required methods:

- `configure(context)`
- `activate()`
- `deactivate()`
- `cleanup()`
- `get_robot_profile()`
- `get_status()`
- `read_state()`
- `command_velocity(cmd)`
- `command_body_pose(cmd)`
- `execute_footstep_plan(plan)`
- `stop(mode)`
- `emergency_stop()`
- `clear_fault()`

## Rules

- Do not expose vendor SDK types outside the adapter package.
- Do not send real motion commands unless `allow_motion` is explicitly true.
- Keep SDK connection failures non-fatal to the runtime when possible.
- Return a meaningful `AdapterStatus` even when disconnected.
- Put robot capability differences in a profile, not scattered code branches.
- Report contracts the robot cannot honour as rejected, not silently faked
  (e.g. the Unitree G1 high-level API has no footstep interface, so
  `execute_footstep_plan` is rejected rather than approximated).

## Dispatch backend pattern

Keep command translation and FSM/state logic pure and SDK-free, and put every
call that touches hardware behind a small backend interface. The Unitree G1
adapter is the reference: a `UnitreeLocoBackend` with two implementations. The
Unitree Go2 quadruped adapter follows the same shape with a `Go2SportBackend`
(SIL + `SportClient`), confirming the pattern is robot-class agnostic — the
backend interface is identical even though the Go2 has a quadruped FSM (lie-down
rest, recovery-stand, four-foot support) and a different vendor client.

- A software-in-the-loop (SIL) backend is always built. It records the
  mode/velocity/posture that *would* be sent, so the adapter and its tests run
  with no vendor SDK and CI never needs vendor headers.
- A vendor backend is compiled only behind a CMake option that does a real
  `find_package(<vendor_sdk> REQUIRED)` + link. The option must fail loudly when
  the SDK is missing so an "on" build never silently degrades to SIL.

A compile-time factory selects the backend. This keeps the default build green
and the hardware path honest and isolated to one translation unit.

## Estop and fault semantics

The estop/fault responsibilities are layered, so keep them in the right place:

- The runtime owns the operator-estop interlock: it refuses `clear_fault` while
  the runtime estop is engaged.
- `emergency_stop()` engages the adapter estop/damp; stopping is always allowed,
  regardless of `allow_motion`.
- `clear_fault()` re-enables the driver: clear the driver fault, release the
  estop latch, and return to a standing/idle state. Do not add a second operator
  interlock inside the adapter — that belongs to the runtime.

This split is what lets an automated recovery policy (the BehaviorTree recovery
node) bring the robot back after the operator releases the estop.

## Checklist

1. Create a new adapter package.
2. Depend on `walking_zoo_core` and `pluginlib`.
3. Implement `WalkingAdapter`, dispatching hardware calls through a backend.
4. Export the adapter in `plugins.xml`.
5. Add a conservative robot profile.
6. Add mockable tests for lifecycle, command rejection, and backend dispatch.
7. Gate the vendor SDK behind a CMake option with a real `find_package` + link.
8. Document real robot prerequisites and safety steps.
