# Safety Model

locomotion_ros2 is default-deny for real robot motion. The mock adapter works out of
the box, but real adapters must require explicit `allow_motion:=true`.

## Current Gates

- Velocity limiter: clamps `linear.x`, `linear.y`, and `angular.z`.
- Command watchdog: blocks stale stamped commands after the configured timeout.
- E-stop gate: blocks all motion commands while active.

## Planned Gates

- Fall detector.
- Adapter health gate.
- Terrain-aware limiter.
- Footstep feasibility checker.
- Humanoid support margin checks.

The runtime must pass every command through the safety pipeline before calling a
robot adapter.
