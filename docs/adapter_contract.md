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

## Checklist

1. Create a new adapter package.
2. Depend on `walking_zoo_core` and `pluginlib`.
3. Implement `WalkingAdapter`.
4. Export the adapter in `plugins.xml`.
5. Add a conservative robot profile.
6. Add mockable tests for lifecycle and command rejection.
7. Document real robot prerequisites and safety steps.
