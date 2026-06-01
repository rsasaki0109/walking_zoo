# Nav2 Integration

Phase 1 uses a bridge from `/cmd_vel` to `/walking_zoo/cmd_vel`.

```text
Nav2 Controller -> /cmd_vel -> walking_zoo_nav2 cmd_vel_bridge
  -> /walking_zoo/cmd_vel -> WalkingRuntimeManager -> SafetyPipeline -> Adapter
```

This lets a legged robot appear as a Nav2 mobile base while keeping walking
execution inside walking_zoo.

## Frames

walking_zoo follows REP-105 style expectations:

- `map`: globally consistent navigation frame.
- `odom`: locally continuous odometry frame.
- `base_link`: robot base frame.

## Future Work

- Dynamic footprint and support polygon.
- Footstep-aware local planning.
- Turn-in-place footstep sequences.
- Stairs, slope, and terrain capability profiles.
- Nav2 BT plugins for stand, sit, ready checks, and fault recovery.
