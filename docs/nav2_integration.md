# Nav2 Integration

Phase 1 uses a bridge from `/cmd_vel` to `/walking_zoo/cmd_vel`.

```text
Nav2 Controller -> /cmd_vel -> walking_zoo_nav2 cmd_vel_bridge
  -> /walking_zoo/cmd_vel -> WalkingRuntimeManager -> SafetyPipeline -> Adapter
```

This lets a legged robot appear as a Nav2 mobile base while keeping walking
execution inside walking_zoo.

## Walking Recovery In The Nav2 BT Navigator

Beyond the velocity bridge, walking_zoo ships BT nodes that drop directly into a
Nav2 `bt_navigator` recovery branch, so a walking-specific fault (an e-stop
residual, a driver fault) is recovered as part of normal Nav2 navigation instead
of needing a separate standalone node.

The `walking_zoo_nav2_bt_nodes` plugin library (in `walking_zoo_bt`) exports two
Nav2-loadable BT nodes that follow the Nav2 plugin convention — they take the ROS
node from the `node` blackboard entry the bt_navigator sets, and reuse the Nav2
`BtServiceNode` machinery:

- `IsWalkingReady` — condition. Subscribes to `/walking_zoo/state` and succeeds
  when the runtime reports the robot ready to walk (same `CheckWalkingReady` rule
  used everywhere else).
- `ClearWalkingFault` — service action built on
  `nav2_behavior_tree::BtServiceNode<walking_zoo_msgs::srv::ClearFault>`. Calls
  `/walking_zoo/clear_fault` and succeeds only when the runtime confirms the
  fault is cleared.

`bt_xml/navigate_to_pose_w_walking_recovery.xml` is the stock Nav2
navigate-to-pose tree with the walking recovery embedded as the first action in
the `RoundRobin` recovery set:

```xml
<RoundRobin name="RecoveryActions">
  <Sequence name="WalkingFaultRecovery">
    <Inverter><IsWalkingReady state_topic="/walking_zoo/state"/></Inverter>
    <ClearWalkingFault service_name="/walking_zoo/clear_fault"/>
  </Sequence>
  <Sequence name="ClearingActions"> ... </Sequence>
  <Spin .../>
  <Wait .../>
  <BackUp .../>
</RoundRobin>
```

The `Inverter` makes the walking recovery a no-op when the robot is already
ready, so Nav2 falls straight through to its generic costmap-clear / spin / wait
/ back-up recoveries. Enable it by overlaying `config/nav2_bt_navigator.yaml`,
which appends `walking_zoo_nav2_bt_nodes` to the bt_navigator `plugin_lib_names`
(Nav2 still loads its built-ins automatically) and points
`default_nav_to_pose_bt_xml` at the tree.

The operator-estop interlock still holds: the runtime refuses `clear_fault` while
its e-stop is engaged, so the Nav2 recovery branch can never override an operator
stop — it only clears residual faults after the operator releases.

The integration is verified three ways: `test_nav2_bt_recovery_nodes` loads the
plugin library exactly as bt_navigator does and drives the nodes against a fake
runtime; `tools/check_nav2_bt_recovery_e2e.py` ticks the branch through the real
`nav2_behavior_tree::BehaviorTreeEngine` against the live runtime and proves it
clears a residual fault; and `tools/check_nav2_recovery_tree.py` statically
guards that the droppable navigate tree keeps the walking recovery in the Nav2
recovery branch.

## Costmap-Driven Footstep Terrain

The footstep planner's terrain can be fed from a real Nav2 costmap instead of
hand-authored boxes. `footstep_marker_publisher` takes a `costmap_topic`
parameter; point it at a `nav_msgs/OccupancyGrid` (e.g. a Nav2
`global_costmap/costmap`) and cells at or above `occupied_threshold` become
keep-out footholds, so the planner nudges feet around real obstacles. An optional
`elevation_topic` (a second `OccupancyGrid` read as a coarse height field via
`elevation_height_per_unit`) drives step-up heights. When a costmap arrives the
planner adopts its frame so foothold queries align with the map cells. Grid yaw
is not modelled (cells are assumed axis-aligned), and the costmap is expected to
cover the planning origin — a robot-centred or transformed costmap is the natural
input.

```bash
ros2 launch walking_zoo_runtime footstep_markers.launch.py \
  costmap_topic:=/global_costmap/costmap
```

Covered by `test_occupancy_terrain` (OccupancyGrid → terrain → planner) and
`tools/check_footstep_costmap_e2e.py` (a live costmap nudging a real plan).

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
- Nav2 BT plugins for stand and sit (ready checks and fault recovery shipped via
  `walking_zoo_nav2_bt_nodes`).
