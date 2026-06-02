# Changelog

## 0.1.0

- Initial ROS2 workspace structure.
- Stable walking runtime messages, services, and actions.
- C++ adapter contract.
- Safety pipeline skeleton with velocity limiter, watchdog, and estop gate.
- Mock adapter.
- Lifecycle runtime manager.
- Nav2 `/cmd_vel` bridge.
- Bringup launch files, examples, docs, and CI.
- README PyBullet simulation GIF assets and generator.
- README MuJoCo Unitree G1 humanoid simulation GIF and simulation-only visual tour.
- README Unitree G1 gait gallery covering forward walk, run, sidestep, and turn-in-place.
- Optional live MuJoCo Unitree G1 gait demo driven by `/cmd_vel` and semantic actions.
- One-command MuJoCo Unitree G1 gait showcase launch for walk, run, sidestep, turn, and e-stop.
- README hero MuJoCo Unitree G1 showcase GIF and asset validation helper.
- Runtime trace recorder and MuJoCo G1 runtime showcase launch with JSON/Markdown demo artifacts.
- Demo evidence documentation and richer runtime trace validator output.
- Improved MuJoCo Unitree G1 run gait pose, cadence, body lean, and README GIF assets.
- Added `walk_backward` reverse gait to the MuJoCo Unitree G1 visualizer, gait showcase sequence, hero showcase GIF, and a `move_backward` semantic-to-velocity runtime mapping.
- Added a command-to-visual traceability table to the demo evidence docs mapping each showcase action through `/cmd_vel`, the Nav2 bridge, runtime state, and the rendered gait.
- Expanded the README MuJoCo Unitree G1 gait gallery to a six-tile grid adding reverse walk and stand/stop alongside forward walk, run, sidestep, and turn-in-place.
- Added a dependency-free gait gallery GIF metadata validator (signature, dimensions, frame count, size) and a CI asset-validation job covering the showcase and gallery README assets.
- Added gtest coverage for the `walking_zoo_vla` semantic action mapper, including the new `move_backward`/`walk_backward` reverse-velocity mapping.
- Completed the semantic action mapper surface with `run_forward`, `sidestep_left`, `sidestep_right`, and `walk_forward` mappings, documented in the package README and covered by tests.
- Expanded command arbiter tests to cover the full teleop/Nav2/VLA priority ordering, source aliases, latest-wins ties, and the invariant that VLA intent never outranks operator override or safety.
- Strengthened the showcase asset validator to also check GIF/PNG dimensions and GIF frame count, matching the gait gallery validator.
- Added gtest coverage for the runtime mode manager, verifying the idle default, every valid locomotion mode, and rejection of unknown or out-of-range modes.
- Added body-pose visualization (`body_crouch`, `body_pitch`, `body_roll`) to the live MuJoCo Unitree G1 demo, mapping `MODE_BODY_POSE`-style semantic actions to held humanoid poses.
- Added a README MuJoCo Unitree G1 body-pose gallery GIF (neutral stand, crouch, pitch, roll) with shared gallery rendering and asset validation covering both galleries.
- Added a deterministic `FootstepPlanner` that builds an alternating-leg `FootstepPlan` (forward progress, lateral offset, sidestep drift, metadata) as a foundation for footstep markers and `ExecuteFootstepPlan`, covered by gtest.
- Added a `footstep_marker_publisher` node and launch file that publish the stub `FootstepPlan` and matching RViz `MarkerArray` foot/path markers on `/walking_zoo/footstep_plan` and `/walking_zoo/footstep_markers`.
- Added a placeholder `StepFeasibilityChecker` (max stride distance, lateral offset, and swing height) with gtest coverage, and used it to flag infeasible footsteps in red in the marker preview.
- Added a placeholder `FallDetector` (upright/tilted/fallen bands from body tilt) with gtest coverage, plus a fall-detected visualization pose in the live MuJoCo Unitree G1 demo.
- Implemented the `ExecuteFootstepPlan` action in the runtime manager: it runs the `StepFeasibilityChecker` gate, dispatches feasible plans to the adapter with per-step feedback and cancellation, aborts infeasible plans, and is covered by an end-to-end check (`tools/check_footstep_action_e2e.py`).
- Wired the `FallDetector` into the `SafetyPipeline` as a body-pose gate (`filter_body_pose`): it rejects requested torso tilts that fall into the fall band and clamps anything beyond the per-axis roll/pitch limits, covered by gtest, and surfaced the adapter fall flag in the published `SafetyState`.
- Implemented the `ExecuteBodyPose` action in the runtime manager: it runs the fall-aware body-pose safety gate, dispatches safe poses to the adapter with held-pose feedback and cancellation, rejects fall-band tilts, and is covered by an end-to-end check (`tools/check_bodypose_action_e2e.py`).
- Grew the Unitree SDK2 (G1) adapter from a stub into a software-in-the-loop model: added an SDK-free G1 command translation layer (`unitree_loco_command`) that clamps velocity/posture to the G1 envelope and models the loco FSM, made the adapter track real locomotion state (stand-up on activate, velocity→walking, body pose→balance-stand, e-stop damping) and honestly report footstep plans as unsupported, with gtest coverage and an end-to-end runtime-load check (`tools/check_unitree_adapter_e2e.py`).
- Made the Nav2 cmd_vel bridge legged-aware instead of a raw Twist passthrough: added a `LeggedVelocityShaper` that clamps to an asymmetric legged envelope (forward/back/lateral/yaw), rate-limits acceleration, dead-bands tiny side-steps, and sheds forward speed when turning hard; and gated the bridge on the runtime's published readiness (`/walking_zoo/state`) so velocities are held while the robot is e-stopped or unbalanced. Covered by gtest and an end-to-end check (`tools/check_legged_nav2_bridge_e2e.py`).
- Turned the BehaviorTree skeleton into real BehaviorTree.CPP 4.x nodes: wrapped the readiness and clear-fault decision cores in a `CheckWalkingReady` condition node and `ClearWalkingFault` action node with input ports, added a factory registration function and a loadable `BT_REGISTER_NODES` plugin entry point, rewrote the recovery `bt_xml` as a valid BTCPP v4 tree, and covered node registration, the shipped tree, and recovery outcomes by ticking real trees in gtest.
- Added a LeRobot dataset exporter (`walking_zoo_lerobot_export.py`) that resamples a `walking_zoo.demo_trace.v1` runtime trace into a LeRobot v2.1 dataset (`meta/info.json`, `tasks.jsonl`, `episodes.jsonl`, `stats.json`, and a parquet episode with a jsonl fallback), pairing the Nav2/teleop command as `action` with the executed velocity and locomotion state as `observation.state`. Covered by pytest and a CI-safe round-trip check (`tools/check_lerobot_export.py`).
- Grew the footstep planner from a flat-ground stub into a terrain-aware planner: added a `TerrainModel` (stacked axis-aligned keep-out and curb boxes over flat ground) and a `FootstepPlanner::plan_over_terrain` that nudges feet laterally out of keep-out zones, places them on raised patches, lifts the swing apex to clear step-ups, and reports blocked steps when no foothold is found. Wired it into the `footstep_marker_publisher` (new `no_step_zone`/`curb_box` params, amber-for-nudged / red-for-blocked markers), covered by gtest and an end-to-end check (`tools/check_footstep_terrain_e2e.py`).
- Wired the BehaviorTree.CPP recovery tree into a live `walking_zoo_bt_recovery_node`: added a ROS-integrated `ClearWalkingFaultService` BT action that actually calls `/walking_zoo/clear_fault`, a recovery node that subscribes to `/walking_zoo/state`, ticks the tree, and drives recovery (spinning on a background executor so the service call does not deadlock the tick loop), and a launch file. Fixed the matching runtime/adapter semantics so recovery is real: the mock adapter's `clear_fault` now re-enables the driver (clears the estop latch) and the runtime enforces the operator-estop interlock (a fault may not be cleared while the runtime estop is still engaged). Covered by gtest and an end-to-end check that proves the runtime stays faulted on its own and only the BT clears it (`tools/check_bt_recovery_e2e.py`).
- Gave the Unitree SDK2 (G1) adapter a real vendor-SDK link path. Introduced a `UnitreeLocoBackend` dispatch boundary so all hardware calls live behind one interface: a `SilLocoBackend` (always built, records what would be sent, unit-tested and verified wired through the adapter) and an `Sdk2LocoBackend` (`src/sdk2_loco_backend.cpp`, compiled only with `-DWALKING_ZOO_WITH_UNITREE_SDK2=ON`) that drives the G1 `LocoClient` (`Move`/`BalanceStand`/`Damp` over the DDS channel). Wired the CMake option to a real `find_package(unitree_sdk2 REQUIRED)` + link that fails loudly when the SDK is absent (so an ON build never silently degrades), removed the in-source `#ifdef`s from the adapter, and documented the `unitree_sdk2_DIR`/`CMAKE_PREFIX_PATH` setup.
- Extended the LeRobot exporter to collect multiple runtime traces into one multi-episode dataset: added `write_episodes_dataset` (one episode per trace) that de-duplicates tasks into a shared task table, keeps the global frame `index` continuous across episodes, shards episodes into `chunk-XYZ` directories, and computes `stats.json` over every frame; `write_dataset` is now a single-episode wrapper and the CLI accepts several trace paths. Covered by added pytest cases and a multi-episode round-trip in `tools/check_lerobot_export.py`.
- Embedded the walking_zoo recovery into a real Nav2 `bt_navigator` recovery
  branch. Added a Nav2-loadable BT plugin library `walking_zoo_nav2_bt_nodes`
  exporting `IsWalkingReady` (a topic condition that reads `/walking_zoo/state`
  via the `node` blackboard the bt_navigator sets) and `ClearWalkingFault` (built
  on `nav2_behavior_tree::BtServiceNode<ClearFault>`, so it uses the exact Nav2
  service-node machinery and succeeds only when the runtime confirms the fault is
  cleared). Shipped `bt_xml/navigate_to_pose_w_walking_recovery.xml` — the stock
  Nav2 navigate-to-pose tree with the walking recovery as the first action in the
  `RoundRobin` recovery set, guarded by `Inverter/IsWalkingReady` so it is a
  no-op when the robot is already ready — and `config/nav2_bt_navigator.yaml`
  which appends the plugin to `plugin_lib_names` and selects the tree. Verified
  three ways: `test_nav2_bt_recovery_nodes` loads the plugin the same way
  bt_navigator does and drives it against a fake runtime; a
  `walking_zoo_nav2_recovery_harness` ticks the recovery branch through the real
  `nav2_behavior_tree::BehaviorTreeEngine`, exercised end-to-end against the live
  runtime by `tools/check_nav2_bt_recovery_e2e.py` (runtime stays faulted alone,
  the Nav2-loaded branch clears it); and `tools/check_nav2_recovery_tree.py`
  statically guards the droppable navigate tree. The operator-estop interlock
  still holds through the Nav2 path.
- Added a second real-robot adapter, `walking_zoo_unitree_go2`, giving the adapter hub breadth beyond humanoids: a Unitree Go2 quadruped sport-mode adapter (`UnitreeGo2Adapter`) that reuses the dispatch-backend pattern with a `Go2SportBackend` (always-built software-in-the-loop `SilSportBackend`, plus an `Sdk2SportBackend` behind `-DWALKING_ZOO_WITH_UNITREE_SDK2=ON` driving the Go2 `SportClient` `Move`/`Euler`/`BodyHeight`/`RecoveryStand`/`StandDown`/`Damp`). The model is genuinely quadruped: it rests lying down (`STATE_SITTING`), stands up on activate and lies back down on deactivate, self-rights into balance-stand via recovery-stand, sits on a quick stop, reports four-foot `SUPPORT_QUADRUPED`, and tilts its torso via Euler angles. The vendor link is gated behind a real `find_package(unitree_sdk2 REQUIRED)` that fails loudly when the SDK is absent. Covered by gtest and an end-to-end runtime-load check (`tools/check_unitree_go2_adapter_e2e.py`) that loads the plugin, autostarts to a quadruped stand, and trots via the real `ExecuteVelocity` action.
