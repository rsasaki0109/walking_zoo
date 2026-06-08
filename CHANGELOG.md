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
- Added gtest coverage for the `locomotion_ros2_vla` semantic action mapper, including the new `move_backward`/`walk_backward` reverse-velocity mapping.
- Completed the semantic action mapper surface with `run_forward`, `sidestep_left`, `sidestep_right`, and `walk_forward` mappings, documented in the package README and covered by tests.
- Expanded command arbiter tests to cover the full teleop/Nav2/VLA priority ordering, source aliases, latest-wins ties, and the invariant that VLA intent never outranks operator override or safety.
- Strengthened the showcase asset validator to also check GIF/PNG dimensions and GIF frame count, matching the gait gallery validator.
- Added gtest coverage for the runtime mode manager, verifying the idle default, every valid locomotion mode, and rejection of unknown or out-of-range modes.
- Added body-pose visualization (`body_crouch`, `body_pitch`, `body_roll`) to the live MuJoCo Unitree G1 demo, mapping `MODE_BODY_POSE`-style semantic actions to held humanoid poses.
- Added a README MuJoCo Unitree G1 body-pose gallery GIF (neutral stand, crouch, pitch, roll) with shared gallery rendering and asset validation covering both galleries.
- Added a deterministic `FootstepPlanner` that builds an alternating-leg `FootstepPlan` (forward progress, lateral offset, sidestep drift, metadata) as a foundation for footstep markers and `ExecuteFootstepPlan`, covered by gtest.
- Added a `footstep_marker_publisher` node and launch file that publish the stub `FootstepPlan` and matching RViz `MarkerArray` foot/path markers on `/locomotion_ros2/footstep_plan` and `/locomotion_ros2/footstep_markers`.
- Added a placeholder `StepFeasibilityChecker` (max stride distance, lateral offset, and swing height) with gtest coverage, and used it to flag infeasible footsteps in red in the marker preview.
- Added a placeholder `FallDetector` (upright/tilted/fallen bands from body tilt) with gtest coverage, plus a fall-detected visualization pose in the live MuJoCo Unitree G1 demo.
- Implemented the `ExecuteFootstepPlan` action in the runtime manager: it runs the `StepFeasibilityChecker` gate, dispatches feasible plans to the adapter with per-step feedback and cancellation, aborts infeasible plans, and is covered by an end-to-end check (`tools/check_footstep_action_e2e.py`).
- Wired the `FallDetector` into the `SafetyPipeline` as a body-pose gate (`filter_body_pose`): it rejects requested torso tilts that fall into the fall band and clamps anything beyond the per-axis roll/pitch limits, covered by gtest, and surfaced the adapter fall flag in the published `SafetyState`.
- Implemented the `ExecuteBodyPose` action in the runtime manager: it runs the fall-aware body-pose safety gate, dispatches safe poses to the adapter with held-pose feedback and cancellation, rejects fall-band tilts, and is covered by an end-to-end check (`tools/check_bodypose_action_e2e.py`).
- Grew the Unitree SDK2 (G1) adapter from a stub into a software-in-the-loop model: added an SDK-free G1 command translation layer (`unitree_loco_command`) that clamps velocity/posture to the G1 envelope and models the loco FSM, made the adapter track real locomotion state (stand-up on activate, velocity→walking, body pose→balance-stand, e-stop damping) and honestly report footstep plans as unsupported, with gtest coverage and an end-to-end runtime-load check (`tools/check_unitree_adapter_e2e.py`).
- Made the Nav2 cmd_vel bridge legged-aware instead of a raw Twist passthrough: added a `LeggedVelocityShaper` that clamps to an asymmetric legged envelope (forward/back/lateral/yaw), rate-limits acceleration, dead-bands tiny side-steps, and sheds forward speed when turning hard; and gated the bridge on the runtime's published readiness (`/locomotion_ros2/state`) so velocities are held while the robot is e-stopped or unbalanced. Covered by gtest and an end-to-end check (`tools/check_legged_nav2_bridge_e2e.py`).
- Turned the BehaviorTree skeleton into real BehaviorTree.CPP 4.x nodes: wrapped the readiness and clear-fault decision cores in a `CheckWalkingReady` condition node and `ClearWalkingFault` action node with input ports, added a factory registration function and a loadable `BT_REGISTER_NODES` plugin entry point, rewrote the recovery `bt_xml` as a valid BTCPP v4 tree, and covered node registration, the shipped tree, and recovery outcomes by ticking real trees in gtest.
- Added a LeRobot dataset exporter (`locomotion_ros2_lerobot_export.py`) that resamples a `locomotion_ros2.demo_trace.v1` runtime trace into a LeRobot v2.1 dataset (`meta/info.json`, `tasks.jsonl`, `episodes.jsonl`, `stats.json`, and a parquet episode with a jsonl fallback), pairing the Nav2/teleop command as `action` with the executed velocity and locomotion state as `observation.state`. Covered by pytest and a CI-safe round-trip check (`tools/check_lerobot_export.py`).
- Grew the footstep planner from a flat-ground stub into a terrain-aware planner: added a `TerrainModel` (stacked axis-aligned keep-out and curb boxes over flat ground) and a `FootstepPlanner::plan_over_terrain` that nudges feet laterally out of keep-out zones, places them on raised patches, lifts the swing apex to clear step-ups, and reports blocked steps when no foothold is found. Wired it into the `footstep_marker_publisher` (new `no_step_zone`/`curb_box` params, amber-for-nudged / red-for-blocked markers), covered by gtest and an end-to-end check (`tools/check_footstep_terrain_e2e.py`).
- Wired the BehaviorTree.CPP recovery tree into a live `locomotion_ros2_bt_recovery_node`: added a ROS-integrated `ClearWalkingFaultService` BT action that actually calls `/locomotion_ros2/clear_fault`, a recovery node that subscribes to `/locomotion_ros2/state`, ticks the tree, and drives recovery (spinning on a background executor so the service call does not deadlock the tick loop), and a launch file. Fixed the matching runtime/adapter semantics so recovery is real: the mock adapter's `clear_fault` now re-enables the driver (clears the estop latch) and the runtime enforces the operator-estop interlock (a fault may not be cleared while the runtime estop is still engaged). Covered by gtest and an end-to-end check that proves the runtime stays faulted on its own and only the BT clears it (`tools/check_bt_recovery_e2e.py`).
- Gave the Unitree SDK2 (G1) adapter a real vendor-SDK link path. Introduced a `UnitreeLocoBackend` dispatch boundary so all hardware calls live behind one interface: a `SilLocoBackend` (always built, records what would be sent, unit-tested and verified wired through the adapter) and an `Sdk2LocoBackend` (`src/sdk2_loco_backend.cpp`, compiled only with `-DLOCOMOTION_ROS2_WITH_UNITREE_SDK2=ON`) that drives the G1 `LocoClient` (`Move`/`BalanceStand`/`Damp` over the DDS channel). Wired the CMake option to a real `find_package(unitree_sdk2 REQUIRED)` + link that fails loudly when the SDK is absent (so an ON build never silently degrades), removed the in-source `#ifdef`s from the adapter, and documented the `unitree_sdk2_DIR`/`CMAKE_PREFIX_PATH` setup.
- Extended the LeRobot exporter to collect multiple runtime traces into one multi-episode dataset: added `write_episodes_dataset` (one episode per trace) that de-duplicates tasks into a shared task table, keeps the global frame `index` continuous across episodes, shards episodes into `chunk-XYZ` directories, and computes `stats.json` over every frame; `write_dataset` is now a single-episode wrapper and the CLI accepts several trace paths. Covered by added pytest cases and a multi-episode round-trip in `tools/check_lerobot_export.py`.
- Fed the terrain-aware footstep planner from a real cost/elevation source
  instead of hand-authored boxes. `TerrainModel` gained an embedded `TerrainGrid`
  so keep-out and height queries can be answered from a map grid (O(1) cell
  lookups), alongside the existing boxes. Added `occupancy_terrain`, which builds
  that grid from a Nav2-style `nav_msgs/OccupancyGrid` costmap (cells at or above
  a configurable `occupied_threshold` become keep-out footholds; `-1` unknowns
  optionally block) plus an optional second OccupancyGrid read as a coarse
  elevation field (`metres = value * elevation_height_per_unit`) for step-ups.
  `footstep_marker_publisher` now subscribes to `costmap_topic` / `elevation_topic`
  (transient-local), plans in the costmap frame so footholds align with the map
  cells, and still honours the hand-authored `no_step_zone` / `curb_box` params.
  Covered by `test_terrain_model` grid cases, a new `test_occupancy_terrain`
  (OccupancyGrid → terrain → planner dodges a costmap obstacle), and
  `tools/check_footstep_costmap_e2e.py` (a live OccupancyGrid nudging a real
  published footstep plan, frame and all).
- Added `experiments/gait_lab`, a physics-driven testbed for comparing walking
  gait algorithms on a real MuJoCo Unitree G1 (position actuators + `mj_step`,
  not kinematic playback). A single `GaitController` interface
  (`update(obs, cmd) -> ctrl[29]`) lets gait algorithms be plugged in and scored
  on the same robot with the same metrics (forward distance, survival time,
  lateral drift, min torso height). Ships three algorithms — `stand-hold`
  (stable baseline), `open-loop-cpg` (sinusoidal stepping with no feedback, the
  honest failure that topples in ~1 s), and `balanced-cpg` (stepping + lateral
  weight-shift + torso-attitude ankle feedback, which survives ~3× longer and
  makes net forward progress) — plus `run_compare.py` (metrics table, optional
  per-algorithm GIF/JSON) and a skip-if-unavailable pytest suite asserting the
  comparison invariants. Kept under `experiments/` so the hardware-free ROS 2
  build never depends on MuJoCo or a model checkout.
- Added a second *class* of gait algorithm to `experiments/gait_lab`: a
  model-based `capture-point` walker alongside the CPG controllers. It models the
  G1 as a linear inverted pendulum, commits each footstep at the instantaneous
  capture point (`xi = x_com + v_com/omega`, laterally to catch the fall plus a
  forward speed term), and realises both feet with a new Jacobian damped-least-
  squares leg IK (`G1Model.solve_leg_ik`, driving the `left_foot`/`right_foot`
  sites through the 6-DOF leg chains). It walks the farthest and straightest of
  all algorithms (~0.6 m, lateral drift ~0.04 m vs the open-loop CPG's ~0.17 m)
  but is the least durable stepper — a deliberate, honest "farthest walker vs.
  most stable" tradeoff the testbed surfaces, motivating a learned/optimisation
  gait behind the same interface. Covered by added IK and comparison pytest
  cases.
- Added an optimisation-based gait to `experiments/gait_lab`. `optimize.py` runs
  a Cross-Entropy Method over a controller's `TUNABLES` parameter space, scoring
  each candidate by a physics rollout and warm-starting at the hand-tuned
  defaults. The parameters it found for the capture-point walker are baked into
  a new `OptimizedCapturePoint` controller (same algorithm, same
  `GaitController` interface — only the constants come from optimisation rather
  than by hand), which walks ~2× farther than the hand-tuned version (1.25 m vs
  0.61 m). Demonstrates that an optimisation-based gait beats hand-tuning on the
  objective it was given (distance), and that a learned policy would plug into
  the same interface. Covered by a pytest case asserting the optimised gait
  out-walks the hand-tuned one.
- Added the most principled model-based gait to `experiments/gait_lab`: a
  `zmp-preview` walker using Kajita preview control. New `gait_lab.zmp_preview`
  designs the cart-table LIPM preview gains via the discrete algebraic Riccati
  equation (SciPy) so a future ZMP reference produces a smooth CoM trajectory
  whose induced ZMP tracks — and leads — the footstep plan. `ZMPPreviewWalk`
  lays down a forward-marching footstep schedule, preview-tracks it into a CoM
  trajectory, and realises both feet relative to that planned CoM via the leg IK
  (so commanding the feet drives the pelvis along the planned sway), with ankle
  attitude feedback on top. It is the best all-rounder among the steppers — it
  walks farther than `balanced-cpg` and survives longer than the reactive
  `capture-point` (0.66 m, 2.4 s) because planning ahead sways the CoM over the
  next stance foot before the step. SciPy-gated: the controller and its tests
  skip cleanly where SciPy is absent, and `run_compare.py` skips it rather than
  aborting. Covered by offline preview-tracking and in-physics pytest cases.
- Probed whether optimisation can close the gait_lab "farthest walker vs. most
  stable" gap. `optimize.py` gained an `--objective` switch (`distance` vs.
  `balanced`, the latter rewarding distance scaled by the fraction of the horizon
  survived, so only ground covered *without* falling counts). The honest finding,
  now documented in the README: no — not by tuning a reactive gait. Whatever the
  objective, the best `capture-point` parameters top out around a ~1.5 s survival
  ceiling (distance-optimised → 1.25 m/1.3 s; sustained-optimised → ~1.3 m/1.5 s),
  a structural limit of one-step-lookahead foot placement rather than a tuning
  problem. Optimisation reliably improves the axis it is rewarded on, but closing
  the gap needs a better algorithm (the `zmp-preview` look-ahead already survives
  longer) or a learned policy behind the same interface. Added a pure-function
  pytest case asserting each objective prefers the gait good on its own axis.
- Visualised the gait_lab comparison. `render_montage.py` rolls out every
  algorithm with rendering and tiles the frames into one image (a row per
  algorithm, time left→right, a colour swatch per row), making the qualitative
  difference obvious at a glance: `stand-hold` stays upright, the steppers topple
  at different rates, `optimized-cp` walks farthest, `zmp-preview` stays balanced.
  Because the MuJoCo environments ship no image encoder, added `gait_lab.pngio`,
  a dependency-free PNG writer (stdlib `zlib`/`struct` only), and embedded the
  generated `assets/gait_comparison.png` in the README with a colour legend.
  Covered by a PNG round-trip pytest case.
- Added a learned gait to `experiments/gait_lab`: `learned-feedback` keeps a CPG
  feedforward but replaces the hand-tuned ankle gains with a learned linear
  feedback policy (`residual = W @ observation` over torso roll/pitch/rates and
  CoM velocity), `W` trained by the Cross-Entropy Method (`train_policy.py`). It
  walks ~2.7× farther and far straighter than the hand-tuned `balanced-cpg`
  (0.74 m vs 0.27 m, drift 0.10 m) but does not out-survive it — learning the
  feedback buys distance, not a broken balance ceiling (the same farthest-vs-
  stable through-line). The training is deliberately *robust*: each candidate is
  scored on the worst of several perturbed initial states, because a falling
  humanoid is chaotic and a naive single-rollout search overfits — a first run
  found a "3.4 s" policy that collapsed to 1.8 s under mere 4-decimal weight
  rounding. Added `G1Model.perturb` / a `perturb_seed` rollout option for
  robustness testing, and pytest cases asserting the learned gait out-walks
  hand-tuning, is reproducible, and stays robust under perturbation. The chaotic-
  overfit lesson and the gait-class balance ceiling are documented in the README.
- Captured multi-episode LeRobot datasets from live runtime runs and confirmed
  HuggingFace `datasets.load_dataset` compatibility. Added
  `tools/capture_lerobot_episodes.py`, which brings up the mock runtime and
  drives several distinct semantic-action episodes (walk_forward / turn_left /
  sidestep_left / ...), recording each through the live `locomotion_ros2_demo_recorder`
  over real ROS topics (cmd_vel bridge → runtime → safety pipeline → adapter) and
  aggregating them into one LeRobot v2.1 dataset via the existing exporter. Added
  `tools/check_lerobot_hf_load.py` and skip-if-unavailable `locomotion_ros2_examples`
  pytest cases that prove the export round-trips through HuggingFace `datasets`:
  the parquet episode files load via `load_dataset("parquet", ...)` with row
  count, columns, and `observation.state`/`action` widths matching
  `meta/info.json`, and the `meta/tasks.jsonl` / `meta/episodes.jsonl` tables load
  via `load_dataset("json", ...)`. Verified end-to-end with three live episodes
  (129 frames, three distinct task labels) loading cleanly through HuggingFace.
- Embedded the locomotion_ros2 recovery into a real Nav2 `bt_navigator` recovery
  branch. Added a Nav2-loadable BT plugin library `locomotion_ros2_nav2_bt_nodes`
  exporting `IsWalkingReady` (a topic condition that reads `/locomotion_ros2/state`
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
  `locomotion_ros2_nav2_recovery_harness` ticks the recovery branch through the real
  `nav2_behavior_tree::BehaviorTreeEngine`, exercised end-to-end against the live
  runtime by `tools/check_nav2_bt_recovery_e2e.py` (runtime stays faulted alone,
  the Nav2-loaded branch clears it); and `tools/check_nav2_recovery_tree.py`
  statically guards the droppable navigate tree. The operator-estop interlock
  still holds through the Nav2 path.
- Added a second real-robot adapter, `locomotion_ros2_unitree_go2`, giving the adapter hub breadth beyond humanoids: a Unitree Go2 quadruped sport-mode adapter (`UnitreeGo2Adapter`) that reuses the dispatch-backend pattern with a `Go2SportBackend` (always-built software-in-the-loop `SilSportBackend`, plus an `Sdk2SportBackend` behind `-DLOCOMOTION_ROS2_WITH_UNITREE_SDK2=ON` driving the Go2 `SportClient` `Move`/`Euler`/`BodyHeight`/`RecoveryStand`/`StandDown`/`Damp`). The model is genuinely quadruped: it rests lying down (`STATE_SITTING`), stands up on activate and lies back down on deactivate, self-rights into balance-stand via recovery-stand, sits on a quick stop, reports four-foot `SUPPORT_QUADRUPED`, and tilts its torso via Euler angles. The vendor link is gated behind a real `find_package(unitree_sdk2 REQUIRED)` that fails loudly when the SDK is absent. Covered by gtest and an end-to-end runtime-load check (`tools/check_unitree_go2_adapter_e2e.py`) that loads the plugin, autostarts to a quadruped stand, and trots via the real `ExecuteVelocity` action.
- Broke the gait_lab stability ceiling with reinforcement learning. First made
  the ceiling explicit: `stability_ceiling.py` sweeps each gait *class* over its
  own parameters and shows that the best-tuned hand/model-based gait tops out at
  ~2.85 s (no setting walks the full 8 s horizon) — a position-controlled
  humanoid in single support is a laterally-unstable inverted pendulum, and
  reactive position feedback cannot arrest the fall regardless of gains. Then
  added `rl-residual`, the first gait in the lab to break it: the same
  `BalancedCPG` rhythm `learned-feedback` uses, but the correction is a neural
  policy (a two-hidden-layer MLP) trained by PPO instead of a linear CEM map. New
  modules: `gait_lab/rl_env.py` (a framework-free residual-gait env — BalancedCPG
  feedforward + a learned position-target residual on the 12 leg actuators, 50 Hz
  control decimation, survival-dominated reward with upright-gated forward
  progress), `gait_lab/ppo.py` (a self-contained PPO — actor-critic MLPs,
  GAE, clipped objective, obs normaliser; no SB3/RLlib), `train_rl.py` (PPO
  training with 12 parallel CPU rollout workers feeding one GPU learner — pin
  `OMP_NUM_THREADS=1` per worker or thread oversubscription is ~8× slower),
  `eval_policy.py` (robust evaluation across perturbed starts), and the
  `RLResidualWalk` controller (dependency-free numpy inference from an exported
  `gait_lab/rl_policy.npz`, registered in `CONTROLLERS`). The trained policy is
  the only gait besides `stand-hold` to survive the full 8 s horizon (`[ok]`,
  `minH` 0.77 m — never near a fall) while walking forward (0.73 m); evaluated
  from a nominal start plus eight perturbed ones, 8/9 reach the full horizon
  (mean 7.55 s). Two lessons baked in: robustness must be in the objective
  (saving on the worst of several perturbed starts + domain randomisation, or the
  `learned-feedback` chaos trap recurs one level up — a policy that aces one lucky
  rollout is a fluke), and inference must match the training control rate (a
  silent 50 Hz-train / 500 Hz-infer mismatch turned a robust full-horizon walker
  into a 5 s faller until `RLResidualWalk` decimated identically). Covered by
  added pytest cases (GAE correctness, the env's zero-residual reproduces
  balanced-cpg, and `rl-residual` breaks the ceiling — survives the full horizon
  and walks forward, deterministically and torch-free). README documents the
  ceiling, the RL result, and both lessons; the 8-row comparison montage is
  regenerated with `rl-residual` as the only gait still walking at 6 s.
- Brought the gait_lab RL gait into the runtime as a software-in-the-loop
  adapter (experiment → product). New package `locomotion_ros2_gait_lab_sil` adds
  `GaitLabSilAdapter`, a `locomotion_ros2_core::WalkingAdapter` pluginlib plugin that
  drives a MuJoCo Unitree G1 through a gait_lab controller (default: the
  reinforcement-learned `rl-residual` policy) behind the real runtime + safety
  pipeline. To keep MuJoCo an optional dependency of one node (never of
  locomotion_ros2), it is a thin ROS bridge — no MuJoCo/Python build dependency: the
  adapter forwards the runtime's safety-filtered velocity + lifecycle to a
  companion Python sim (`locomotion_ros2_examples/scripts/gait_lab_sil_sim.py`, rclpy
  + MuJoCo + gait_lab) and reports the simulated robot's WalkingState back. The
  adapter owns a small internal node drained non-blockingly from `read_state()`
  via `spin_some` (no background thread); all command gating / state fusion lives
  in the ROS-free `GaitLabSilModel` (8 gtest cases). Added a
  `gait_lab_sil_runtime.launch.py` (runtime + cmd_vel bridge + sim) and an
  end-to-end check (`tools/check_gait_lab_sil_e2e.py`) that brings up the runtime
  with the adapter + sim, drives an `ExecuteVelocity` goal through the safety
  pipeline, and confirms the RL gait walks the full command without falling.
  Three integration bugs found and fixed along the way: the initial "activate"
  was missed by the late-loading sim until the control topic was made latched
  (transient-local); numpy bools leaked into the WalkingState message until cast
  to native bool; and the runtime sends a velocity once and expects persistence,
  so the sim must hold the commanded velocity (the stale-command watchdog belongs
  to the runtime's safety pipeline, not the sim) — re-expiring it mid-walk
  snapped the robot to a stand pose and toppled it.
- Verified the Nav2 velocity path drives the gait_lab SIL robot. The SIL launch
  already wires the legged `cmd_vel_bridge` (Nav2's shaper + readiness gate), so a
  plain `/cmd_vel` Twist — exactly what Nav2's controller server emits — is shaped
  to the legged envelope, ready-gated on `/locomotion_ros2/state`, and fed through the
  runtime's safety pipeline into the SIL adapter. Added
  `tools/check_gait_lab_sil_nav2_e2e.py`, which streams a Nav2-style `/cmd_vel` and
  confirms the reinforcement-learned gait walks under it without falling
  (end-to-end through bridge → runtime → safety → SIL adapter → MuJoCo G1).
- Added a push-recovery benchmark to gait_lab and a hero filmstrip for the SIL
  adapter. `G1Model.push` applies a horizontal base velocity kick; `GaitHarness`
  and `rl_env` can shove the robot on a seeded mid-rollout schedule, and
  `eval_policy.py --push-speeds` reports survival per shove magnitude (plus a new
  `--policy` to benchmark any exported policy). `render_rl_walk.py` writes a
  dependency-free filmstrip of `rl-residual` walking the full horizon, now the
  SIL package's hero image. Honest negative result, documented in the README:
  training for push recovery (`train_rl.py --push-interval/--push-speed`, with
  shoves folded into the robust worst-of-perturbations eval) did not beat the
  nominal policy's incidental shove tolerance and regressed locomotion (the
  policy drifted backward to balance in place) — a fixed CPG rhythm plus a small
  position residual is too thin a substrate for genuine push recovery, so the
  failed weights are not shipped (the nominal forward-walking policy stays the
  default) and only the benchmark + training hooks are kept. Push-determinism
  pytest added; suite at 20 gait_lab tests.
- Investigated a **steerable** gait_lab gait (track forward speed + yaw rate, so
  Nav2 can drive the SIL G1) and mapped a substrate ladder, each rung tested:
  (1) command-conditioned RL on the CPG substrate (`SteerableCPG`/`RLSteerableWalk`,
  `train_rl.py --steerable`) becomes robust-but-spiralling — survives every command
  but cannot turn, because a leg residual on a fixed sinusoid has no lever on foot
  placement; (2) a footstep substrate (`SteerableFootstepGait`: capture-point in a
  rotating heading frame) *does* steer — a yaw command genuinely curves the
  footsteps and the learned yaw-tracking error drops; (3) but kinematic footstep
  walkers top out near a ~2 s balance ceiling, so the residual cannot carry the
  steering base the full horizon (`RLSteerableFootstepWalk`,
  `train_rl.py --steerable --footstep`). Honest conclusion: clean steering needs
  foot placement **and** force-aware (ZMP/torque) balance — beyond position
  control, the same frontier as push recovery. Added `eval_steerable.py`, three
  RL-plumbing fixes in `train_rl.py` (double-applied normaliser epsilon,
  swamped/under-counted warm-start normaliser, critic-warmup so a random critic
  cannot wreck a warm-started actor) plus a cross-dimension warm-start, and tests
  pinning both the negative (CPG can't steer) and the positive (footstep does);
  suite at 25 gait_lab tests.
- Added a software-in-the-loop **Nav2 navigation** stack for the gait_lab SIL G1:
  `gait_lab_sil_nav2.launch.py` brings up map server + NavFn planner + Regulated
  Pure Pursuit controller + behaviour tree + lifecycle over the runtime/safety
  pipeline and the MuJoCo sim (with a static identity `map->odom`, since the sim
  publishes true `odom` + TF). The sim node now publishes `nav_msgs/Odometry` and
  the `odom->base_link` transform. Added an arena map + `params/nav2_sil.yaml`,
  taught `cmd_vel_bridge` to accept a `TwistStamped` input (Nav2 Jazzy publishes
  stamped `cmd_vel`), and an end-to-end goal-navigation check
  (`tools/check_gait_lab_sil_nav2_nav_e2e.py`). The full stack plans and the drive
  chain is verified end-to-end; reaching arbitrary goals is gated on a steering
  gait (above).
- Added a **live ROS-driven SIL filmstrip**: the sim node can capture rendered
  frames and write a rolling filmstrip, and `tools/capture_sil_live.py` drives the
  robot through a command sequence over the real runtime + safety + bridge path
  and saves the SIL G1 as actually driven through ROS, not a harness rollout.
- Started the **force-control** rung the steering / push-recovery frontier needs:
  `G1Model.set_torque_mode` switches chosen actuators from position servos to
  torque (motor) mode in place (reversible), so a controller can command joint
  *torques* (ground-reaction / ZMP balance) the position gait structurally cannot
  — covered by `test_torque_mode_actuates_by_force`. `force_balance.py` is the
  honest baseline on top of it, comparing three strategies under the same shove:
  a stiff position stand, an ankle-strategy torque, and a whole-body CoM
  controller (every leg joint in torque mode, a restoring CoM force mapped through
  the CoM Jacobian, gravity-compensated each step via `mj_inverse`). **Neither**
  force strategy beats the stiff position stand for a *standing* shove — two
  instructive reasons: stiffness is itself resistance, and without the *contact*
  Jacobian the unconstrained CoM Jacobian barely couples leg torque to CoM motion
  (the CoM gain has ~no effect). The payoff is dynamic — a contact-constrained
  whole-body controller (force-distribution QP) plus a capture *step*, which
  position control cannot decide to take — the documented next rung, now that
  torque actuation is in place.
- Implemented the proper **contact-Jacobian whole-body controller**
  (`run_wbc_contact` in `force_balance.py`): a restoring CoM force split across
  the feet and mapped to joint torques through each foot's *contact* (site)
  Jacobian, gravity-compensated each step via `mj_inverse`. The finding sharpens
  and becomes multi-method: **none** of the torque-mode standing strategies —
  ankle, CoM-Jacobian, or the proper contact-Jacobian WBC — beats the stiff
  position stand for a *standing* shove, because standing favours stiffness (the
  500-gain servo's feedback is very effective) and an open-loop gravity
  feedforward drifts (it does not even hold the stand without high-gain posture
  feedback that just recreates the servo). Force at the feet pays off in *motion*,
  not in standing — the working balance win is the capture step, and a contact-WBC's
  place is regulating a moving CoM/ZMP inside a walker.
- **Push recovery that works — a capture step.** Following the force-balance
  finding (in-place strategies cannot beat a stiff stand; the missing ingredient
  is the decision to *step*, not torque), `capture_step.py` holds a normal stand
  until a shove drives the capture point `xi = com + com_vel/omega` outside the
  feet, then takes one (or, on re-trigger, several) step(s): it swings the foot on
  the falling side to the reach-clamped capture point via the existing leg IK
  while the stance foot holds. A forward shove that topples the static
  position stand at ~2.1 s is recovered to the full horizon by the capture step --
  the honest, working rung of push recovery (the earlier CPG-residual push-training
  result was negative), and the reactive-footstep substrate a force-aware gait
  would build on. Covered by `test_capture_step_recovers_a_forward_push`; suite at
  27 gait_lab tests.
- Added `SteerableZMPWalk`: steering on the most balance-aware base. It subclasses
  the `zmp-preview` walker (Kajita preview control — the CoM trajectory leads the
  ZMP, the most stable model-based gait here) and makes the footstep schedule
  *curve*: each step advances by `forward_speed*step_duration` along a heading that
  rotates by `yaw_rate*step_duration`, plant offset half a stance-width across it,
  with the preview-tracked CoM swaying along the arc. It walks forward (+0.7 m) on
  a more stable base than the kinematic `steerable-footstep`, but — like every
  position-controlled footstep walker — still tops out near the ~2 s ceiling, so
  the curve does not fully develop before it topples: clean full-horizon steering
  remains the contact-constrained-WBC frontier. Covered by
  `test_steerable_zmp_plan_curves_and_walks`; suite at 28 gait_lab tests.
- **Capstone — the synthesis attempt and the full map.** `ReactiveSteerableWalk`
  unifies the two half-working ideas: every footstep target is the capture point
  (balance, like `capture_step.py`) plus a commanded forward offset along a heading
  that rotates with the yaw command (steering) — one reactive foot-placement lever.
  Honest verdict (`test_reactive_steerable_walks_but_does_not_break_the_ceiling`):
  it walks and responds to the command but is *less* stable than the open-loop
  `steerable-zmp` (~1.2 s) — continuous reactive capture-stepping destabilises a
  walk even though it rescues a stand. This closes the sweep: across CPG-residual,
  capture-point, ZMP-preview, and this reactive synthesis, **no position-controlled
  kinematic steerable walker reaches the full horizon**. Added a "full map" section
  to the gait_lab README tying the whole investigation together (steering needs
  foot placement; kinematic footsteps cap at ~1.5–2.5 s; the working balance win is
  a capture step, not torque-mode standing; the frontier is a contact-constrained
  torque WBC regulating a moving CoM/ZMP while stepping). Suite at 29 gait_lab tests.
- Ran the frontier controller to its root: `force_walk.py` builds the proper
  force-aware ZMP walker — leg torques from `mj_inverse` gravity compensation + a
  posture task tracking the ZMP-preview IK pose + a contact-Jacobian CoM task — and
  it still **loses** to the position-IK `zmp-preview` (~1.3 s vs ~2.4 s). The cause
  is the *substrate*, not the controller: on a model built for position control,
  the high-gain servos the solver applies implicitly/continuously/exactly are not
  beatable by explicit torque control with a feedforward gravity term, which drifts
  and caps near ~1.3 s whatever the gains (standing or walking). The WBC is correct;
  genuine force-aware walking needs a *torque-native* model, not the position-servo
  menagerie G1 — the boundary is the model, not the controller. Covered by
  `test_force_walk_torque_wbc_runs`; suite at 30 gait_lab tests.
- Added the textbook bipedal **hybrid** WBC (`run_force_walk_hybrid`): position-IK
  swing leg for precise foot placement, torque-stance leg for the force/balance WBC,
  modes switching at each strike — and **corrected an over-claim**. A well-tuned
  torque WBC actually *holds a stand* ~3 s (the earlier "~1.3 s standing" was
  under-tuned); the real limit is *walking* — tracking the moving footstep
  trajectory with torque tops out ~1.3 s for both the all-torque and the proper
  hybrid controller, below the ~2.4 s position-IK walk, because the implicit
  high-gain servo tracks the fast swing exactly where explicit torque does not (and
  the CoM task barely couples through the brief single-support contact). Conclusion
  unchanged — position IK wins on this position-built model — but precisely scoped:
  the limit is the substrate *plus* a hand-tuned (non-QP) controller; genuine
  force-aware walking wants a torque-native model and a contact-QP WBC.
- **Built the contact-QP WBC** that conclusion asked for: `wbc_qp.py` implements
  proper task-space inverse dynamics (TSID), solving the joint accelerations *and*
  the per-foot-corner ground-reaction forces together each step under the friction
  cone and unilateral contact (CoM/posture/swing tasks in a weighted-least-squares
  objective, GRF a genuine decision variable rather than a hand-split assumption).
  The honest result closes the loop rather than breaking the wall: the QP **holds a
  quiet stand indefinitely** with real friction-cone GRF (a posture task is the
  stable backbone; a moderate-weight CoM task adds force authority without fighting
  the rigid double-support constraints), but it still does **not** beat position
  control — under a 0.6 m/s shove it goes **infeasible** the instant the capture
  point leaves the support polygon (measured ~5 cm past the toe), and walking it tops
  out below the position-IK `zmp-preview`. That infeasibility is the most valuable
  output: the controller itself *certifies* that no force in the friction cone can
  recover without a step — the rigorous statement of "step, don't push." Building the
  textbook QP confirms the boundary is *standing-without-stepping plus a position-
  built model*, not the absence of a QP; the move that beats the limit remains the
  capture step, taken exactly when the QP says you must. Tested by
  `test_qp_wbc_holds_a_stand_but_certifies_the_support_polygon_limit` (uses a fresh
  `G1Model`); suite at 31 gait_lab tests. Needs a QP solver (`qpsolvers`/`osqp`).
- Tried the **culmination** (`wbc_qp.py::run_qp_capture_step`): force-aware QP balance
  that hands off to a capture step on the QP's own feasibility certificate (infeasible
  / capture point out of support) — the step trigger is the QP's boundary, not a
  hand-tuned threshold. Honest **null result**: it extends survival over the bare QP
  (0.6 m/s forward ~0.5 s → ~1.3 s) but does NOT beat the stiff stand or a standalone
  capture step — the QP's compliance lets a hard shove develop before the step fires,
  and the G1's large feet make a stiff forward stand very push-robust already (rides
  out 0.4 m/s ~2.3 s). Stepping clearly wins only for gentle pushes (~0.3 m/s → full
  horizon) the QP could also just absorb. The stiff stand wins yet again — the
  through-line of the whole map.
- **Fixed a silent lab-wide bug the culmination surfaced**: the capture-point velocity
  term was always zero. `data.subtree_linvel` (whole-body CoM velocity) is only filled
  by MuJoCo when a subtree-velocity sensor exists, and the menagerie G1 has none — so
  `observe().com_vel_xy`, and every capture point `xi = com + com_vel/omega` built on
  it, used a zero velocity (the capture step was carried by ankle feedback, not a real
  step). New `G1Model.com_velocity_xy()` computes it correctly as `J_com · qvel`;
  `capture_step.py` and the culmination now use it, so the capture step **genuinely
  steps** and recovers gentle shoves (0.3 m/s: a 2.6 s stand → the full horizon). The
  RL policies keep reading the old (zero) field they were trained against, so their
  golden tests are untouched. Tested by `test_com_velocity_xy_is_real_not_silently_zero`
  and `test_qp_capture_step_steps_and_beats_bare_qp_but_not_the_stiff_stand`; suite at
  33 gait_lab tests.
- **Made the contact-QP WBC torque-honest (the complete TSID), settling the
  "torque-native model" frontier.** The notes conjectured the stiff position servo
  only wins by applying unbounded force — but the menagerie G1 already ships real joint
  torque limits (`jnt_actfrcrange`: ankle ±50, knee/hip-roll ±139 Nm) that MuJoCo
  enforces on every actuator, and under a 0.6 m/s shove the servo uses at most ~40 % of
  any budget, never saturating: it wins by being *gentle*, not by cheating. The real
  gap was the controller — the friction-cone-only QP planned ankle torques up to 383 %
  of the limit (56 steps before it gave up), which MuJoCo silently clamped, so the
  "proper TSID" was never dynamically consistent under load. `WBCSolver(tau_limits=True)`
  adds the joint torque limits as two-sided inequalities on
  `τ = (M q̈ + h − Jᶜᵀf)[actuated]`; `run_qp_torque_audit` shows the peak demand drop
  from 383 % to a clean 100 % cap (steps-over-limit 56 → 0) at no cost to survival — a
  quiet stand needs only ~45 % of the budget. Correctness improved; the verdict is
  unchanged, because the binding constraint under a shove is the support polygon, not
  the torque budget. Tested by
  `test_complete_tsid_is_torque_honest_but_the_wall_is_unchanged`; suite at 34 gait_lab
  tests.
- **Found and paid the last idealisation: the stiff servo's standing-balance win was a
  MuJoCo implicit-damping artifact (`motor_model.py`).** The lab's through-line — "a
  stiff 500-gain position servo beats the force controllers" — rested on the servo being
  a MuJoCo *implicit* actuator: MuJoCo integrates its velocity-damping term `−kd q̇`
  implicitly (its Euler step always does), an unconditionally-stable inner velocity loop
  at the full 500 Hz that no finite-rate digital servo and no explicit-torque QP enjoys.
  `motor_model.py` re-implements **both** controllers as explicit torque through one
  shared `MotorModel` (control-rate ZOH + first-order bandwidth lag + the real
  `jnt_actfrcrange` clamp); the servo torque `kp(q_des−q) − kd q̇` is bit-identical to
  MuJoCo's position actuator (verified to 1e-13), so only the damping integration
  changes. Result: the explicit servo **cannot hold even a quiet stand** (topples
  ~1.3 s, unchanged from 200 Hz down to 50 Hz — not a control-rate effect), while
  routing `−kd q̇` back through implicit joint damping recovers most of the hold
  (~2.8 s); `localize_servo_idealisation` pins the crutch to the velocity term. The
  model-based complete-TSID QP, on the identical explicit-torque footing, **does** hold
  the quiet stand the servo cannot, and degrades gracefully with control rate (holds at
  ≥200 Hz). So the standing-balance half of the verdict **flips**: a model-based torque
  controller is the better stand-keeper on honest footing. The push-recovery half is
  untouched — under a 0.6 m/s shove both still fail at ~0.6 s and the QP certifies *must
  step*: removing the servo's free lunch buys standing balance, not push recovery, and
  the shove wall is still the support polygon. Tested by
  `test_motor_model_zoh_and_bandwidth_are_honest`,
  `test_stiff_servos_standing_win_was_an_implicit_damping_idealisation`, and
  `test_under_a_real_shove_the_support_polygon_wall_is_unchanged`; suite at 37 gait_lab
  tests.
- **Turned the same lens on the lab's central conclusion — walking — and it survives
  (`motor_model.py::run_motor_zmp_walk`).** "Position-IK `zmp-preview` beats the torque
  WBC while walking" used the same implicit servo as its winning baseline, so it gets
  the same audit: re-run both controllers as explicit torque on the ZMP-preview plan.
  Unlike standing, the verdict does **not** flip. The position servo loses ~a third of
  its survival to the idealisation (implicit-IK walk ~2.15 s → honest explicit torque
  ~1.45 s) but still beats the QP walk (~0.6 s) by a wide margin at every control rate.
  The asymmetry is the finding: standing balance is won by the idealised inner damping
  loop (so removing it flips the result), but walking is won by genuinely tracking the
  fast swing trajectory — a real control-authority advantage the high-gain position
  servo has and the compliant torque WBC does not, which survives the honest test. The
  standing claim needed a crutch; the walking claim never did. Tested by
  `test_paying_the_idealisation_does_not_flip_the_walking_verdict`; suite at 38 gait_lab
  tests.
- **Mapped the push-recovery showdown's thesis into a hard number — the push-robustness
  frontier (`push_frontier.py`, `render_frontier.py`).** The showdown is one shove from
  one direction; this measures the whole map. For each controller and every shove
  direction, a binary search finds the largest base-velocity kick (m/s) it survives for
  the full horizon — a *robustness polygon* in velocity space, rendered as a polar hero
  (`assets/push_frontier.png`). The shape is the recovery anisotropy: both the stiff
  stand and the capture step bulge sideways (the G1's wide stance) and pinch fore-aft
  (the narrow ankle-pitch axis). The capture step encloses the stiff stand with **+55 %
  recoverable area** (0.61 vs 0.39 (m/s)²) — but the two **tie at the backward worst
  case** (~0.2 m/s): stepping widens the frontier where it can reach and honestly does
  not where it can't. The contact-QP WBC collapses to a point at the origin — it holds a
  quiet stand but goes infeasible under *any* shove, certifying "must step", so its
  in-place frontier has zero width. Tested by
  `test_push_frontier_search_and_area_are_exact`,
  `test_capture_step_widens_the_forward_push_frontier_but_not_the_backward`, and
  `test_contact_qp_push_frontier_collapses_to_a_must_step_certificate`; suite at 41
  gait_lab tests.
- **Added a DCM (divergent-component-of-motion) step-adjustment walker — the
  closed-loop model-based baseline the other steppers leave out (`DCMWalk`).**
  `capture-point` reasons about the foothold only at each foot strike and `zmp-preview`
  plans the CoM trajectory open-loop; `dcm-walk` does both — a nominal footstep plan
  (so it bootstraps and marches straight) plus a DCM correction recomputed *every
  control tick*, placing the next foot proportional to the CoM velocity error
  `k·(v - v_nom)/omega`. It walks the second-farthest of all the steppers (0.81 m,
  behind only the CEM-optimised `optimized-cp`) and very straight (drift 0.06 m). **The
  honest result is a null one:** on this position-controlled G1 (no CoP authority within
  a step) the closed loop buys no survival — every footstep walker topples from its own
  dynamics in ~1-1.3 s, and the *open-loop* `zmp-preview` actually outlives it (2.42 s);
  the full DCM law needs force authority to track a within-step CoP, which position
  servos don't give. The lab's recurring ceiling, found from a new direction. Registered
  in the zoo (9 controllers now, 3×3 hero grids regenerated); tested by
  `test_dcm_step_adjustment_reacts_to_forward_velocity_error` and
  `test_dcm_walk_walks_well_but_its_closed_loop_does_not_break_the_ceiling`; suite at 43
  gait_lab tests.
- **Implemented a 2024 method that ships no public code — adaptive step *duration* on
  restricted footholds (`adaptive_step.py`, `render_stepping_stones.py`).** A faithful,
  dependency-light port of "Adaptive Step Duration for Accurate Foot Placement"
  (arXiv:2403.17136), which uses the commercial FORCES PRO solver and releases no
  implementation. It reimplements the discrete step-to-step DCM map and the
  receding-horizon program over footstep location *and* step timing with SciPy SLSQP
  (the bilinear `e^{lam T}` coupling is a small NLP). The headline result, quantified:
  on irregular stepping stones a fixed cadence can hit the footholds but lets the DCM
  diverge (viability error mean 11.6, max 47 — it would topple), while **adapting the
  step duration keeps the DCM viable (mean 0.26, a 44× smaller error)** and still hits
  every stone — choosing 0.51 s for a long gap, 0.25 s for a short one. Honest coda:
  the closed-loop realisation on the position-controlled G1 (`run_adaptive_walk`) walks
  off the mark but topples at ~1 s like every footstep walker here — the planner's
  viability edge needs within-step CoP authority to cash, the lab's recurring ceiling.
  Tested by `test_adaptive_step_planner_recovers_a_clean_nominal_gait`,
  `test_adaptive_step_timing_keeps_dcm_viable_where_fixed_cadence_diverges`, and
  `test_adaptive_walk_realises_the_plan_but_hits_the_position_control_ceiling`; suite at
  46 gait_lab tests.
- **Added a survival-time curve that un-flattens the push-robustness frontier, and put
  the force+step synthesis on it (`push_frontier.py --curve`, `render_survival_curve.py`,
  the `qp-capture-step` frontier controller).** The robustness *polygon* scores survival
  as a binary — did you reach the full horizon? — collapsing both the contact-QP WBC and
  the QP-balance-then-capture-step synthesis (`wbc_qp.run_qp_capture_step`) onto the same
  `r=0`. The curve sweeps raw time-to-fall vs shove magnitude with the horizon drawn as a
  *recovery ceiling*, separating **recovering** from merely **delaying** a fall. The
  measured forward result: the immediate **capture step** recovers a shove up to 0.35 m/s
  (the stiff stand to 0.25); neither force-aware controller recovers *any* nonzero shove,
  but feeding the QP's must-step certificate to a late capture step (`qp-capture-step`)
  **roughly doubles** the bare QP's time-to-fall (~1.2 s plateau vs ~0.55 s) — real,
  measurable value the binary polygon hid at `r=0`, and still short of recovery because
  the late step starts from a drifted compliant-balance state. Force authority delays the
  fall; only an immediate step reaches the ceiling. Tested by
  `test_detail_time_parses_the_survival_time_from_every_adapter_format` and
  `test_survival_curve_separates_recovering_from_merely_delaying`; suite at 48 gait_lab
  tests.
- **Predicted the lab's recurring ~1 s collapse from first principles
  (`fall_time_theory.py`, `render_fall_theory.py`).** The "balance is lost in about a
  second, for every controller" finding repeated across a dozen experiments becomes a
  prediction from the LIPM's two quantities. *Claim 1 — the push frontier is geometry*:
  the max in-place-recoverable kick is `v* = d·ω` (support margin × `ω=√(g/z)`), which
  matches the measured frontier within ~5 % **laterally (0.56 vs 0.57 m/s) and backward
  (0.20 vs 0.20)** where the support polygon binds, and **over-predicts forward (0.44 vs
  0.28)** where ankle-pitch torque binds instead — the gap localises which limit is
  active (corroborating `wbc_qp.py` Experiment 4's torque budget). *Claim 2 — the fall
  clock is leg length, not the controller*: the measured stiff-stand fall time asymptotes
  to **~2/ω ≈ 0.53 s** under hard shoves with the free-inverted-pendulum topple a strict
  lower bound beneath it, so the universal **~1 s ceiling is a few multiples of `1/ω =
  √(z/g) ≈ 0.27 s`** — which is why force-vs-position control never moved the wall. Tested
  by `test_capturability_predicts_the_frontier_anisotropy_from_geometry` and
  `test_free_inverted_pendulum_topple_is_the_two_over_omega_floor_and_a_lower_bound`;
  suite at 50 gait_lab tests.
- **Tested the capturability theory on terrain by tilting gravity into a slope
  (`terrain_frontier.py`, `render_terrain_frontier.py`).** Walking an incline of angle
  `α` is equivalent to rotating `model.opt.gravity` (a downhill `g·sinα` plus a reduced
  normal `g·cosα`), so the same shove experiments run on a slope with no model surgery —
  the cleanest test of the flat-ground theory in a regime it was not fitted to. Results,
  honest including where the clean theory bends: the frontier **shifts uphill** as
  predicted (downhill recoverable kick collapses toward zero, uphill grows, reversing the
  flat forward>backward order by ~3°); the **critical slope is torque-limited** — the
  stand self-holds only to **~4.7°**, far below the geometric `arctan(d_fwd/z) ≈ 9.6°`,
  the same ankle-torque limit the fall-time theory found forward; the uphill kick grows
  *more* than the static margin predicts because gravity also dynamically decelerates an
  uphill shove (a partial, honest validation of `v*=d·ω`); and the lab's through-line
  holds — **stepping extends the limit** (critical slope ~4.7° → ~5.9°, downhill kick
  roughly doubled) until even a step lands on ground that keeps falling away (~8°). Tested
  by `test_terrain_critical_slope_is_torque_limited_and_stepping_extends_it` and
  `test_slope_biases_the_capturability_frontier_uphill`; suite at 52 gait_lab tests.
- **B3 first rung: ros2_control-split gait_lab SIL path.** Added
  `g1_sil_ros2_control.urdf` with `joint_state_topic_hardware_interface`,
  `gait_lab_sil_gait_controller.py` for policy-only control,
  `ros2_control_split` mode on `gait_lab_sil_sim.py`,
  `gait_lab_sil_ros2_control_runtime.launch.py`, and
  `tools/check_gait_lab_sil_ros2_control_e2e.py`. Split-mode stability now
  mirrors monolithic timing via per-tick `physics_snapshot` sync, shadow-model
  substeps, and batched joint commands; `check_gait_lab_sil_ros2_control_e2e.py`
  passes with `rl-residual`. The legacy monolithic sim launch is unchanged.
- **B3 deep: ros2_control forward path wiring.** Added
  `GaitLabSilJointForwardController`, `use_ros2_control_forward:=true` launch
  arg, hardware `write()` relay, delayed sim/policy start, and E2E `--forward`
  flag. Default launch still uses the direct `joint_commands` path (E2E verified).
- **B3 deep: forward path 500 Hz E2E.** Split controller configs (50 Hz direct,
  500 Hz forward), `relay_commands` URDF param on `GaitLabSilTopicSystem`,
  `g1_sil_ros2_control_forward.urdf`, latched `physics_snapshot` QoS to break
  the policy/sim bootstrap deadlock, and per-cycle hardware relay so MuJoCo steps
  even when stand targets match the reported state. Both
  `check_gait_lab_sil_ros2_control_e2e.py` and `--forward` pass with
  `rl-residual`.
- **B2 first rung: rl-steerable on ros2_control SIL.** Extended
  `check_gait_lab_sil_ros2_control_e2e.py` with `--steer-loose` (yaw velocity
  goal on the embedded relay path with `rl-steerable`).
- **B2 quantitative rung: signed yaw gate.** `--steer` requires embedded
  `rl-steerable` to walk an 8 s ``ExecuteVelocity`` (0.2 m/s, 0.25 rad/s yaw)
  with ≥0.20 rad signed heading change, ≥0.25 m travel, no fall during the
  command, and logs a tracking ratio vs the commanded yaw.
- **B3 deep first rung: embedded C++ RL policy.** Added `GaitLabRlPolicy` (npz
  loader + MLP), `GaitLabSilRlResidualController`, `use_embedded_rl_policy:=true`
  launch mode, and gtest coverage. Python gait node publishes CPG feedforward and
  policy observations; inference runs in the ros2_control plugin.
- **B4 first rung: full Nav2 autonomous SIL navigation.** Extended
  `gait_lab_sil_nav2.launch.py` with optional ros2_control embedded stack (50 Hz
  nav config, delayed Nav2 bringup, SIL `require_ready:=false` on the legged bridge),
  continuous odom/TF on the split sim path, and `RLSteerableWalk` in `CONTROLLERS()`.
  `tools/check_gait_lab_sil_nav2_nav_e2e.py` sends `NavigateToPose` after cmd_vel
  prime and lifecycle settle; monolithic `rl-steerable` reaches the goal. Embedded
  path fixed by publishing obs/ff from the sim snapshot only (no local `mj_step`).
- **B4 hardening.** Nav2 E2E gates on `fell_before_reach` (ignores late falls after
  the goal), keeps monitoring odometry after BT `Goal failed`, cmd_vel prime, and
  relaxed progress checker; defaults `goal-x=2.0`, `tolerance=0.8`.
- **B4 embedded 0.8 m.** Fixed embedded gait-controller policy decimation (advance
  `feedforward_and_observation` once per MuJoCo substep so C++ RL runs at 50 Hz,
  not 5 Hz). Tightened Nav2 yaw/forward caps for `rl-steerable` spiral control;
  `--embedded` Nav2 E2E reaches 2 m at 0.8 m tolerance.
- **B2 flake reduction.** Split gait controller ramps commanded yaw per MuJoCo
  substep for `rl-steerable*` (mirrors harness stability); `--steer` E2E primes
  straight walking, retries up to three times with `/locomotion_ros2/clear_fault`,
  and uses gentler arc commands (0.15 m/s, 0.20 rad/s yaw).
- **B4 Nav2 flake reduction.** Monolithic `gait_lab_sil_sim` ramps `rl-steerable*`
  yaw per MuJoCo substep; SIL Nav2 launch caps bridge `max_yaw_accel` at 0.20 rad/s².
  Full Nav2 E2E retries navigation twice with `clear_fault` and a straight cmd_vel prime.
- **B4 spiral reduction.** Added `legged.yaw_deadband` to the Nav2 cmd_vel bridge
  shaper (gtest-covered), capped lateral velocity on the SIL Nav2 launch, registered
  `RLSteerableFootstepWalk` for optional `--controller`, and log `max_lateral` in
  the Nav2 E2E checker.
- **Added fall-detected and recovery-blocked visuals to the MuJoCo G1 gait surface.**
  `fall_detected` holds the fallen placeholder pose with FAULT overlay semantics;
  `recovery_blocked` shows a frozen mid-recovery attempt while a walk command is
  blocked by the safety gate. Both appear in the showcase sequence, gait gallery,
  and demo-evidence traceability table.
- **Added `slow_careful_walk` to the MuJoCo Unitree G1 demo surface.** Short-stride
  cautious forward stepping with arms held forward, semantic-action and `/cmd_vel`
  mapping (`x=0.10`), showcase sequence slot, VLA mapper aliases, gait gallery tile,
  and demo-evidence traceability row.
- **Refined the MuJoCo Unitree G1 `run_forward` gait for a clearer walk/run split.**
  Faster cadence (period 15 vs 30), higher forward speed, stronger knee lift and
  toe-off, a more natural forward lean with a subtler vertical bounce, and reduced
  arm/torso sway so running reads as running rather than sped-up walking. Updated
  the live demo, hero showcase renderer, and gait gallery assets; validated with
  `check_mujoco_g1_showcase_assets.py`, `check_readme_gallery_assets.py`, and the
  runtime showcase smoke path.
