# locomotion_ros2 Development Plan

locomotion_ros2 is ROS2-native locomotion for humanoid and legged robots, spanning
two halves: an honest, physics-based gait lab (`gait_lab`) where walking
controllers are developed and benchmarked — bad gaits actually fall over — and a
production-shaped ROS2 runtime that deploys a chosen gait safely across robot
SDKs ("Nav2 for walking robots"). The gait is the subject; the runtime is how it
reaches a robot.

This plan is intentionally practical. The immediate goal is to make the project
feel useful and exciting within the first minute on GitHub: a real ROS2 runtime,
real robot models rendered through existing simulators, visible walking/running
gaits, clear safety boundaries, and copy-paste demos that work without hardware.

## Product Direction

### North Star

A gait should be developed and proven honestly, then deployed to any walking
robot through a common ROS2 runtime — one path from algorithm to robot.

locomotion_ros2 should become both the honest gait lab and the runtime layer
between:

- Nav2 and walking robots.
- Teleoperation tools and walking robots.
- Learned policies and real robot SDKs.
- Future VLA systems and safe walking execution.
- Robot-specific SDKs and stable ROS2 applications.

### Positioning

locomotion_ros2 is:

- An honest gait lab: walking controllers benchmarked on real physics, with
  negatives reported (`gait_lab`).
- A ROS2-native walking runtime that deploys a chosen gait.
- An adapter hub for humanoid and legged robot SDKs.
- A safety-first command admission layer.
- A Nav2 companion for walking platforms.
- A future VLA command target.
- A place to normalize walking commands, states, profiles, diagnostics, and
  robot capability contracts.

locomotion_ros2 is not:

- A slide-deck gait collection — controllers must survive physics, or the lab
  reports that they don't.
- Its own simulator — `gait_lab` drives existing physics (MuJoCo) and model
  assets; the runtime is simulator-free by default.
- A toy visualization project.
- A vendor-specific Unitree wrapper.
- A shortcut around safety gates.

## Current Project State

The repository already has the important v0.1 foundation:

- `locomotion_ros2_msgs` defines walking-specific ROS2 messages, services, and
  actions.
- `locomotion_ros2_core` defines the adapter contract.
- `locomotion_ros2_safety` provides velocity limiting, watchdog, and e-stop gates.
- `locomotion_ros2_runtime` provides the lifecycle runtime manager.
- `locomotion_ros2_mock_adapter` gives an always-buildable adapter.
- `locomotion_ros2_nav2` bridges Nav2-style `/cmd_vel` into `/locomotion_ros2/cmd_vel`.
- `locomotion_ros2_bringup` provides one-command demos.
- README assets include MuJoCo Unitree G1 and PyBullet Laikago simulation GIFs.
- The MuJoCo G1 runtime showcase records JSON and Markdown trace evidence.

The next phase is not about adding many unrelated packages. The next phase is
about making the demo and runtime story much more convincing.

## Immediate Theme

Make humanoid walking and running look credible, reproducible, and tied to the
ROS2 runtime.

The README should not show toy GIFs. Every visual should come from an existing
simulator or existing robot model asset:

- MuJoCo for Unitree G1 humanoid showcases.
- PyBullet for lightweight quadruped runtime showcases.
- External robot model repositories such as `mujoco_menagerie` when possible.

The README should make the viewer think:

> This is already a real ROS2 runtime target, not a paper demo.

## Immediate Work Items

### 1. Improve The MuJoCo G1 Run

The current MuJoCo G1 visualizer should continue moving from simple kinematic
gait rendering toward a more credible runtime demo.

Target improvements:

- More natural forward lean during run.
- Clearer difference between walk and run cadence.
- Better knee lift and leg extension at toe-off.
- Less exaggerated arm swing.
- More stable torso orientation.
- Small vertical body motion that sells running without looking unstable.
- Runtime overlay that clearly shows the active semantic action and safety
  state.

Done means:

- `run_forward` looks meaningfully faster than `walk_forward`.
- The GIF does not look like walk playback at higher speed.
- The robot remains visually balanced.
- The generated README GIFs pass asset validation.
- The runtime showcase trace still proves `/cmd_vel`, `/locomotion_ros2/cmd_vel`,
  state publishing, and e-stop behavior.

### 2. Build A Rich Gait Gallery

The gait gallery is the most GitHub-visible feature. It should become a visual
index of the walking command surface that locomotion_ros2 normalizes.

Initial gallery entries:

- Stand.
- Walk forward.
- Run forward.
- Turn left.
- Turn right.
- Sidestep left.
- Sidestep right.
- Stop.
- E-stop.

Next gallery entries:

- Walk backward.
- Slow careful walk.
- Narrow-passage side-step.
- Turn-in-place with visible yaw.
- Body height adjustment.
- Body pitch/roll pose command.
- Fall detected placeholder state.
- Recovery blocked by safety gate.

Done means:

- README contains a compact, high-signal gait gallery GIF.
- Each visual maps to a real ROS2 command path or a planned message type.
- The gallery does not imply unsupported real robot capability.
- The generator script is deterministic enough for CI-style validation.

### 3. Make The Demo Evidence Stronger

The GIF should never stand alone. The project should show evidence that the GIF
is connected to runtime behavior.

Add or improve:

- `demo_trace.json` with topic samples.
- `demo_trace.md` with a readable timeline.
- Asset validation scripts for GIF dimensions, frame count, and size.
- Trace validation that checks:
  - `/cmd_vel` input.
  - `/locomotion_ros2/cmd_vel` bridge output.
  - `/locomotion_ros2/state` transitions.
  - `/locomotion_ros2/adapter_status`.
  - `/locomotion_ros2/safety_state`.
  - e-stop activation.

Done means:

- A skeptical ROS2 developer can reproduce the GIF and inspect the trace.
- README links to demo evidence instead of only showing images.
- The demo remains hardware-free and safe by default.

### 4. Keep Runtime Quality Ahead Of Visual Hype

Visuals bring attention, but the repository must remain a runtime project.

The runtime should keep improving in parallel:

- Better lifecycle logging.
- Clear active adapter state.
- Better command source tagging.
- Command arbitration tests for teleop/Nav2/VLA priority.
- More complete action cancellation behavior.
- More useful `/locomotion_ros2/state` transitions.
- Diagnostic messages that are readable in `ros2 topic echo`.

Done means:

- The demo visuals are backed by real runtime state.
- New GIF features do not bypass the safety pipeline.
- Commands still flow through the adapter contract.

## Milestones

### v0.1: Runtime Skeleton With Real Demo Feel

Goal:

Ship a buildable ROS2 workspace that proves the walking runtime shape.

Scope:

- ROS2 interfaces.
- Adapter contract.
- Safety pipeline.
- Lifecycle runtime manager.
- Mock adapter.
- Nav2 `/cmd_vel` bridge.
- MuJoCo G1 visual demo.
- PyBullet Laikago visual demo.
- README GIFs.
- Demo evidence traces.
- CI build and tests.

Exit criteria:

- `colcon build --symlink-install` passes.
- `colcon test` passes.
- Mock runtime E2E check passes.
- MuJoCo G1 showcase generates `latest.png`, `live.gif`, `demo_trace.json`,
  and `demo_trace.md`.
- README explains that generated visuals use existing simulators and are not
  toy animations.

### v0.2: Better Humanoid Runtime Surface

Goal:

Make humanoid command concepts visible without claiming full whole-body control.

Scope:

- More complete `ExecuteVelocity` behavior.
- Stub or partial `ExecuteBodyPose` action path.
- Body pose command visualization in MuJoCo G1.
- Footstep marker visualization, even before real footstep execution.
- Runtime state transitions for body pose and footstep execution modes.
- Better `SemanticAction` mapping for:
  - `move_forward`
  - `run_forward`
  - `turn_left`
  - `turn_right`
  - `sidestep_left`
  - `sidestep_right`
  - `stop`

Exit criteria:

- README shows multiple humanoid command modes.
- Docs clearly separate supported runtime behavior from planned robot control.
- Gait gallery becomes a reason to star the repository.

### v0.3: Unitree Adapter Readiness

Goal:

Make Unitree support credible while keeping the default build vendor-free.

Scope:

- Keep `LOCOMOTION_ROS2_WITH_UNITREE_SDK2` default `OFF`.
- Improve Unitree SDK2 stub diagnostics.
- Add explicit safety checklist for real robot testing.
- Validate Go2, G1, and H1 robot profile fields.
- Add adapter configuration examples.
- Document CycloneDDS and Unitree networking expectations without making them
  required for the default build.

Exit criteria:

- The repository still builds without Unitree SDK2.
- Unitree-specific logic does not leak into `locomotion_ros2_core` or
  `locomotion_ros2_msgs`.
- Real robot motion remains disabled unless explicitly enabled.

### v0.4: Footstep And Legged-Aware Navigation

Goal:

Move beyond `cmd_vel` while staying runtime-focused.

Scope:

- `ExecuteFootstepPlan` action behavior for mock/sim adapters.
- Footstep plan RViz markers.
- Step feasibility placeholder interface.
- Dynamic footprint and support polygon documentation.
- Nav2 BT nodes for walking readiness and fault handling.

Exit criteria:

- Users can see why `cmd_vel` is not enough for humanoids.
- Nav2 integration still remains simple for mobile-base compatibility.
- Footstep APIs are useful without forcing a custom planner.

### v0.5: VLA-Ready Runtime

Goal:

Make locomotion_ros2 a safe target for semantic robot commands.

Scope:

- Semantic action mapper node.
- Semantic action action server.
- Runtime policy that keeps VLA below safety and operator override priority.
- Dataset/log export design for future LeRobot-style workflows.
- Documentation for VLA systems as intent sources, not direct SDK controllers.

Exit criteria:

- README can honestly say locomotion_ros2 is VLA-ready at the runtime boundary.
- Semantic commands are traceable, cancellable, and safety-gated.

## README And Star Strategy

The README should sell the project in this order:

1. One-line identity.
2. Strong hero GIF from MuJoCo G1.
3. Proof link to demo evidence.
4. Visual tour of gait gallery.
5. Why `cmd_vel` is not enough.
6. Quick mock runtime demo.
7. Live MuJoCo G1 demo.
8. Nav2 integration.
9. Adapter contract.
10. Safety-first warning.
11. Supported/planned robot table.
12. Roadmap and contribution guide.

High-value README assets:

- Hero GIF: Unitree G1 moving through locomotion_ros2 runtime showcase.
- Runtime GIF: topic-driven G1 demo with overlay.
- Gait gallery GIF: walk, run, sidestep, turn, stop, e-stop.
- Safety GIF: e-stop blocks motion.
- Architecture diagram: Nav2/VLA/Teleop -> Runtime -> Safety -> Adapter.

Rules for README visuals:

- No toy GIFs.
- No fake robot silhouettes.
- Use existing simulators or existing robot model assets.
- Make the robot visible, centered, and inspectable.
- Do not over-darken or blur the robot.
- Include trace evidence when the GIF claims runtime behavior.

## Demo Architecture

### MuJoCo G1 Showcase

Expected stack:

```text
showcase driver
  -> /cmd_vel
  -> locomotion_ros2_nav2 bridge
  -> /locomotion_ros2/cmd_vel
  -> WalkingRuntimeManager
  -> SafetyPipeline
  -> MockWalkingAdapter
  -> /locomotion_ros2/state
  -> MuJoCo G1 visualizer
  -> latest.png / live.gif
  -> trace recorder
```

This keeps the visualizer honest: it reflects runtime state and semantic
actions instead of being only an offline animation.

### Offline README Rendering

Expected stack:

```text
MuJoCo model asset
  -> deterministic render script
  -> generated GIF / preview PNG
  -> asset validator
  -> README
```

This keeps README assets reproducible and independent of real hardware.

## Safety Plan

locomotion_ros2 should be boringly strict about motion safety:

- Real robot motion disabled by default.
- `allow_motion:=true` required for any real robot adapter.
- All commands pass through safety pipeline.
- E-stop blocks every motion command.
- Watchdog stops stale commands.
- Conservative default velocity limits.
- Runtime state should expose safety status clearly.
- VLA commands never outrank operator override or safety.

Upcoming safety improvements:

- Fall detector placeholder interface.
- Adapter health gate improvements.
- Command source audit trail.
- Per-profile velocity and body pose limits.
- Clearer fault/clear-fault state machine.

## Adapter Plan

Adapter contract rules:

- Vendor SDK types stay inside adapter packages.
- Robot capabilities live in profiles.
- Adapters report status even when disconnected.
- Real robot commands are opt-in.
- Mock and sim adapters are first-class test targets.

Priority adapters/profiles:

- Mock adapter: always buildable baseline.
- Unitree Go2: first real quadruped target.
- Unitree G1: humanoid visual and future runtime target.
- Unitree H1: humanoid profile validation target.
- Future: Digit, Figure, Booster, Fourier, ANYmal.

## Issue Backlog

Good first issues:

- Add a README frame contact sheet for the MuJoCo G1 gait gallery.
- Add `walk_backward` semantic action to the MuJoCo G1 visualizer.
- Add `sidestep_right` to the runtime showcase sequence.
- Add GIF metadata validation for frame count and dimensions.
- Add robot profile validation tests for Unitree G1 and H1.
- Add docs for command source priority.
- Add docs for why VLA commands must go through safety gates.
- Add RViz marker placeholders for footstep plans.

Medium issues:

- Improve `ExecuteVelocity` cancellation tests.
- Add command source tagging to runtime debug output.
- Add body pose command visualization in MuJoCo G1.
- Add trace validator checks for full state ordering.
- [x] Add Nav2 BT stub docs and sample recovery tree (superseded by the live
  `locomotion_ros2_bt_recovery_node` and `bt_xml/locomotion_ros2_recovery_live.xml`).
- Add runtime diagnostics publisher coverage.

Hard issues (completed in the deep-integration pass):

- [x] Optional Unitree SDK2 adapter implementation — SIL by default, with a real
  vendor-SDK link path behind a `UnitreeLocoBackend` dispatch boundary.
- [x] Footstep plan execution contract — `ExecuteFootstepPlan` action with a
  feasibility gate, plus a terrain-aware `FootstepPlanner` (keep-out avoidance,
  curb step-up).
- [x] BehaviorTree.CPP integration without making the default build fragile — a
  live `locomotion_ros2_bt_recovery_node` that calls `/locomotion_ros2/clear_fault`.
- [x] Runtime log export design for LeRobot-style datasets — single- and
  multi-episode exporter (`locomotion_ros2_lerobot_export.py`).
- [x] Legged-aware Nav2 integration beyond `cmd_vel` — `LeggedVelocityShaper`
  plus a readiness-gated bridge.

Each landed wired into the real runtime/pipeline, with gtest/pytest coverage and
an end-to-end check, not as an orphan utility.

## Next Phase

With the hard issues closed, the next phase widens the runtime rather than
deepening single features:

- [x] A second real adapter to give the "adapter hub" real breadth and validate
  the dispatch-backend pattern across robot classes. **Done:**
  `locomotion_ros2_unitree_go2` adds a Unitree Go2 quadruped sport-mode adapter
  (`UnitreeGo2Adapter`) reusing the `Go2SportBackend` SIL/SDK2 dispatch pattern,
  with a genuinely quadruped FSM (lie-down rest, recovery-stand, sit on quick
  stop, four-foot `SUPPORT_QUADRUPED`) and a real `find_package(unitree_sdk2
  REQUIRED)` link path. Covered by gtest and
  `tools/check_unitree_go2_adapter_e2e.py`.
- [x] Embed the `locomotion_ros2_bt` recovery nodes inside an actual Nav2 BT
  navigator recovery branch (not just the standalone recovery node). **Done:**
  `locomotion_ros2_nav2_bt_nodes` is a Nav2-loadable BT plugin library exporting
  `IsWalkingReady` (topic condition) and `ClearWalkingFault` (built on
  `nav2_behavior_tree::BtServiceNode`), wired into
  `bt_xml/navigate_to_pose_w_walking_recovery.xml` as the first action in the
  Nav2 `RoundRobin` recovery branch and enabled by overlaying
  `config/nav2_bt_navigator.yaml`. Verified by `test_nav2_bt_recovery_nodes`
  (loads the plugin the bt_navigator way), `tools/check_nav2_bt_recovery_e2e.py`
  (ticks the branch through the real `nav2_behavior_tree::BehaviorTreeEngine`
  against the live runtime), and `tools/check_nav2_recovery_tree.py` (static tree
  guard).
- [x] Terrain-aware footstep planning fed from a real elevation/cost source
  instead of hand-authored boxes. **Done:** `TerrainModel` now answers keep-out
  and height queries from an embedded grid sampled from a real map source.
  `occupancy_terrain` builds that grid from a Nav2-style `nav_msgs/OccupancyGrid`
  costmap (cells ≥ `occupied_threshold` become keep-out) plus an optional
  elevation grid (step-up heights). `footstep_marker_publisher` subscribes to a
  `costmap_topic` / `elevation_topic` and plans in the costmap frame. Covered by
  `test_terrain_model` (grid queries), `test_occupancy_terrain` (OccupancyGrid →
  terrain → planner), and `tools/check_footstep_costmap_e2e.py` (a live
  OccupancyGrid nudging a real published plan).
- [x] Capture multi-episode LeRobot datasets from live showcase runs and confirm
  HuggingFace `load_dataset` compatibility. **Done:**
  `tools/capture_lerobot_episodes.py` brings up the mock runtime and records
  several distinct semantic-action episodes through the real ROS pipeline
  (cmd_vel bridge → runtime → safety → adapter → recorder), aggregating them into
  one LeRobot v2.1 dataset. `tools/check_lerobot_hf_load.py` and a
  skip-if-unavailable `locomotion_ros2_examples` pytest confirm the parquet episodes
  and `meta/*.jsonl` tables load via HuggingFace `datasets.load_dataset` with row
  counts, columns, and feature widths matching `meta/info.json`.

## Definition Of Done For The Next Push

The next high-impact push should satisfy:

- MuJoCo G1 `run_forward` looks clearly better than before.
- README GIF assets are regenerated.
- `tools/check_mujoco_g1_showcase_assets.py` passes.
- `colcon build --symlink-install` passes.
- `colcon test` passes.
- Mock runtime E2E check passes.
- MuJoCo runtime showcase smoke test generates:
  - `latest.png`
  - `live.gif`
  - `demo_trace.json`
  - `demo_trace.md`
- Changelog mentions the visual/runtime improvement.

## Near-Term Execution Order

1. Finish and validate the improved MuJoCo G1 run gait.
2. Regenerate README GIFs from the updated simulator renderers.
3. Run asset validation, build, tests, and mock E2E.
4. Run MuJoCo runtime showcase smoke test.
5. Commit and push the run gait improvement.
6. Open issues for the next gait gallery expansion.
7. Add the next visible gait: `walk_backward` or `sidestep_right`.
8. Add a small README/demo evidence section for command-to-visual traceability.

The best next development loop is short and visual:

```text
add one gait or runtime state
  -> render it with MuJoCo
  -> validate the asset
  -> prove the ROS2 trace
  -> update README evidence
  -> push
```

That loop makes the repository more star-worthy without drifting away from the
core runtime mission.
