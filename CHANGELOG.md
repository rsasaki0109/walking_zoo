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
