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
