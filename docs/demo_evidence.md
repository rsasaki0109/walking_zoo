# Demo Evidence

The MuJoCo G1 runtime showcase is the fastest way to verify that locomotion_ros2 is
more than a README GIF. It produces visual artifacts and a ROS2 runtime trace
from the same launch command.

## Run

```bash
colcon build --symlink-install
source install/setup.bash
python3 -m pip install -r tools/readme_gif_requirements.txt
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git /tmp/locomotion_ros2_mujoco_menagerie

ros2 launch locomotion_ros2_bringup mujoco_g1_runtime_showcase.launch.py
python3 tools/check_demo_trace.py /tmp/locomotion_ros2_mujoco_g1_runtime_showcase/demo_trace.json --require-estop
```

If the trace comes back empty (`events: 0`), the default ROS domain is usually
congested with stale DDS participants or shared-memory port locks from other
sessions. Re-run the launch on an unused `ROS_DOMAIN_ID`:

```bash
export ROS_DOMAIN_ID=77
ros2 launch locomotion_ros2_bringup mujoco_g1_runtime_showcase.launch.py
```

A clean domain gives Fast DDS a fresh shared-memory namespace and lets every node
discover each other. Cyclone DDS (`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`)
is an alternative transport if shared memory is unavailable on your machine.

## Artifacts

The default output directory is `/tmp/locomotion_ros2_mujoco_g1_runtime_showcase`.

| File | Purpose |
| --- | --- |
| `latest.png` | Last rendered MuJoCo frame with runtime overlay. |
| `live.gif` | Lightweight live GIF written by the visualizer. |
| `demo_trace.json` | Machine-readable ROS2 topic trace. |
| `demo_trace.md` | Human-readable runtime timeline. |

## Expected Timeline

The runtime showcase drives both semantic intent and `/cmd_vel` so the trace
shows the real runtime path:

```text
/locomotion_ros2/semantic_action  semantic -> walk_forward
/cmd_vel                      twist x=0.22 y=0.00 z=0.00
/locomotion_ros2/cmd_vel          twist x=0.22 y=0.00 z=0.00
/locomotion_ros2/state            walking state -> WALKING

/locomotion_ros2/semantic_action  semantic -> walk_backward
/cmd_vel                      twist x=-0.18 y=0.00 z=0.00
/locomotion_ros2/cmd_vel          twist x=-0.18 y=0.00 z=0.00
/locomotion_ros2/state            walking state -> WALKING

/locomotion_ros2/semantic_action  semantic -> turn_left
/cmd_vel                      twist x=0.00 y=0.00 z=0.55
/locomotion_ros2/cmd_vel          twist x=0.00 y=0.00 z=0.55
/locomotion_ros2/state            walking state -> TURNING

/locomotion_ros2/semantic_action  semantic -> estop
/locomotion_ros2/state            walking state -> ESTOPPED
/locomotion_ros2/adapter_status   adapter -> ESTOPPED
/locomotion_ros2/safety_state     safety -> ESTOPPED
```

## Command-to-Visual Traceability

Every gait in the showcase GIF maps to a concrete ROS2 command path, so a
skeptical viewer can match what the robot does on screen to a row of the trace.
The runtime collapses fine-grained intent into a small set of locomotion states,
while `/cmd_vel` and the visual gait carry the direction detail.

| Semantic action | `/cmd_vel` | `/locomotion_ros2/cmd_vel` (bridge) | Visual gait | Runtime state |
| --- | --- | --- | --- | --- |
| `walk_forward` | x=0.22 | x=0.22 | forward walk | WALKING |
| `run_forward` | x=0.45 | x=0.45 | forward run | WALKING |
| `walk_backward` | x=-0.18 | x=-0.18 | reverse walk | WALKING |
| `sidestep_left` | y=0.22 | y=0.22 | sidestep left | WALKING |
| `sidestep_right` | y=-0.22 | y=-0.22 | sidestep right | WALKING |
| `turn_left` | z=0.55 | z=0.55 | turn-in-place left | TURNING |
| `turn_right` | z=-0.55 | z=-0.55 | turn-in-place right | TURNING |
| `stop` | zero | zero | stand | STANDING |
| `estop` | (blocked) | (blocked) | estopped pose | ESTOPPED |

Each row is reproducible: the showcase publishes the semantic action and the
matching `/cmd_vel`, the Nav2 bridge republishes it as `/locomotion_ros2/cmd_vel`,
the runtime updates `/locomotion_ros2/state`, and the visualizer renders the gait.
The `estop` row is the safety proof: the command path is blocked rather than
forwarded to the adapter.

## Safety Proof

The e-stop is not just a visual state. The showcase calls the
`/locomotion_ros2/estop` service, the runtime safety gate becomes active, the mock
adapter reports `ESTOPPED`, and subsequent motion is blocked by the safety
pipeline.

The validator requires these conditions:

- Required topics are present:
  `/cmd_vel`, `/locomotion_ros2/cmd_vel`, `/locomotion_ros2/state`,
  `/locomotion_ros2/adapter_status`, `/locomotion_ros2/safety_state`,
  `/locomotion_ros2/semantic_action`.
- Runtime state reaches `WALKING`.
- With `--require-estop`, trace evidence includes e-stop behavior.

## Why This Matters

locomotion_ros2 is positioned as a ROS2-native Walking Runtime & Adapter Hub. This
demo proves the core loop in a reproducible way:

```text
semantic action / Nav2-style velocity
  -> locomotion_ros2 runtime topics
  -> safety state
  -> adapter status
  -> visible humanoid runtime target
  -> JSON/Markdown evidence artifact
```

The MuJoCo renderer is only a visual target for the demo. The runtime contract,
safety gate, adapter status, and trace are ROS2 artifacts.
