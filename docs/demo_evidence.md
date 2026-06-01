# Demo Evidence

The MuJoCo G1 runtime showcase is the fastest way to verify that walking_zoo is
more than a README GIF. It produces visual artifacts and a ROS2 runtime trace
from the same launch command.

## Run

```bash
colcon build --symlink-install
source install/setup.bash
python3 -m pip install -r tools/readme_gif_requirements.txt
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git /tmp/walking_zoo_mujoco_menagerie

ros2 launch walking_zoo_bringup mujoco_g1_runtime_showcase.launch.py
python3 tools/check_demo_trace.py /tmp/walking_zoo_mujoco_g1_runtime_showcase/demo_trace.json --require-estop
```

If Fast DDS shared-memory ports are stale on your machine, run the launch with
Cyclone DDS and a clean domain:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
```

## Artifacts

The default output directory is `/tmp/walking_zoo_mujoco_g1_runtime_showcase`.

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
/walking_zoo/semantic_action  semantic -> walk_forward
/cmd_vel                      twist x=0.22 y=0.00 z=0.00
/walking_zoo/cmd_vel          twist x=0.22 y=0.00 z=0.00
/walking_zoo/state            walking state -> WALKING

/walking_zoo/semantic_action  semantic -> turn_left
/cmd_vel                      twist x=0.00 y=0.00 z=0.55
/walking_zoo/cmd_vel          twist x=0.00 y=0.00 z=0.55
/walking_zoo/state            walking state -> TURNING

/walking_zoo/semantic_action  semantic -> estop
/walking_zoo/state            walking state -> ESTOPPED
/walking_zoo/adapter_status   adapter -> ESTOPPED
/walking_zoo/safety_state     safety -> ESTOPPED
```

## Safety Proof

The e-stop is not just a visual state. The showcase calls the
`/walking_zoo/estop` service, the runtime safety gate becomes active, the mock
adapter reports `ESTOPPED`, and subsequent motion is blocked by the safety
pipeline.

The validator requires these conditions:

- Required topics are present:
  `/cmd_vel`, `/walking_zoo/cmd_vel`, `/walking_zoo/state`,
  `/walking_zoo/adapter_status`, `/walking_zoo/safety_state`,
  `/walking_zoo/semantic_action`.
- Runtime state reaches `WALKING`.
- With `--require-estop`, trace evidence includes e-stop behavior.

## Why This Matters

walking_zoo is positioned as a ROS2-native Walking Runtime & Adapter Hub. This
demo proves the core loop in a reproducible way:

```text
semantic action / Nav2-style velocity
  -> walking_zoo runtime topics
  -> safety state
  -> adapter status
  -> visible humanoid runtime target
  -> JSON/Markdown evidence artifact
```

The MuJoCo renderer is only a visual target for the demo. The runtime contract,
safety gate, adapter status, and trace are ROS2 artifacts.
