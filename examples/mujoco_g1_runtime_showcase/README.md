# MuJoCo G1 Runtime Showcase

This example runs the Unitree G1 visualizer, the walking_zoo runtime, the
`/cmd_vel` bridge, the gait showcase driver, and the demo trace recorder.

## Run

```bash
colcon build --symlink-install
source install/setup.bash
python3 -m pip install -r tools/readme_gif_requirements.txt
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git /tmp/walking_zoo_mujoco_menagerie

ros2 launch walking_zoo_bringup mujoco_g1_runtime_showcase.launch.py
```

Validate the generated trace:

```bash
python3 tools/check_demo_trace.py /tmp/walking_zoo_mujoco_g1_runtime_showcase/demo_trace.json --require-estop
```

## Output

Default output directory:

```text
/tmp/walking_zoo_mujoco_g1_runtime_showcase
```

Generated files:

- `latest.png`: final rendered frame with runtime overlay.
- `live.gif`: lightweight animated GIF.
- `demo_trace.json`: machine-readable ROS2 topic trace.
- `demo_trace.md`: human-readable runtime timeline.

## What It Proves

The trace should show:

- `/cmd_vel` commands from the showcase driver.
- `/walking_zoo/cmd_vel` commands from the Nav2 bridge.
- `/walking_zoo/state` moving through `STANDING`, `WALKING`, `TURNING`, and
  `ESTOPPED`.
- `/walking_zoo/adapter_status` reporting the mock adapter state.
- `/walking_zoo/safety_state` reporting `ESTOPPED` after the e-stop.

## Troubleshooting

If ROS discovery or Fast DDS shared-memory ports are stale:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
```

If the G1 model is missing:

```bash
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git /tmp/walking_zoo_mujoco_menagerie
```

If MuJoCo cannot create an OpenGL context in a headless environment:

```bash
export MUJOCO_GL=egl
```

This example does not send commands to real hardware.
