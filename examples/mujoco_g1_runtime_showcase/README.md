# MuJoCo G1 Runtime Showcase

This example runs the Unitree G1 visualizer, the locomotion_ros2 runtime, the
`/cmd_vel` bridge, the gait showcase driver, and the demo trace recorder.

## Run

```bash
colcon build --symlink-install
source install/setup.bash
python3 -m pip install -r tools/readme_gif_requirements.txt
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git /tmp/locomotion_ros2_mujoco_menagerie

ros2 launch locomotion_ros2_bringup mujoco_g1_runtime_showcase.launch.py
```

Validate the generated trace:

```bash
python3 tools/check_demo_trace.py /tmp/locomotion_ros2_mujoco_g1_runtime_showcase/demo_trace.json --require-estop
```

## Output

Default output directory:

```text
/tmp/locomotion_ros2_mujoco_g1_runtime_showcase
```

Generated files:

- `latest.png`: final rendered frame with runtime overlay.
- `live.gif`: lightweight animated GIF.
- `demo_trace.json`: machine-readable ROS2 topic trace.
- `demo_trace.md`: human-readable runtime timeline.

## What It Proves

The trace should show:

- `/cmd_vel` commands from the showcase driver.
- `/locomotion_ros2/cmd_vel` commands from the Nav2 bridge.
- `/locomotion_ros2/state` moving through `STANDING`, `WALKING`, `TURNING`, and
  `ESTOPPED`.
- `/locomotion_ros2/adapter_status` reporting the mock adapter state.
- `/locomotion_ros2/safety_state` reporting `ESTOPPED` after the e-stop.

## Troubleshooting

If the trace comes back empty (`events: 0`), the default ROS domain is congested
with stale DDS participants or shared-memory port locks. Re-run on an unused
domain so Fast DDS gets a fresh shared-memory namespace:

```bash
export ROS_DOMAIN_ID=77
```

Cyclone DDS is an alternative transport if shared memory is unavailable:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

If the G1 model is missing:

```bash
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git /tmp/locomotion_ros2_mujoco_menagerie
```

If MuJoCo cannot create an OpenGL context in a headless environment:

```bash
export MUJOCO_GL=egl
```

This example does not send commands to real hardware.
