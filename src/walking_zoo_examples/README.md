# walking_zoo_examples

Run the mock runtime:

```bash
ros2 launch walking_zoo_bringup mock_runtime.launch.py
```

Send a low-speed command:

```bash
ros2 run walking_zoo_examples send_mock_cmd_vel.py
```

Trigger the emergency stop gate:

```bash
ros2 run walking_zoo_examples send_estop.py
```

Run the optional MuJoCo Unitree G1 gait demo:

```bash
colcon build --symlink-install
source install/setup.bash
python3 -m pip install -r tools/readme_gif_requirements.txt
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git /tmp/walking_zoo_mujoco_menagerie
ros2 launch walking_zoo_bringup mujoco_g1_gait_demo.launch.py
```

Switch gaits with semantic actions:

```bash
ros2 topic pub /walking_zoo/semantic_action walking_zoo_msgs/msg/SemanticAction "{action: 'run_forward'}" --once
ros2 topic pub /walking_zoo/semantic_action walking_zoo_msgs/msg/SemanticAction "{action: 'sidestep_left'}" --once
ros2 topic pub /walking_zoo/semantic_action walking_zoo_msgs/msg/SemanticAction "{action: 'turn_right'}" --once
```

The demo writes `/tmp/walking_zoo_mujoco_g1_demo/latest.png` and
`/tmp/walking_zoo_mujoco_g1_demo/live.gif`.

Run the automated gait showcase:

```bash
ros2 launch walking_zoo_bringup mujoco_g1_gait_showcase.launch.py
```

The showcase publishes semantic actions for walk, run, sidestep, turn,
stop, and e-stop. It writes `/tmp/walking_zoo_mujoco_g1_showcase/latest.png`
and `/tmp/walking_zoo_mujoco_g1_showcase/live.gif`.

Run the runtime trace showcase:

```bash
ros2 launch walking_zoo_bringup mujoco_g1_runtime_showcase.launch.py
python3 tools/check_demo_trace.py /tmp/walking_zoo_mujoco_g1_runtime_showcase/demo_trace.json --require-estop
```

This adds `demo_trace.json` and `demo_trace.md` next to the live MuJoCo images.
