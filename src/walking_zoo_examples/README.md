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

## Export a run to a LeRobot dataset

Turn a recorded runtime trace into a [LeRobot](https://github.com/huggingface/lerobot)
dataset so walking_zoo runs can feed imitation-learning pipelines:

```bash
ros2 run walking_zoo_examples walking_zoo_lerobot_export.py \
  /tmp/walking_zoo_mujoco_g1_runtime_showcase/demo_trace.json \
  --out /tmp/walking_zoo_lerobot --fps 10
```

The change-triggered event trace is resampled to a fixed-rate frame timeline and
written in the LeRobot v2.1 layout (`meta/info.json`, `meta/tasks.jsonl`,
`meta/episodes.jsonl`, `meta/stats.json`, and a parquet episode under
`data/chunk-000/`; a `.jsonl` episode is written if `pyarrow` is unavailable).

Pass several traces to collect them as multiple episodes in one dataset:

```bash
ros2 run walking_zoo_examples walking_zoo_lerobot_export.py \
  run_a.json run_b.json run_c.json \
  --out /tmp/walking_zoo_lerobot --fps 10
```

Each trace becomes one episode. Tasks are de-duplicated into a shared task
table, the global frame `index` is continuous across episodes, episodes are
sharded into `chunk-XYZ` directories, and `stats.json` covers every frame.

Frame mapping:

| LeRobot feature | Source | Vector |
| --- | --- | --- |
| `action` | `/cmd_vel` (Nav2/teleop command) | `[cmd_linear_x, cmd_linear_y, cmd_angular_z]` |
| `observation.state` | `/walking_zoo/cmd_vel` (executed) + `/walking_zoo/state` | `[exec_linear_x, exec_linear_y, exec_angular_z, locomotion_state, locomotion_mode, estop_active]` |
| `task` | most frequent `/walking_zoo/semantic_action`, else teleop | string |

The trace → dataset logic is pure Python and covered by
`walking_zoo_examples` pytest plus `tools/check_lerobot_export.py`.
