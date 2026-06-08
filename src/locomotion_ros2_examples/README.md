# locomotion_ros2_examples

Run the mock runtime:

```bash
ros2 launch locomotion_ros2_bringup mock_runtime.launch.py
```

Send a low-speed command:

```bash
ros2 run locomotion_ros2_examples send_mock_cmd_vel.py
```

Trigger the emergency stop gate:

```bash
ros2 run locomotion_ros2_examples send_estop.py
```

Run the optional MuJoCo Unitree G1 gait demo:

```bash
colcon build --symlink-install
source install/setup.bash
python3 -m pip install -r tools/readme_gif_requirements.txt
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git /tmp/locomotion_ros2_mujoco_menagerie
ros2 launch locomotion_ros2_bringup mujoco_g1_gait_demo.launch.py
```

Switch gaits with semantic actions:

```bash
ros2 topic pub /locomotion_ros2/semantic_action locomotion_ros2_msgs/msg/SemanticAction "{action: 'run_forward'}" --once
ros2 topic pub /locomotion_ros2/semantic_action locomotion_ros2_msgs/msg/SemanticAction "{action: 'sidestep_left'}" --once
ros2 topic pub /locomotion_ros2/semantic_action locomotion_ros2_msgs/msg/SemanticAction "{action: 'turn_right'}" --once
```

The demo writes `/tmp/locomotion_ros2_mujoco_g1_demo/latest.png` and
`/tmp/locomotion_ros2_mujoco_g1_demo/live.gif`.

Run the automated gait showcase:

```bash
ros2 launch locomotion_ros2_bringup mujoco_g1_gait_showcase.launch.py
```

The showcase publishes semantic actions for walk, run, sidestep, turn,
stop, and e-stop. It writes `/tmp/locomotion_ros2_mujoco_g1_showcase/latest.png`
and `/tmp/locomotion_ros2_mujoco_g1_showcase/live.gif`.

Run the runtime trace showcase:

```bash
ros2 launch locomotion_ros2_bringup mujoco_g1_runtime_showcase.launch.py
python3 tools/check_demo_trace.py /tmp/locomotion_ros2_mujoco_g1_runtime_showcase/demo_trace.json --require-estop
```

This adds `demo_trace.json` and `demo_trace.md` next to the live MuJoCo images.

## Export a run to a LeRobot dataset

Turn a recorded runtime trace into a [LeRobot](https://github.com/huggingface/lerobot)
dataset so locomotion_ros2 runs can feed imitation-learning pipelines:

```bash
ros2 run locomotion_ros2_examples locomotion_ros2_lerobot_export.py \
  /tmp/locomotion_ros2_mujoco_g1_runtime_showcase/demo_trace.json \
  --out /tmp/locomotion_ros2_lerobot --fps 10
```

The change-triggered event trace is resampled to a fixed-rate frame timeline and
written in the LeRobot v2.1 layout (`meta/info.json`, `meta/tasks.jsonl`,
`meta/episodes.jsonl`, `meta/stats.json`, and a parquet episode under
`data/chunk-000/`; a `.jsonl` episode is written if `pyarrow` is unavailable).

Pass several traces to collect them as multiple episodes in one dataset:

```bash
ros2 run locomotion_ros2_examples locomotion_ros2_lerobot_export.py \
  run_a.json run_b.json run_c.json \
  --out /tmp/locomotion_ros2_lerobot --fps 10
```

Each trace becomes one episode. Tasks are de-duplicated into a shared task
table, the global frame `index` is continuous across episodes, episodes are
sharded into `chunk-XYZ` directories, and `stats.json` covers every frame.

### Capture multiple episodes from live runs

To build a multi-episode dataset straight from live runtime runs (instead of
hand-collecting trace files), use the capture tool. It brings up the mock
runtime, drives a distinct semantic-action episode for each requested episode
(recording each with the live `locomotion_ros2_demo_recorder`), and exports them all
into one LeRobot dataset:

```bash
python3 tools/capture_lerobot_episodes.py \
  --episodes 3 --out /tmp/locomotion_ros2_lerobot_live --episode-duration 3.0
```

The traces come from real ROS topics flowing through the cmd_vel bridge,
runtime, safety pipeline, and adapter — not synthetic data.

### HuggingFace `load_dataset` compatibility

The export is consumable by HuggingFace `datasets` (the common LeRobot entry
point that does not need the full `lerobot` package):

```python
from datasets import load_dataset
ds = load_dataset(
    "parquet",
    data_files="/tmp/locomotion_ros2_lerobot_live/data/chunk-000/episode_*.parquet",
    split="train",
)
print(ds, ds[0]["observation.state"], ds[0]["action"])
```

`tools/check_lerobot_hf_load.py` (and a skip-if-unavailable
`locomotion_ros2_examples` pytest) prove this round-trips: row count, columns, and
feature widths match `meta/info.json`, and the `meta/*.jsonl` tables load too.

Frame mapping:

| LeRobot feature | Source | Vector |
| --- | --- | --- |
| `action` | `/cmd_vel` (Nav2/teleop command) | `[cmd_linear_x, cmd_linear_y, cmd_angular_z]` |
| `observation.state` | `/locomotion_ros2/cmd_vel` (executed) + `/locomotion_ros2/state` | `[exec_linear_x, exec_linear_y, exec_angular_z, locomotion_state, locomotion_mode, estop_active]` |
| `task` | most frequent `/locomotion_ros2/semantic_action`, else teleop | string |

The trace → dataset logic is pure Python and covered by
`locomotion_ros2_examples` pytest plus `tools/check_lerobot_export.py`,
`tools/check_lerobot_hf_load.py`, and the live `tools/capture_lerobot_episodes.py`.
