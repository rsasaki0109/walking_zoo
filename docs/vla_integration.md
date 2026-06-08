# VLA Integration

VLA systems should not directly command joints, controllers, or vendor SDKs.
They should produce bounded semantic intent that locomotion_ros2 can arbitrate and
send through safety gates.

```text
VLA / LLM Agent -> SemanticAction -> locomotion_ros2_vla
  -> Nav2 or WalkingRuntime -> SafetyPipeline -> Adapter -> Robot
```

Example semantic actions:

- `move_forward`
- `turn_left`
- `turn_right`
- `approach_object`
- `follow_person`
- `stop`

Runtime runs export to LeRobot datasets for robot-learning stacks. A live
capture tool (`tools/capture_lerobot_episodes.py`) drives the runtime through
several distinct semantic-action episodes, records each through the real ROS
pipeline, and aggregates them into one multi-episode LeRobot v2.1 dataset. The
export is confirmed loadable by HuggingFace `datasets.load_dataset` (the common
LeRobot entry point that does not require the full `lerobot` package), covered by
`tools/check_lerobot_hf_load.py` and a skip-if-unavailable pytest.
