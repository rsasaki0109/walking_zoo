# VLA Integration

VLA systems should not directly command joints, controllers, or vendor SDKs.
They should produce bounded semantic intent that walking_zoo can arbitrate and
send through safety gates.

```text
VLA / LLM Agent -> SemanticAction -> walking_zoo_vla
  -> Nav2 or WalkingRuntime -> SafetyPipeline -> Adapter -> Robot
```

Example semantic actions:

- `move_forward`
- `turn_left`
- `turn_right`
- `approach_object`
- `follow_person`
- `stop`

Future dataset work can export runtime logs and rosbag2 data toward formats
used by robot learning stacks such as LeRobot.
