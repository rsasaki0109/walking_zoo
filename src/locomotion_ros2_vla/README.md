# locomotion_ros2_vla

This package is a lightweight semantic action layer for future VLA integration.
It does not include a VLA model, ML runtime, or dataset dependency.

The intended flow is:

```text
VLA / LLM agent -> SemanticAction -> locomotion_ros2 runtime or Nav2 -> safety pipeline -> adapter
```

VLA systems should not directly command joints, vendor SDKs, or robot motion.

## Supported Semantic Actions

`SemanticActionMapper` translates a `SemanticAction` into a conservative
`TwistStamped` (or a stop request). Unrecognized actions are reported as
`recognized = false` so the runtime can ignore them safely.

| Action (aliases) | Mapped command |
| --- | --- |
| `move_forward` (`walk_forward`) | linear.x = 0.20 |
| `run_forward` | linear.x = 0.35 |
| `move_backward` (`walk_backward`) | linear.x = -0.15 |
| `sidestep_left` | linear.y = 0.20 |
| `sidestep_right` | linear.y = -0.20 |
| `turn_left` | angular.z = 0.30 |
| `turn_right` | angular.z = -0.30 |
| `stop` | stop request |

These velocities are intentionally conservative; the safety pipeline still
limits them and the e-stop gate can block them entirely.
