# Robot Profiles

Robot profiles describe capabilities, limits, frames, and safety defaults.
They keep robot differences out of the stable runtime API.

Example:

```yaml
robot_model: unitree_go2
robot_family: quadruped
adapter_plugin: locomotion_ros2_unitree_sdk2/UnitreeSdk2Adapter
capabilities:
  velocity_command: true
  body_pose_command: true
  footstep_plan: false
limits:
  max_linear_x: 0.5
  max_linear_y: 0.3
  max_angular_z: 0.8
  command_timeout_sec: 0.25
safety:
  allow_motion_default: false
```

The runtime loads profile YAML when the `robot_profile` parameter points to a
file. ROS parameters can still provide defaults, but the profile file is the
source of truth for robot model, adapter plugin, capabilities, frames, and
limits. The runtime always overrides `real_robot_motion_allowed` from the
explicit `allow_motion` ROS parameter so real motion remains opt-in.
