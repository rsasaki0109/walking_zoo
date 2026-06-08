# Mock Nav2 Demo

Start the mock runtime and bridge:

```bash
ros2 launch locomotion_ros2_bringup mock_runtime.launch.py
```

Publish a Nav2-style velocity command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.1}}" --once
```
