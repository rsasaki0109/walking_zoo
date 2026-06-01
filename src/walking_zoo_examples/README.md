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
