# Contributing

walking_zoo is built around a small promise: keep walking robot runtime
interfaces ROS2-native, safe by default, and independent of any single vendor
SDK.

Good first contributions:

- Add or improve a robot profile YAML.
- Add tests for safety gates or runtime state transitions.
- Improve mock demos and documentation.
- Add adapter skeletons that do not leak vendor SDK types.

Before opening a pull request:

```bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

Adapter contributions must keep real robot motion disabled by default and must
not bypass the safety pipeline.
