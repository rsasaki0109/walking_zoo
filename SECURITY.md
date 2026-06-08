# Security And Safety

locomotion_ros2 is robotics infrastructure. Treat unsafe motion behavior as a
security-sensitive issue.

Report privately if you find:

- A path that sends robot motion without `allow_motion:=true`.
- A command path that bypasses the safety pipeline.
- A vendor SDK adapter exposing credentials, network secrets, or unsafe default
  endpoints.
- A reproducible crash in runtime command handling.

Never test safety issues near people, stairs, traffic, or unstable hardware
setups.
