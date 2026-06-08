#!/usr/bin/env bash
set -euo pipefail

ros2 bag record \
  /locomotion_ros2/state \
  /locomotion_ros2/adapter_status \
  /locomotion_ros2/safety_state \
  /locomotion_ros2/cmd_vel
