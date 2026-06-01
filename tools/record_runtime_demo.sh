#!/usr/bin/env bash
set -euo pipefail

ros2 bag record \
  /walking_zoo/state \
  /walking_zoo/adapter_status \
  /walking_zoo/safety_state \
  /walking_zoo/cmd_vel
