#!/usr/bin/env python3
import argparse


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate a conservative walking_zoo robot profile.")
    parser.add_argument("robot_model")
    parser.add_argument("--family", default="quadruped")
    parser.add_argument("--adapter", default="walking_zoo_mock_adapter/MockWalkingAdapter")
    args = parser.parse_args()

    print(f"""robot_model: {args.robot_model}
robot_family: {args.family}
adapter_plugin: {args.adapter}
capabilities:
  velocity_command: true
  body_pose_command: false
  footstep_plan: false
  whole_body_goal: false
  sit_stand: true
  estop: true
limits:
  max_linear_x: 0.3
  max_linear_y: 0.2
  max_angular_z: 0.5
  command_timeout_sec: 0.25
frames:
  base_frame: base_link
  odom_frame: odom
  map_frame: map
safety:
  allow_motion_default: false
  requires_explicit_allow_motion: true
""")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
