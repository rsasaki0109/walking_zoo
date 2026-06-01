# ADR 0002: ROS2-Native Interfaces

## Status

Accepted.

## Decision

walking_zoo uses ROS2 msg/srv/action, lifecycle nodes, and pluginlib as the
primary integration surface.

## Consequences

The runtime remains composable with Nav2, rosbag2, diagnostics, launch, and
standard ROS tooling. Custom RPC and vendor-specific public APIs are avoided.
