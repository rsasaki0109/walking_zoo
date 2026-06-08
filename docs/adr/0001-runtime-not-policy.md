# ADR 0001: Runtime, Not Policy Zoo

## Status

Accepted.

## Decision

locomotion_ros2 is a ROS2-native walking runtime and adapter hub. It does not
implement custom RL training, a simulator, MPC, WBC, or a new gait algorithm in
v0.1.

## Consequences

The project can focus on stable interfaces, safety gates, lifecycle behavior,
and adapter contracts. Learned policies and simulators can integrate through
ROS2 interfaces instead of being reimplemented here.
