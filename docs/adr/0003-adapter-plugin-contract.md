# ADR 0003: Adapter Plugin Contract

## Status

Accepted.

## Decision

Robot SDK integration happens behind `walking_zoo_core::WalkingAdapter`
pluginlib classes.

## Consequences

Vendor SDK types stay inside adapter packages. The runtime can load mock,
Unitree, or future robot adapters through one stable contract.
