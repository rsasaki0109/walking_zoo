# walking_zoo_unitree_sdk2

This package is an optional Unitree SDK2 adapter skeleton for walking_zoo.

It builds without the vendor SDK by default. Configure with
`-DWALKING_ZOO_WITH_UNITREE_SDK2=ON` only after installing and wiring the
vendor SDK in this package. Vendor SDK types must not appear in
`walking_zoo_core` or `walking_zoo_msgs`.

Real robot motion is disabled by default. Any future SDK-backed command path
must require `allow_motion:=true` and still pass through the walking_zoo safety
pipeline.
