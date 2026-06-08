#!/usr/bin/env python3
"""Dependency-free structural validator for the Nav2 walking-recovery BT tree.

The full navigate_to_pose tree cannot be instantiated offline (its Nav2 action
nodes wait for live action servers in their constructors), so this check guards
the droppable artifact statically: it confirms the tree is well-formed, targets
MainTree, and embeds the locomotion_ros2 recovery (IsWalkingReady + ClearWalkingFault,
the node IDs registered by locomotion_ros2_nav2_bt_nodes) inside the Nav2 RoundRobin
recovery branch with the expected ports wired.

This pairs with test_nav2_bt_recovery_nodes (the nodes are genuinely Nav2-loaded)
and check_nav2_bt_recovery_e2e.py (the loaded branch recovers a live runtime).
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def find_recovery_tree() -> Path:
    here = Path(__file__).resolve().parent.parent
    candidates = [
        here / "src/locomotion_ros2_bt/bt_xml/navigate_to_pose_w_walking_recovery.xml",
    ]
    for path in candidates:
        if path.is_file():
            return path
    print(f"recovery tree not found in: {[str(c) for c in candidates]}", file=sys.stderr)
    raise SystemExit(2)


def fail(message: str) -> None:
    print(f"nav2 recovery tree check FAILED: {message}", file=sys.stderr)
    raise SystemExit(1)


def main() -> int:
    path = find_recovery_tree()
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as error:
        fail(f"not well-formed XML: {error}")

    if root.tag != "root":
        fail(f"expected <root>, got <{root.tag}>")
    if root.get("BTCPP_format") != "4":
        fail("expected BTCPP_format=\"4\"")
    if root.get("main_tree_to_execute") != "MainTree":
        fail("expected main_tree_to_execute=\"MainTree\"")

    # The walking recovery must live inside the RoundRobin recovery action set,
    # not somewhere on the navigation happy path.
    round_robin = None
    for node in root.iter("RoundRobin"):
        if node.get("name") == "RecoveryActions":
            round_robin = node
            break
    if round_robin is None:
        fail("no <RoundRobin name=\"RecoveryActions\"> recovery branch found")

    walking_seq = None
    for child in round_robin:
        if child.tag == "Sequence" and child.get("name") == "WalkingFaultRecovery":
            walking_seq = child
            break
    if walking_seq is None:
        fail("WalkingFaultRecovery sequence is not a child of the recovery RoundRobin")

    # It must only fire when the robot is NOT ready (guarded by Inverter), then
    # call clear_fault. Verify both custom node IDs and their ports.
    inverter = walking_seq.find("Inverter")
    if inverter is None or inverter.find("IsWalkingReady") is None:
        fail("WalkingFaultRecovery must guard ClearWalkingFault with Inverter/IsWalkingReady")

    is_ready = inverter.find("IsWalkingReady")
    if is_ready.get("state_topic") != "/locomotion_ros2/state":
        fail("IsWalkingReady must monitor state_topic=\"/locomotion_ros2/state\"")

    clear = walking_seq.find("ClearWalkingFault")
    if clear is None:
        fail("WalkingFaultRecovery has no ClearWalkingFault action")
    if clear.get("service_name") != "/locomotion_ros2/clear_fault":
        fail("ClearWalkingFault must call service_name=\"/locomotion_ros2/clear_fault\"")

    # The generic Nav2 recoveries must still be present after the walking recovery.
    for required in ("Spin", "Wait", "BackUp"):
        if next(round_robin.iter(required), None) is None:
            fail(f"generic Nav2 recovery <{required}> missing from the recovery branch")

    print(f"nav2 recovery tree check passed: {path.name}")
    print("  - well-formed BTCPP v4 tree targeting MainTree")
    print("  - WalkingFaultRecovery embedded in the RoundRobin recovery branch")
    print("  - Inverter/IsWalkingReady guards ClearWalkingFault(/locomotion_ros2/clear_fault)")
    print("  - generic Nav2 recoveries (Spin/Wait/BackUp) preserved")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
