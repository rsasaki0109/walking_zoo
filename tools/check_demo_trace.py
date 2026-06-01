#!/usr/bin/env python3
"""Validate a walking_zoo demo trace JSON file."""

from pathlib import Path
import argparse
import json


REQUIRED_TOPICS = {
    "/cmd_vel",
    "/walking_zoo/cmd_vel",
    "/walking_zoo/state",
    "/walking_zoo/adapter_status",
    "/walking_zoo/safety_state",
    "/walking_zoo/semantic_action",
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "trace",
        nargs="?",
        default="/tmp/walking_zoo_mujoco_g1_runtime_showcase/demo_trace.json",
        help="Path to demo_trace.json.",
    )
    parser.add_argument("--require-estop", action="store_true")
    args = parser.parse_args()

    trace_path = Path(args.trace)
    if not trace_path.exists():
        raise SystemExit(f"missing trace: {trace_path}")
    payload = json.loads(trace_path.read_text(encoding="utf-8"))
    if payload.get("schema") != "walking_zoo.demo_trace.v1":
        raise SystemExit(f"unexpected schema in {trace_path}")

    events = payload.get("events", [])
    if len(events) < 4:
        raise SystemExit(f"trace has too few events: {len(events)}")

    topics = {event.get("topic") for event in events}
    missing = sorted(REQUIRED_TOPICS - topics)
    if missing:
        raise SystemExit(f"trace is missing topics: {', '.join(missing)}")

    summaries = " ".join(str(event.get("summary", "")) for event in events)
    if "walking state -> WALKING" not in summaries:
        raise SystemExit("trace does not show a WALKING runtime state")

    if args.require_estop:
        latest = json.dumps(payload.get("latest", {}), sort_keys=True)
        if "estop" not in summaries.lower() and "ESTOPPED" not in latest:
            raise SystemExit("trace does not show estop behavior")

    print(f"demo trace looks valid: {trace_path}")


if __name__ == "__main__":
    main()
