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


def event_summaries(events):
    return [str(event.get("summary", "")) for event in events]


def find_state_transitions(summaries):
    prefix = "walking state -> "
    states = []
    for summary in summaries:
        if summary.startswith(prefix):
            states.append(summary[len(prefix):])
    return states


def print_report(trace_path, payload, topics, states, estop_confirmed):
    print(f"demo trace looks valid: {trace_path}")
    print(f"duration_sec: {payload.get('duration_sec', 0.0):.3f}")
    print(f"events: {len(payload.get('events', []))}")
    print("topics found:")
    for topic in sorted(topics):
        print(f"  - {topic}")
    print("state transitions found:")
    for state in states:
        print(f"  - {state}")
    print(f"estop confirmed: {'yes' if estop_confirmed else 'not required'}")


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

    summaries = event_summaries(events)
    states = find_state_transitions(summaries)
    summary_text = " ".join(summaries)
    if "WALKING" not in states:
        raise SystemExit("trace does not show a WALKING runtime state")

    estop_confirmed = False
    if args.require_estop:
        latest = json.dumps(payload.get("latest", {}), sort_keys=True)
        estop_confirmed = "estop" in summary_text.lower() or "ESTOPPED" in latest
        if not estop_confirmed:
            raise SystemExit("trace does not show estop behavior")

    print_report(trace_path, payload, topics, states, estop_confirmed)


if __name__ == "__main__":
    main()
