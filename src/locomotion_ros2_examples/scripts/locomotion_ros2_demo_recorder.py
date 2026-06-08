#!/usr/bin/env python3
"""Record a compact runtime trace for locomotion_ros2 demos."""

from pathlib import Path
import json
import time

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from locomotion_ros2_msgs.msg import AdapterStatus, SafetyState, SemanticAction, WalkingState


WALKING_STATES = {
    WalkingState.STATE_UNKNOWN: "UNKNOWN",
    WalkingState.STATE_IDLE: "IDLE",
    WalkingState.STATE_STANDING: "STANDING",
    WalkingState.STATE_WALKING: "WALKING",
    WalkingState.STATE_TURNING: "TURNING",
    WalkingState.STATE_BODY_POSE_CONTROL: "BODY_POSE_CONTROL",
    WalkingState.STATE_EXECUTING_FOOTSTEPS: "EXECUTING_FOOTSTEPS",
    WalkingState.STATE_STOPPING: "STOPPING",
    WalkingState.STATE_SITTING: "SITTING",
    WalkingState.STATE_FALLEN: "FALLEN",
    WalkingState.STATE_FAULT: "FAULT",
    WalkingState.STATE_ESTOPPED: "ESTOPPED",
}

ADAPTER_STATES = {
    AdapterStatus.STATUS_UNKNOWN: "UNKNOWN",
    AdapterStatus.STATUS_DISCONNECTED: "DISCONNECTED",
    AdapterStatus.STATUS_CONNECTED: "CONNECTED",
    AdapterStatus.STATUS_ACTIVE: "ACTIVE",
    AdapterStatus.STATUS_FAULT: "FAULT",
    AdapterStatus.STATUS_ESTOPPED: "ESTOPPED",
}

SAFETY_STATES = {
    SafetyState.STATE_UNKNOWN: "UNKNOWN",
    SafetyState.STATE_OK: "OK",
    SafetyState.STATE_LIMITED: "LIMITED",
    SafetyState.STATE_BLOCKED: "BLOCKED",
    SafetyState.STATE_ESTOPPED: "ESTOPPED",
    SafetyState.STATE_FAULT: "FAULT",
}


class LocomotionRos2DemoRecorder(Node):
    def __init__(self):
        super().__init__("locomotion_ros2_demo_recorder")
        self.declare_parameter("output_dir", "/tmp/locomotion_ros2_mujoco_g1_showcase")
        self.declare_parameter("json_filename", "demo_trace.json")
        self.declare_parameter("markdown_filename", "demo_trace.md")
        self.declare_parameter("write_period_sec", 1.0)

        self.output_dir = Path(self.get_parameter("output_dir").value)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.json_path = self.output_dir / self.get_parameter("json_filename").value
        self.markdown_path = self.output_dir / self.get_parameter("markdown_filename").value
        self.start_time = time.monotonic()
        self.events = []
        self.latest = {
            "walking_state": None,
            "adapter_status": None,
            "safety_state": None,
            "cmd_vel": None,
            "locomotion_ros2_cmd_vel": None,
            "semantic_action": None,
        }
        self.last_state = None
        self.last_adapter = None
        self.last_safety = None

        self.create_subscription(WalkingState, "/locomotion_ros2/state", self.on_walking_state, 10)
        self.create_subscription(AdapterStatus, "/locomotion_ros2/adapter_status", self.on_adapter_status, 10)
        self.create_subscription(SafetyState, "/locomotion_ros2/safety_state", self.on_safety_state, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(TwistStamped, "/locomotion_ros2/cmd_vel", self.on_locomotion_ros2_cmd_vel, 10)
        self.create_subscription(SemanticAction, "/locomotion_ros2/semantic_action", self.on_semantic_action, 10)

        period = float(self.get_parameter("write_period_sec").value)
        self.timer = self.create_timer(max(period, 0.2), self.write_trace)
        self.get_logger().info(f"recording locomotion_ros2 demo trace to {self.output_dir}")

    def elapsed(self):
        return round(time.monotonic() - self.start_time, 3)

    def append_event(self, topic, summary, data):
        event = {
            "t_sec": self.elapsed(),
            "topic": topic,
            "summary": summary,
            "data": data,
        }
        self.events.append(event)
        self.write_trace()

    def on_walking_state(self, msg):
        state = WALKING_STATES.get(msg.locomotion_state, str(msg.locomotion_state))
        data = {
            "state": state,
            "mode": msg.locomotion_mode,
            "estop_active": msg.estop_active,
            "adapter_connected": msg.adapter_connected,
            "adapter": msg.active_adapter,
            "robot": msg.active_robot_model,
            "status_text": msg.status_text,
        }
        self.latest["walking_state"] = data
        if state != self.last_state:
            self.last_state = state
            self.append_event("/locomotion_ros2/state", f"walking state -> {state}", data)

    def on_adapter_status(self, msg):
        status = ADAPTER_STATES.get(msg.status, str(msg.status))
        data = {
            "status": status,
            "connected": msg.connected,
            "active": msg.active,
            "allow_motion": msg.allow_motion,
            "adapter": msg.adapter_name,
            "robot": msg.robot_model,
            "status_text": msg.status_text,
        }
        self.latest["adapter_status"] = data
        key = (status, msg.adapter_name, msg.connected, msg.active)
        if key != self.last_adapter:
            self.last_adapter = key
            self.append_event("/locomotion_ros2/adapter_status", f"adapter -> {status}", data)

    def on_safety_state(self, msg):
        state = SAFETY_STATES.get(msg.state, str(msg.state))
        data = {
            "state": state,
            "estop_active": msg.estop_active,
            "command_stale": msg.command_stale,
            "fall_detected": msg.fall_detected,
            "adapter_healthy": msg.adapter_healthy,
            "limits": {
                "max_linear_x": round(float(msg.max_linear_x), 3),
                "max_linear_y": round(float(msg.max_linear_y), 3),
                "max_angular_z": round(float(msg.max_angular_z), 3),
            },
            "status_text": msg.status_text,
        }
        self.latest["safety_state"] = data
        key = (state, msg.estop_active, msg.command_stale, msg.fall_detected)
        if key != self.last_safety:
            self.last_safety = key
            self.append_event("/locomotion_ros2/safety_state", f"safety -> {state}", data)

    def on_cmd_vel(self, msg):
        data = self.twist_data(msg)
        self.latest["cmd_vel"] = data
        self.append_event("/cmd_vel", self.twist_summary(data), data)

    def on_locomotion_ros2_cmd_vel(self, msg):
        data = self.twist_data(msg.twist)
        data["frame_id"] = msg.header.frame_id
        self.latest["locomotion_ros2_cmd_vel"] = data
        self.append_event("/locomotion_ros2/cmd_vel", self.twist_summary(data), data)

    def on_semantic_action(self, msg):
        data = {
            "source": msg.source,
            "action": msg.action,
            "target": msg.target,
            "confidence": round(float(msg.confidence), 3),
            "tags": list(msg.tags),
        }
        self.latest["semantic_action"] = data
        self.append_event("/locomotion_ros2/semantic_action", f"semantic -> {msg.action}", data)

    def twist_data(self, twist):
        return {
            "linear_x": round(float(twist.linear.x), 3),
            "linear_y": round(float(twist.linear.y), 3),
            "linear_z": round(float(twist.linear.z), 3),
            "angular_x": round(float(twist.angular.x), 3),
            "angular_y": round(float(twist.angular.y), 3),
            "angular_z": round(float(twist.angular.z), 3),
        }

    def twist_summary(self, data):
        return f"twist x={data['linear_x']:.2f} y={data['linear_y']:.2f} z={data['angular_z']:.2f}"

    def trace_payload(self):
        return {
            "schema": "locomotion_ros2.demo_trace.v1",
            "generated_by": self.get_name(),
            "output_dir": str(self.output_dir),
            "duration_sec": self.elapsed(),
            "latest": self.latest,
            "events": self.events,
        }

    def write_trace(self):
        payload = self.trace_payload()
        self.atomic_write(self.json_path, json.dumps(payload, indent=2, sort_keys=True) + "\n")
        self.atomic_write(self.markdown_path, self.render_markdown(payload))

    def atomic_write(self, path, text):
        tmp_path = path.with_name(f"{path.name}.tmp")
        tmp_path.write_text(text, encoding="utf-8")
        tmp_path.replace(path)

    def render_markdown(self, payload):
        lines = [
            "# locomotion_ros2 Demo Trace",
            "",
            f"- Duration: `{payload['duration_sec']:.3f}s`",
            f"- Output directory: `{payload['output_dir']}`",
            f"- Events: `{len(payload['events'])}`",
            "",
            "## Latest Runtime Snapshot",
            "",
        ]
        for key, value in payload["latest"].items():
            lines.append(f"- `{key}`: `{json.dumps(value, sort_keys=True)}`")
        lines.extend([
            "",
            "## Timeline",
            "",
            "| t_sec | topic | summary |",
            "| ---: | --- | --- |",
        ])
        for event in payload["events"]:
            summary = str(event["summary"]).replace("|", "\\|")
            lines.append(f"| {event['t_sec']:.3f} | `{event['topic']}` | {summary} |")
        lines.append("")
        return "\n".join(lines)


def main():
    rclpy.init()
    node = LocomotionRos2DemoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_trace()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
