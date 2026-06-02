#!/usr/bin/env python3
"""Live MuJoCo Unitree G1 gait demo driven by walking_zoo ROS2 topics.

This is an optional visual demo. It does not participate in the runtime safety
path and it does not add MuJoCo as a dependency of walking_zoo itself.
"""

from collections import deque
from pathlib import Path
import math
import os

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from walking_zoo_msgs.msg import AdapterStatus, SafetyState, SemanticAction, WalkingState


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


def require_visual_deps():
    os.environ.setdefault("MUJOCO_GL", "egl")
    try:
        import mujoco
        from PIL import Image, ImageDraw, ImageFont
    except ImportError as error:
        raise RuntimeError(
            "MuJoCo G1 demo dependencies are missing. Install optional demo dependencies with:\n"
            "  python3 -m pip install -r tools/readme_gif_requirements.txt"
        ) from error
    return mujoco, Image, ImageDraw, ImageFont


class UnitreeG1Renderer:
    def __init__(self, menagerie_path, width=960, height=540):
        self.mujoco, self.Image, self.ImageDraw, self.ImageFont = require_visual_deps()
        self.width = width
        self.height = height

        scene_path = Path(menagerie_path) / "unitree_g1" / "scene.xml"
        if not scene_path.exists():
            raise RuntimeError(
                "Unitree G1 MJCF assets were not found. Clone MuJoCo Menagerie or set "
                "menagerie_path / WALKING_ZOO_MENAGERIE_PATH:\n"
                "  git clone --depth 1 https://github.com/google-deepmind/"
                "mujoco_menagerie.git /tmp/walking_zoo_mujoco_menagerie"
            )

        self.model = self.mujoco.MjModel.from_xml_path(str(scene_path))
        self.model.vis.global_.offwidth = width
        self.model.vis.global_.offheight = height
        self.data = self.mujoco.MjData(self.model)
        self.mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self.stand_qpos = self.data.qpos.copy()
        self.joint_qpos = {}
        for i in range(self.model.njnt):
            name = self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, i)
            self.joint_qpos[name] = self.model.jnt_qposadr[i]

        self.renderer = self.mujoco.Renderer(self.model, height=height, width=width)
        self.camera = self.mujoco.MjvCamera()
        self.camera.distance = 1.75
        self.camera.azimuth = 142
        self.camera.elevation = -14

    def close(self):
        self.renderer.close()

    def font(self, size, bold=False):
        candidates = [
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf" if bold else
            "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
        ]
        for candidate in candidates:
            if Path(candidate).exists():
                return self.ImageFont.truetype(candidate, size)
        return self.ImageFont.load_default()

    def _set_joint(self, qpos, name, value):
        qpos[self.joint_qpos[name]] = value

    def _yaw_quat(self, yaw):
        return [math.cos(yaw * 0.5), 0.0, 0.0, math.sin(yaw * 0.5)]

    def _body_quat(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]

    def _forward_gait(self, qpos, sin_phase, cos_phase, fast=False):
        hip_amp = 0.62 if fast else 0.30
        knee_base = 0.30 if fast else 0.18
        knee_amp = 0.74 if fast else 0.38
        ankle_base = -0.22 if fast else -0.13
        ankle_amp = 0.22 if fast else 0.12
        arm_amp = 0.58 if fast else 0.36

        for side, value in (("left", sin_phase), ("right", -sin_phase)):
            swing = max(0.0, value)
            stance = max(0.0, -value)
            hip_drive = -hip_amp * value - (0.10 * stance if fast else 0.0)
            knee_drive = knee_base + knee_amp * swing + (0.12 * stance if fast else 0.0)
            ankle_drive = (
                ankle_base
                - ankle_amp * swing
                + (0.14 if fast else 0.10) * value
                - (0.06 * stance if fast else 0.0)
            )
            self._set_joint(qpos, f"{side}_hip_pitch_joint", hip_drive)
            self._set_joint(qpos, f"{side}_knee_joint", knee_drive)
            self._set_joint(
                qpos,
                f"{side}_ankle_pitch_joint",
                ankle_drive,
            )

        roll_amp = 0.07 if fast else 0.04
        self._set_joint(qpos, "left_hip_roll_joint", roll_amp * cos_phase)
        self._set_joint(qpos, "right_hip_roll_joint", -roll_amp * cos_phase)
        self._set_joint(qpos, "waist_pitch_joint", -0.11 if fast else -0.07)
        self._set_joint(qpos, "waist_yaw_joint", (0.08 if fast else 0.05) * sin_phase)
        self._set_joint(qpos, "waist_roll_joint", 0.035 * cos_phase if fast else 0.0)
        self._set_joint(qpos, "left_shoulder_pitch_joint", -arm_amp * sin_phase)
        self._set_joint(qpos, "right_shoulder_pitch_joint", arm_amp * sin_phase)
        self._set_joint(qpos, "left_shoulder_roll_joint", 0.16 if fast else 0.10)
        self._set_joint(qpos, "right_shoulder_roll_joint", -0.16 if fast else -0.10)
        elbow_base = 0.95 if fast else 0.50
        elbow_amp = 0.18 if fast else 0.20
        self._set_joint(qpos, "left_elbow_joint", elbow_base + elbow_amp * max(0.0, -sin_phase))
        self._set_joint(qpos, "right_elbow_joint", elbow_base + elbow_amp * max(0.0, sin_phase))

    def pose(self, frame_index, gait):
        qpos = self.stand_qpos.copy()
        period = 18.0 if gait == "run" else (30.0 if gait == "walk" else 24.0)
        phase = 2.0 * math.pi * (frame_index / period)
        sin_phase = math.sin(phase)
        cos_phase = math.cos(phase)

        if gait == "stand":
            qpos[2] = 0.81
            return qpos
        if gait == "estopped":
            qpos[2] = 0.79
            self._set_joint(qpos, "left_knee_joint", 0.36)
            self._set_joint(qpos, "right_knee_joint", 0.36)
            self._set_joint(qpos, "left_ankle_pitch_joint", -0.18)
            self._set_joint(qpos, "right_ankle_pitch_joint", -0.18)
            return qpos
        if gait == "walk":
            qpos[0] = 0.022 * frame_index
            qpos[2] = 0.81 + 0.012 * max(0.0, cos_phase)
            qpos[3:7] = self._yaw_quat(0.0)
            self._forward_gait(qpos, sin_phase, cos_phase, fast=False)
            return qpos
        if gait == "run":
            qpos[0] = 0.046 * frame_index
            qpos[2] = 0.815 + 0.026 * (0.5 + 0.5 * math.cos(2.0 * phase))
            qpos[3:7] = self._body_quat(0.0, -0.07 + 0.012 * sin_phase, 0.0)
            self._forward_gait(qpos, sin_phase, cos_phase, fast=True)
            return qpos
        if gait in ("sidestep_left", "sidestep_right"):
            direction = 1.0 if gait == "sidestep_left" else -1.0
            qpos[1] = direction * 0.018 * frame_index
            qpos[2] = 0.81 + 0.012 * max(0.0, cos_phase)
            qpos[3:7] = self._yaw_quat(0.0)
            for side, value, sign in (("left", sin_phase, 1.0), ("right", -sin_phase, -1.0)):
                swing = max(0.0, value)
                self._set_joint(qpos, f"{side}_hip_pitch_joint", -0.08 * value)
                self._set_joint(qpos, f"{side}_hip_roll_joint", direction * sign * (0.06 + 0.24 * swing))
                self._set_joint(qpos, f"{side}_hip_yaw_joint", 0.06 * value)
                self._set_joint(qpos, f"{side}_knee_joint", 0.24 + 0.42 * swing)
                self._set_joint(qpos, f"{side}_ankle_pitch_joint", -0.10 - 0.08 * swing)
                self._set_joint(qpos, f"{side}_ankle_roll_joint", -direction * sign * (0.05 + 0.16 * swing))
            self._set_joint(qpos, "waist_roll_joint", direction * 0.05 * sin_phase)
            self._set_joint(qpos, "left_elbow_joint", 0.50)
            self._set_joint(qpos, "right_elbow_joint", 0.50)
            return qpos
        if gait in ("turn_left", "turn_right"):
            direction = 1.0 if gait == "turn_left" else -1.0
            qpos[2] = 0.81 + 0.010 * max(0.0, cos_phase)
            qpos[3:7] = self._yaw_quat(direction * 0.030 * frame_index)
            for side, value, sign in (("left", sin_phase, 1.0), ("right", -sin_phase, -1.0)):
                swing = max(0.0, value)
                self._set_joint(qpos, f"{side}_hip_pitch_joint", -0.22 * value)
                self._set_joint(qpos, f"{side}_hip_yaw_joint", direction * sign * (0.10 + 0.18 * swing))
                self._set_joint(qpos, f"{side}_knee_joint", 0.24 + 0.42 * swing)
                self._set_joint(qpos, f"{side}_ankle_pitch_joint", -0.12 - 0.10 * swing)
                self._set_joint(qpos, f"{side}_ankle_roll_joint", -direction * sign * 0.06 * swing)
            self._set_joint(qpos, "waist_yaw_joint", direction * 0.08 * sin_phase)
            self._set_joint(qpos, "left_elbow_joint", 0.52)
            self._set_joint(qpos, "right_elbow_joint", 0.52)
            return qpos
        return qpos

    def render(self, frame_index, gait):
        self.data.qpos[:] = self.pose(frame_index, gait)
        self.data.qvel[:] = 0.0
        self.mujoco.mj_forward(self.model, self.data)
        x = self.data.qpos[0]
        y = self.data.qpos[1]
        self.camera.lookat[:] = [x + 0.18, y, 0.82]
        self.renderer.update_scene(self.data, camera=self.camera)
        return self.Image.fromarray(self.renderer.render())

    def draw_overlay(
        self,
        img,
        gait,
        source,
        runtime_state,
        adapter_state,
        safety_state,
        latest_cmd,
        output_dir,
    ):
        draw = self.ImageDraw.Draw(img)
        panel = (18, 18, 425, 118)
        accent = (245, 94, 94) if gait == "estopped" else (176, 132, 255)
        draw.rounded_rectangle(panel, radius=16, fill=(8, 14, 22), outline=accent, width=2)
        draw.text((38, 33), "walking_zoo MuJoCo G1 live demo", font=self.font(23, True), fill=(232, 238, 245))
        draw.text((40, 69), f"gait={gait}  source={source}", font=self.font(15), fill=(144, 160, 176))
        draw.text((40, 92), f"output={output_dir}", font=self.font(13), fill=(144, 160, 176))

        state_panel = (642, 112, 902, 342)
        draw.rounded_rectangle(state_panel, radius=16, fill=(26, 40, 56), outline=accent, width=2)
        rows = [
            ("runtime", runtime_state or "unknown"),
            ("adapter", adapter_state or "unknown"),
            ("safety", safety_state or "unknown"),
            ("cmd", latest_cmd),
            ("model", "Unitree G1"),
        ]
        draw.text((675, 134), "Runtime Target", font=self.font(20), fill=accent)
        y = 174
        for key, value in rows:
            draw.text((668, y), key, font=self.font(14), fill=(144, 160, 176))
            draw.text((760, y), value[:18], font=self.font(14), fill=(232, 238, 245))
            y += 30
        return img


class MujocoG1GaitDemo(Node):
    def __init__(self):
        super().__init__("walking_zoo_mujoco_g1_gait_demo")
        self.declare_parameter(
            "menagerie_path",
            os.environ.get("WALKING_ZOO_MENAGERIE_PATH", "/tmp/walking_zoo_mujoco_menagerie"),
        )
        self.declare_parameter("output_dir", "/tmp/walking_zoo_mujoco_g1_demo")
        self.declare_parameter("fps", 12.0)
        self.declare_parameter("command_timeout_sec", 0.8)
        self.declare_parameter("gif_window_frames", 72)
        self.declare_parameter("gif_width", 360)
        self.declare_parameter("gif_preview_sec", 2.0)
        self.declare_parameter("gif_update_sec", 8.0)

        self.output_dir = Path(self.get_parameter("output_dir").value)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.latest_png = self.output_dir / "latest.png"
        self.live_gif = self.output_dir / "live.gif"
        self.fps = float(self.get_parameter("fps").value)
        self.timeout_sec = float(self.get_parameter("command_timeout_sec").value)
        self.gif_window_frames = int(self.get_parameter("gif_window_frames").value)
        self.gif_width = int(self.get_parameter("gif_width").value)
        self.gif_preview_frames = max(
            2,
            int(self.fps * float(self.get_parameter("gif_preview_sec").value)),
        )
        self.gif_update_frames = max(
            self.gif_preview_frames,
            int(self.fps * float(self.get_parameter("gif_update_sec").value)),
        )

        self.renderer = UnitreeG1Renderer(self.get_parameter("menagerie_path").value)
        self.frame_index = 0
        self.gif_frames = deque(maxlen=self.gif_window_frames)
        self.last_gif_save_frame = 0
        self.cmd_gait = "stand"
        self.semantic_gait = None
        self.last_command_time = self.get_clock().now()
        self.estop_active = False
        self.runtime_state = ""
        self.adapter_state = ""
        self.safety_state = ""
        self.source = "idle"
        self.latest_cmd = "zero"

        self.create_subscription(TwistStamped, "/walking_zoo/cmd_vel", self.on_twist_stamped, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_twist, 10)
        self.create_subscription(SemanticAction, "/walking_zoo/semantic_action", self.on_semantic, 10)
        self.create_subscription(SafetyState, "/walking_zoo/safety_state", self.on_safety_state, 10)
        self.create_subscription(AdapterStatus, "/walking_zoo/adapter_status", self.on_adapter_status, 10)
        self.create_subscription(WalkingState, "/walking_zoo/state", self.on_walking_state, 10)
        self.timer = self.create_timer(1.0 / max(self.fps, 1.0), self.on_timer)
        self.get_logger().info(f"writing MuJoCo G1 frames to {self.output_dir}")

    def destroy_node(self):
        if hasattr(self, "renderer"):
            try:
                if not self.live_gif.exists() or self.live_gif.stat().st_size == 0:
                    self.save_live_gif()
            finally:
                try:
                    self.renderer.close()
                except KeyboardInterrupt:
                    pass
                except Exception as error:
                    self.get_logger().warn(f"MuJoCo renderer close failed: {error}")
        super().destroy_node()

    def on_twist_stamped(self, msg):
        self.update_from_twist(msg.twist, "walking_zoo/cmd_vel")

    def on_twist(self, msg):
        self.update_from_twist(msg, "cmd_vel")

    def update_from_twist(self, twist, source):
        linear_x = twist.linear.x
        linear_y = twist.linear.y
        angular_z = twist.angular.z
        self.latest_cmd = f"x={linear_x:.2f} y={linear_y:.2f} z={angular_z:.2f}"
        self.last_command_time = self.get_clock().now()
        self.source = source
        self.semantic_gait = None

        if abs(linear_x) < 0.03 and abs(linear_y) < 0.03 and abs(angular_z) < 0.05:
            self.cmd_gait = "stand"
        elif abs(linear_y) > max(abs(linear_x), abs(angular_z) * 0.2, 0.04):
            self.cmd_gait = "sidestep_left" if linear_y > 0.0 else "sidestep_right"
        elif abs(angular_z) > max(abs(linear_x), 0.08):
            self.cmd_gait = "turn_left" if angular_z > 0.0 else "turn_right"
        elif abs(linear_x) > 0.35:
            self.cmd_gait = "run"
        else:
            self.cmd_gait = "walk"

    def on_semantic(self, msg):
        action = msg.action.lower().replace("-", "_").strip()
        mapping = {
            "walk": "walk",
            "walk_forward": "walk",
            "move_forward": "walk",
            "forward": "walk",
            "run": "run",
            "run_forward": "run",
            "sidestep": "sidestep_left",
            "sidestep_left": "sidestep_left",
            "strafe_left": "sidestep_left",
            "left": "sidestep_left",
            "sidestep_right": "sidestep_right",
            "strafe_right": "sidestep_right",
            "right": "sidestep_right",
            "turn": "turn_left",
            "turn_left": "turn_left",
            "rotate_left": "turn_left",
            "turn_right": "turn_right",
            "rotate_right": "turn_right",
            "stop": "stand",
            "idle": "stand",
            "stand": "stand",
            "estop": "estopped",
            "emergency_stop": "estopped",
        }
        if action in mapping:
            self.semantic_gait = mapping[action]
            self.source = f"semantic:{action}"
            self.last_command_time = self.get_clock().now()
            self.latest_cmd = action
            self.get_logger().info(f"semantic action '{action}' -> gait '{self.semantic_gait}'")
        else:
            self.get_logger().warn(f"unknown semantic action: {msg.action}")

    def on_safety_state(self, msg):
        self.estop_active = msg.estop_active or msg.state == SafetyState.STATE_ESTOPPED
        self.safety_state = SAFETY_STATES.get(msg.state, str(msg.state))
        if msg.estop_active:
            self.safety_state = "ESTOPPED"

    def on_adapter_status(self, msg):
        status = ADAPTER_STATES.get(msg.status, str(msg.status))
        adapter = self.short_adapter_name(msg.adapter_name)
        self.adapter_state = f"{adapter}:{status.lower()}"

    def short_adapter_name(self, adapter_name):
        plugin = adapter_name.rsplit("/", 1)[-1] if adapter_name else "adapter"
        if plugin == "MockWalkingAdapter":
            return "mock"
        if plugin == "UnitreeSdk2Adapter":
            return "unitree"
        return plugin

    def on_walking_state(self, msg):
        self.runtime_state = msg.status_text or str(msg.locomotion_state)
        if msg.estop_active:
            self.estop_active = True

    def current_gait(self):
        if self.estop_active:
            return "estopped"
        now = self.get_clock().now()
        age = (now - self.last_command_time).nanoseconds / 1e9
        if self.semantic_gait is not None:
            return self.semantic_gait
        if age > self.timeout_sec:
            self.source = "timeout"
            return "stand"
        return self.cmd_gait

    def on_timer(self):
        gait = self.current_gait()
        if gait not in ("stand", "estopped"):
            self.frame_index += 1
        img = self.renderer.render(self.frame_index, gait)
        img = self.renderer.draw_overlay(
            img,
            gait,
            self.source,
            self.runtime_state,
            self.adapter_state,
            self.safety_state,
            self.latest_cmd,
            str(self.output_dir),
        )
        self.save_latest_png(img)
        self.gif_frames.append(img.copy())
        live_gif_missing = (
            not self.live_gif.exists() or self.live_gif.stat().st_size == 0
        )
        first_live_gif = live_gif_missing and len(self.gif_frames) >= self.gif_preview_frames
        periodic_live_gif = self.frame_index - self.last_gif_save_frame >= self.gif_update_frames
        if first_live_gif or periodic_live_gif:
            self.save_live_gif()
            self.last_gif_save_frame = self.frame_index

    def save_latest_png(self, img):
        tmp_png = self.latest_png.with_name(f"{self.latest_png.name}.tmp")
        img.save(tmp_png, format="PNG")
        tmp_png.replace(self.latest_png)

    def save_live_gif(self):
        if len(self.gif_frames) < 2:
            return
        resampling = getattr(self.renderer.Image, "Resampling", None)
        resample = resampling.LANCZOS if resampling is not None else self.renderer.Image.LANCZOS
        gif_width = max(240, self.gif_width)
        gif_height = int(gif_width * self.renderer.height / self.renderer.width)
        frames = [
            frame.resize((gif_width, gif_height), resample=resample)
            for frame in list(self.gif_frames)[-self.gif_preview_frames:]
        ]
        duration_ms = int(1000.0 / max(self.fps, 1.0))
        tmp_gif = self.live_gif.with_name(f"{self.live_gif.name}.tmp")
        frames[0].save(
            tmp_gif,
            format="GIF",
            save_all=True,
            append_images=frames[1:],
            duration=duration_ms,
            loop=0,
            optimize=False,
        )
        tmp_gif.replace(self.live_gif)


def main():
    rclpy.init()
    node = MujocoG1GaitDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
