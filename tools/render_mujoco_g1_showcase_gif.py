#!/usr/bin/env python3
"""Render the README hero GIF for the MuJoCo Unitree G1 gait showcase.

This optional documentation tool uses the existing Unitree G1 MJCF model from
MuJoCo Menagerie. It does not add a simulator dependency to the walking_zoo
runtime.
"""

from pathlib import Path
import math
import os

os.environ.setdefault("MUJOCO_GL", "egl")

try:
    import mujoco
    from PIL import Image, ImageDraw, ImageFont
except ImportError as error:
    raise SystemExit(
        "MuJoCo and Pillow are required for the README showcase GIF.\n"
        "Use: python3 -m venv /tmp/walking_zoo_gif_venv && "
        "/tmp/walking_zoo_gif_venv/bin/python -m pip install -r "
        "tools/readme_gif_requirements.txt"
    ) from error


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "assets" / "readme"
MENAGERIE = Path(
    os.environ.get("WALKING_ZOO_MENAGERIE_PATH", "/tmp/walking_zoo_mujoco_menagerie")
)
SIZE = (960, 540)

PANEL = (8, 14, 22)
PANEL_2 = (26, 40, 56)
TEXT = (232, 238, 245)
MUTED = (144, 160, 176)
GREEN = (70, 210, 160)
BLUE = (88, 166, 255)
YELLOW = (245, 198, 85)
RED = (245, 94, 94)
PURPLE = (176, 132, 255)
LINE = (70, 90, 112)


def font(size, bold=False):
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf" if bold else
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ]
    for candidate in candidates:
        if Path(candidate).exists():
            return ImageFont.truetype(candidate, size)
    return ImageFont.load_default()


FONT_TITLE = font(29, True)
FONT_SUB = font(16)
FONT_BODY = font(17)
FONT_SMALL = font(14)
FONT_TINY = font(12)


def round_rect(draw, box, fill, outline, radius=16, width=2):
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)


class G1ShowcaseScene:
    def __init__(self):
        scene_path = MENAGERIE / "unitree_g1" / "scene.xml"
        if not scene_path.exists():
            raise SystemExit(
                "Unitree G1 MJCF assets were not found.\n"
                "Clone mujoco_menagerie or set WALKING_ZOO_MENAGERIE_PATH:\n"
                "  git clone --depth 1 https://github.com/google-deepmind/"
                "mujoco_menagerie.git /tmp/walking_zoo_mujoco_menagerie"
            )
        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.model.vis.global_.offwidth = SIZE[0]
        self.model.vis.global_.offheight = SIZE[1]
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self.stand_qpos = self.data.qpos.copy()
        self.joint_qpos = {}
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            self.joint_qpos[name] = self.model.jnt_qposadr[i]
        self.renderer = mujoco.Renderer(self.model, height=SIZE[1], width=SIZE[0])
        self.camera = mujoco.MjvCamera()
        self.camera.distance = 1.78
        self.camera.azimuth = 142
        self.camera.elevation = -14

    def close(self):
        self.renderer.close()

    def set_joint(self, qpos, name, value):
        qpos[self.joint_qpos[name]] = value

    def yaw_quat(self, yaw):
        return [math.cos(yaw * 0.5), 0.0, 0.0, math.sin(yaw * 0.5)]

    def forward_gait(self, qpos, sin_phase, cos_phase, fast=False):
        hip_amp = 0.48 if fast else 0.30
        knee_base = 0.32 if fast else 0.18
        knee_amp = 0.62 if fast else 0.38
        ankle_base = -0.18 if fast else -0.13
        ankle_amp = 0.18 if fast else 0.12
        arm_amp = 0.65 if fast else 0.36

        for side, value in (("left", sin_phase), ("right", -sin_phase)):
            swing = max(0.0, value)
            self.set_joint(qpos, f"{side}_hip_pitch_joint", -hip_amp * value)
            self.set_joint(qpos, f"{side}_knee_joint", knee_base + knee_amp * swing)
            self.set_joint(
                qpos,
                f"{side}_ankle_pitch_joint",
                ankle_base - ankle_amp * swing + 0.10 * value,
            )

        self.set_joint(qpos, "left_hip_roll_joint", 0.04 * cos_phase)
        self.set_joint(qpos, "right_hip_roll_joint", -0.04 * cos_phase)
        self.set_joint(qpos, "waist_pitch_joint", -0.07)
        self.set_joint(qpos, "waist_yaw_joint", 0.05 * sin_phase)
        self.set_joint(qpos, "left_shoulder_pitch_joint", -arm_amp * sin_phase)
        self.set_joint(qpos, "right_shoulder_pitch_joint", arm_amp * sin_phase)
        self.set_joint(qpos, "left_shoulder_roll_joint", 0.10)
        self.set_joint(qpos, "right_shoulder_roll_joint", -0.10)
        self.set_joint(qpos, "left_elbow_joint", 0.50 + 0.20 * max(0.0, -sin_phase))
        self.set_joint(qpos, "right_elbow_joint", 0.50 + 0.20 * max(0.0, sin_phase))

    def pose(self, frame_index, gait):
        qpos = self.stand_qpos.copy()
        period = 30.0 if gait == "walk" else 24.0
        phase = 2.0 * math.pi * (frame_index / period)
        sin_phase = math.sin(phase)
        cos_phase = math.cos(phase)

        if gait == "stand":
            qpos[2] = 0.81
        elif gait == "estopped":
            qpos[2] = 0.79
            self.set_joint(qpos, "left_knee_joint", 0.36)
            self.set_joint(qpos, "right_knee_joint", 0.36)
            self.set_joint(qpos, "left_ankle_pitch_joint", -0.18)
            self.set_joint(qpos, "right_ankle_pitch_joint", -0.18)
        elif gait == "walk":
            qpos[0] = 0.022 * frame_index
            qpos[2] = 0.81 + 0.012 * max(0.0, cos_phase)
            qpos[3:7] = self.yaw_quat(0.0)
            self.forward_gait(qpos, sin_phase, cos_phase, fast=False)
        elif gait == "run":
            qpos[0] = 0.035 * frame_index
            qpos[2] = 0.82 + 0.02 * max(0.0, cos_phase)
            qpos[3:7] = self.yaw_quat(0.0)
            self.forward_gait(qpos, sin_phase, cos_phase, fast=True)
        elif gait in ("sidestep_left", "sidestep_right"):
            direction = 1.0 if gait == "sidestep_left" else -1.0
            qpos[1] = direction * 0.018 * frame_index
            qpos[2] = 0.81 + 0.012 * max(0.0, cos_phase)
            qpos[3:7] = self.yaw_quat(0.0)
            for side, value, sign in (("left", sin_phase, 1.0), ("right", -sin_phase, -1.0)):
                swing = max(0.0, value)
                self.set_joint(qpos, f"{side}_hip_pitch_joint", -0.08 * value)
                self.set_joint(qpos, f"{side}_hip_roll_joint", direction * sign * (0.06 + 0.24 * swing))
                self.set_joint(qpos, f"{side}_hip_yaw_joint", 0.06 * value)
                self.set_joint(qpos, f"{side}_knee_joint", 0.24 + 0.42 * swing)
                self.set_joint(qpos, f"{side}_ankle_pitch_joint", -0.10 - 0.08 * swing)
                self.set_joint(qpos, f"{side}_ankle_roll_joint", -direction * sign * (0.05 + 0.16 * swing))
            self.set_joint(qpos, "waist_roll_joint", direction * 0.05 * sin_phase)
            self.set_joint(qpos, "left_elbow_joint", 0.50)
            self.set_joint(qpos, "right_elbow_joint", 0.50)
        elif gait in ("turn_left", "turn_right"):
            direction = 1.0 if gait == "turn_left" else -1.0
            qpos[2] = 0.81 + 0.010 * max(0.0, cos_phase)
            qpos[3:7] = self.yaw_quat(direction * 0.030 * frame_index)
            for side, value, sign in (("left", sin_phase, 1.0), ("right", -sin_phase, -1.0)):
                swing = max(0.0, value)
                self.set_joint(qpos, f"{side}_hip_pitch_joint", -0.22 * value)
                self.set_joint(qpos, f"{side}_hip_yaw_joint", direction * sign * (0.10 + 0.18 * swing))
                self.set_joint(qpos, f"{side}_knee_joint", 0.24 + 0.42 * swing)
                self.set_joint(qpos, f"{side}_ankle_pitch_joint", -0.12 - 0.10 * swing)
                self.set_joint(qpos, f"{side}_ankle_roll_joint", -direction * sign * 0.06 * swing)
            self.set_joint(qpos, "waist_yaw_joint", direction * 0.08 * sin_phase)
            self.set_joint(qpos, "left_elbow_joint", 0.52)
            self.set_joint(qpos, "right_elbow_joint", 0.52)
        else:
            raise ValueError(f"unknown gait: {gait}")
        return qpos

    def render(self, frame_index, gait):
        self.data.qpos[:] = self.pose(frame_index, gait)
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        x = self.data.qpos[0]
        y = self.data.qpos[1]
        self.camera.lookat[:] = [x + 0.18, y, 0.82]
        self.renderer.update_scene(self.data, camera=self.camera)
        return Image.fromarray(self.renderer.render())


def draw_badge(draw, box, label, active, color):
    fill = PANEL_2 if active else PANEL
    outline = color if active else LINE
    text_fill = TEXT if active else MUTED
    round_rect(draw, box, fill, outline, radius=12, width=2)
    bbox = draw.textbbox((0, 0), label, font=FONT_TINY)
    x = box[0] + (box[2] - box[0] - bbox[2]) / 2
    y = box[1] + (box[3] - box[1] - bbox[3]) / 2 - 1
    draw.text((x, y), label, font=FONT_TINY, fill=text_fill)


def draw_overlay(img, spec, step_name):
    draw = ImageDraw.Draw(img)
    accent = spec["accent"]
    round_rect(draw, (18, 18, 458, 124), PANEL, accent, radius=16, width=2)
    draw.text((38, 34), "walking_zoo MuJoCo G1 showcase", font=FONT_TITLE, fill=TEXT)
    draw.text((40, 74), "one launch command, multiple humanoid walking modes", font=FONT_SUB, fill=MUTED)
    draw.text((40, 98), "ros2 launch walking_zoo_bringup mujoco_g1_gait_showcase.launch.py", font=FONT_TINY, fill=MUTED)

    round_rect(draw, (640, 122, 902, 340), PANEL_2, accent, radius=16, width=2)
    draw.text((674, 145), "Runtime Target", font=FONT_BODY, fill=accent)
    rows = [
        ("input", spec["input"], BLUE),
        ("gait", spec["gait"], accent),
        ("state", spec["state"], RED if spec["state"] == "ESTOPPED" else PURPLE),
        ("safety", spec["safety"], RED if spec["safety"] == "estop" else YELLOW),
        ("model", "Unitree G1 / MuJoCo", GREEN),
    ]
    y = 188
    for key, value, color in rows:
        draw.text((666, y), key, font=FONT_SMALL, fill=MUTED)
        draw.text((758, y), value[:19], font=FONT_SMALL, fill=color)
        y += 28

    labels = [
        ("walk", GREEN),
        ("run", YELLOW),
        ("left", BLUE),
        ("right", BLUE),
        ("turn", PURPLE),
        ("stop", MUTED),
        ("estop", RED),
    ]
    x = 44
    for label, color in labels:
        draw_badge(draw, (x, 474, x + 92, 510), label, label == step_name, color)
        x += 102
    return img


def build_sequence():
    return [
        {"step": "walk", "gait": "walk", "input": "semantic/walk", "state": "WALKING", "safety": "passed", "accent": GREEN, "frames": 14},
        {"step": "run", "gait": "run", "input": "semantic/run", "state": "RUNNING", "safety": "passed", "accent": YELLOW, "frames": 14},
        {"step": "left", "gait": "sidestep_left", "input": "semantic/sidestep_left", "state": "SIDESTEP", "safety": "passed", "accent": BLUE, "frames": 12},
        {"step": "right", "gait": "sidestep_right", "input": "semantic/sidestep_right", "state": "SIDESTEP", "safety": "passed", "accent": BLUE, "frames": 12},
        {"step": "turn", "gait": "turn_left", "input": "semantic/turn_left", "state": "TURNING", "safety": "passed", "accent": PURPLE, "frames": 14},
        {"step": "stop", "gait": "stand", "input": "semantic/stop", "state": "STANDING", "safety": "zero cmd", "accent": MUTED, "frames": 9},
        {"step": "estop", "gait": "estopped", "input": "semantic/estop", "state": "ESTOPPED", "safety": "estop", "accent": RED, "frames": 13},
    ]


def render_showcase():
    OUT.mkdir(parents=True, exist_ok=True)
    scene = G1ShowcaseScene()
    frames = []
    frame_index = 0
    try:
        for spec in build_sequence():
            for _ in range(spec["frames"]):
                if spec["gait"] not in ("stand", "estopped"):
                    frame_index += 1
                img = scene.render(frame_index, spec["gait"])
                frames.append(draw_overlay(img, spec, spec["step"]))
    finally:
        scene.close()

    gif_path = OUT / "mujoco_unitree_g1_showcase.gif"
    png_path = OUT / "mujoco_unitree_g1_showcase_preview.png"
    frames[-1].save(png_path)
    frames[0].save(
        gif_path,
        save_all=True,
        append_images=frames[1:],
        duration=72,
        loop=0,
        optimize=True,
    )
    print(gif_path.relative_to(ROOT))
    print(png_path.relative_to(ROOT))


if __name__ == "__main__":
    render_showcase()
