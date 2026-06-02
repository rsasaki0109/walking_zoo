#!/usr/bin/env python3
"""Render README GIFs with existing simulators.

Robot GIFs are rendered by MuJoCo and PyBullet using existing robot assets.
The script is intentionally optional and is not part of the walking_zoo runtime
dependency set.
"""

from pathlib import Path
import math
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

os.environ.setdefault("MUJOCO_GL", "egl")

try:
    import mujoco
    import pybullet as p
    import pybullet_data
except ImportError as error:
    raise SystemExit(
        "MuJoCo and PyBullet are required for README robot GIF generation.\n"
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

BG = (9, 14, 22)
PANEL = (18, 28, 40)
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


FONT_TITLE = font(36, True)
FONT_SUB = font(22)
FONT_BODY = font(19)
FONT_SMALL = font(15)
FONT_CODE = font(17)


def draw_round(draw, box, fill, outline=None, radius=18, width=2):
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)


def text_center(draw, box, text, fill=TEXT, fnt=FONT_BODY):
    bbox = draw.textbbox((0, 0), text, font=fnt)
    x = box[0] + (box[2] - box[0] - (bbox[2] - bbox[0])) / 2
    y = box[1] + (box[3] - box[1] - (bbox[3] - bbox[1])) / 2
    draw.text((x, y), text, font=fnt, fill=fill)


def arrow(draw, start, end, color=LINE, width=5, progress=1.0):
    sx, sy = start
    ex, ey = end
    px = sx + (ex - sx) * progress
    py = sy + (ey - sy) * progress
    draw.line((sx, sy, px, py), fill=color, width=width)
    if progress >= 0.96:
        angle = math.atan2(ey - sy, ex - sx)
        length = 14
        for delta in (math.pi * 0.84, -math.pi * 0.84):
            x = ex + math.cos(angle + delta) * length
            y = ey + math.sin(angle + delta) * length
            draw.line((ex, ey, x, y), fill=color, width=width)


def base(title, subtitle):
    img = Image.new("RGB", SIZE, BG)
    draw = ImageDraw.Draw(img)
    draw.text((42, 30), title, font=FONT_TITLE, fill=TEXT)
    draw.text((44, 78), subtitle, font=FONT_SUB, fill=MUTED)
    return img, draw


def save_gif(name, frames, duration=90):
    path = OUT / name
    frames[0].save(
        path,
        save_all=True,
        append_images=frames[1:],
        duration=duration,
        loop=0,
        optimize=True,
    )
    print(path.relative_to(ROOT))


def overlay_panel(img, title, rows, accent=GREEN):
    draw = ImageDraw.Draw(img)
    draw_round(draw, (635, 132, 902, 326), PANEL_2, accent, radius=18)
    text_center(draw, (655, 146, 882, 190), title, accent, FONT_BODY)
    y = 204
    for key, value, color in rows:
        draw.text((662, y), key, font=FONT_SMALL, fill=MUTED)
        draw.text((790, y), value, font=FONT_SMALL, fill=color)
        y += 29


class PyBulletLaikagoScene:
    def __init__(self):
        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        p.setTimeStep(1.0 / 240.0, physicsClientId=self.client)
        self.plane = p.loadURDF("plane.urdf", physicsClientId=self.client)
        self.robot = p.loadURDF(
            "laikago/laikago_toes_zup.urdf",
            [0.0, 0.0, 0.58],
            p.getQuaternionFromEuler([0.0, 0.0, 0.0]),
            flags=p.URDF_USE_SELF_COLLISION,
            physicsClientId=self.client,
        )
        self.legs = {
            "FR": (0, 1, 2, 0.00),
            "FL": (4, 5, 6, math.pi),
            "RR": (8, 9, 10, math.pi),
            "RL": (12, 13, 14, 0.00),
        }
        self.stand_upper = 0.72
        self.stand_knee = -1.42

        for _name, (hip, upper, knee, _phase) in self.legs.items():
            p.resetJointState(self.robot, hip, 0.0, physicsClientId=self.client)
            p.resetJointState(self.robot, upper, self.stand_upper, physicsClientId=self.client)
            p.resetJointState(self.robot, knee, self.stand_knee, physicsClientId=self.client)
        for _ in range(20):
            p.stepSimulation(physicsClientId=self.client)

    def close(self):
        p.disconnect(self.client)

    def apply_trot_pose(self, sim_t, x, stopped=False):
        z = 0.49 + (0.0 if stopped else 0.025 * math.sin(2.0 * math.pi * sim_t * 2.0))
        pitch = 0.0 if stopped else 0.035 * math.sin(2.0 * math.pi * sim_t * 1.4)
        yaw = 0.0 if stopped else 0.015 * math.sin(2.0 * math.pi * sim_t * 0.7)
        p.resetBasePositionAndOrientation(
            self.robot,
            [x, 0.0, z],
            p.getQuaternionFromEuler([0.0, pitch, yaw]),
            physicsClientId=self.client,
        )
        p.resetBaseVelocity(
            self.robot,
            [0.36 if not stopped else 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            physicsClientId=self.client,
        )

        for _name, (hip, upper, knee, phase) in self.legs.items():
            ph = 2.0 * math.pi * 1.7 * sim_t + phase
            if stopped:
                target_hip = 0.0
                target_upper = self.stand_upper
                target_knee = self.stand_knee
            else:
                swing = math.sin(ph)
                lift = max(0.0, math.sin(ph))
                target_hip = 0.10 * math.sin(ph + 0.5)
                target_upper = self.stand_upper + 0.20 * swing + 0.12 * lift
                target_knee = self.stand_knee - 0.25 * lift + 0.08 * swing
            for joint, angle in ((hip, target_hip), (upper, target_upper), (knee, target_knee)):
                p.resetJointState(self.robot, joint, angle, physicsClientId=self.client)
                p.setJointMotorControl2(
                    self.robot,
                    joint,
                    p.POSITION_CONTROL,
                    targetPosition=angle,
                    force=95,
                    positionGain=0.75,
                    velocityGain=0.25,
                    physicsClientId=self.client,
                )
        p.stepSimulation(physicsClientId=self.client)

    def render(self, x, yaw=0.0):
        view = p.computeViewMatrix(
            cameraEyePosition=[x - 1.65, -2.45, 1.15],
            cameraTargetPosition=[x + 0.18, 0.0, 0.38],
            cameraUpVector=[0.0, 0.0, 1.0],
        )
        projection = p.computeProjectionMatrixFOV(
            fov=48,
            aspect=SIZE[0] / SIZE[1],
            nearVal=0.05,
            farVal=50.0,
        )
        _w, _h, rgba, _depth, _seg = p.getCameraImage(
            width=SIZE[0],
            height=SIZE[1],
            viewMatrix=view,
            projectionMatrix=projection,
            renderer=p.ER_TINY_RENDERER,
            physicsClientId=self.client,
        )
        arr = np.reshape(np.asarray(rgba, dtype=np.uint8), (SIZE[1], SIZE[0], 4))
        return Image.fromarray(arr[:, :, :3], "RGB")


def pybullet_laikago_runtime():
    scene = PyBulletLaikagoScene()
    frames = []
    try:
        for frame in range(44):
            sim_t = frame / 12.0
            x = 0.28 * sim_t
            scene.apply_trot_pose(sim_t, x)
            img = scene.render(x)
            draw = ImageDraw.Draw(img)
            draw_round(draw, (26, 24, 595, 110), (8, 14, 22), GREEN, radius=18)
            draw.text((48, 40), "PyBullet Laikago simulation", font=FONT_TITLE, fill=TEXT)
            draw.text((50, 84), "Nav2 /cmd_vel -> walking_zoo runtime -> mock adapter", font=FONT_SMALL, fill=MUTED)
            overlay_panel(
                img,
                "ROS2 runtime",
                [
                    ("input", "/cmd_vel", BLUE),
                    ("safety", "passed", YELLOW),
                    ("adapter", "mock", GREEN),
                    ("state", "WALKING", PURPLE),
                ],
                GREEN,
            )
            frames.append(img)
    finally:
        scene.close()
    save_gif("pybullet_laikago_runtime.gif", frames, duration=80)


def pybullet_laikago_estop():
    scene = PyBulletLaikagoScene()
    frames = []
    try:
        stop_x = 0.0
        for frame in range(44):
            sim_t = frame / 12.0
            estop = frame >= 25
            if not estop:
                x = 0.28 * sim_t
                stop_x = x
                stopped = False
            else:
                tau = min((frame - 25) / 12.0, 0.45)
                x = stop_x + 0.18 * tau * (1.0 - tau / 0.45)
                stopped = frame >= 31
            scene.apply_trot_pose(sim_t, x, stopped=stopped)
            img = scene.render(x)
            draw = ImageDraw.Draw(img)
            draw_round(draw, (26, 24, 585, 110), (8, 14, 22), RED if estop else GREEN, radius=18)
            draw.text((48, 40), "PyBullet e-stop simulation", font=FONT_TITLE, fill=TEXT)
            draw.text((50, 84), "runtime gate blocks adapter commands before motion continues", font=FONT_SMALL, fill=MUTED)
            overlay_panel(
                img,
                "Safety gate",
                [
                    ("estop", "active" if estop else "false", RED if estop else GREEN),
                    ("adapter", "blocked" if estop else "accepted", RED if estop else GREEN),
                    ("velocity", "0.00" if stopped else "0.28", RED if stopped else YELLOW),
                    ("state", "ESTOPPED" if stopped else "WALKING", RED if stopped else PURPLE),
                ],
                RED if estop else GREEN,
            )
            frames.append(img)
    finally:
        scene.close()
    save_gif("pybullet_laikago_estop.gif", frames, duration=80)


class MujocoUnitreeG1Scene:
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
        self.camera.distance = 1.72
        self.camera.azimuth = 142
        self.camera.elevation = -14

    def close(self):
        self.renderer.close()

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

    def _set_forward_gait(
        self,
        qpos,
        sin_phase,
        cos_phase,
        hip_amp,
        knee_base,
        knee_amp,
        ankle_base,
        ankle_amp,
        arm_amp,
    ):
        fast = hip_amp > 0.5
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

    def _g1_gait_pose(self, frame_index, gait):
        qpos = self.stand_qpos.copy()
        period = 18.0 if gait == "run" else (30.0 if gait == "walk" else 24.0)
        phase = 2.0 * math.pi * (frame_index / period)
        sin_phase = math.sin(phase)
        cos_phase = math.cos(phase)

        if gait == "walk":
            qpos[0] = 0.022 * frame_index
            qpos[2] = 0.81 + 0.012 * max(0.0, cos_phase)
            qpos[3:7] = self._yaw_quat(0.0)
            self._set_forward_gait(qpos, sin_phase, cos_phase, 0.30, 0.18, 0.38, -0.13, 0.12, 0.36)
        elif gait == "run":
            qpos[0] = 0.046 * frame_index
            qpos[2] = 0.815 + 0.026 * (0.5 + 0.5 * math.cos(2.0 * phase))
            qpos[3:7] = self._body_quat(0.0, -0.07 + 0.012 * sin_phase, 0.0)
            self._set_forward_gait(qpos, sin_phase, cos_phase, 0.62, 0.30, 0.74, -0.22, 0.22, 0.58)
        elif gait == "walk_backward":
            qpos[0] = -0.016 * frame_index
            qpos[2] = 0.806 + 0.010 * max(0.0, cos_phase)
            qpos[3:7] = self._body_quat(0.0, 0.06, 0.0)
            for side, value in (("left", sin_phase), ("right", -sin_phase)):
                swing = max(0.0, value)
                self._set_joint(qpos, f"{side}_hip_pitch_joint", 0.20 * value)
                self._set_joint(qpos, f"{side}_knee_joint", 0.16 + 0.30 * swing)
                self._set_joint(qpos, f"{side}_ankle_pitch_joint", -0.06 - 0.10 * swing + 0.08 * value)
            self._set_joint(qpos, "left_hip_roll_joint", 0.04 * cos_phase)
            self._set_joint(qpos, "right_hip_roll_joint", -0.04 * cos_phase)
            self._set_joint(qpos, "waist_pitch_joint", 0.05)
            self._set_joint(qpos, "waist_yaw_joint", 0.04 * sin_phase)
            self._set_joint(qpos, "left_shoulder_pitch_joint", 0.10 - 0.18 * sin_phase)
            self._set_joint(qpos, "right_shoulder_pitch_joint", 0.10 + 0.18 * sin_phase)
            self._set_joint(qpos, "left_shoulder_roll_joint", 0.12)
            self._set_joint(qpos, "right_shoulder_roll_joint", -0.12)
            self._set_joint(qpos, "left_elbow_joint", 0.55)
            self._set_joint(qpos, "right_elbow_joint", 0.55)
        elif gait == "sidestep":
            qpos[1] = 0.018 * frame_index
            qpos[2] = 0.81 + 0.012 * max(0.0, cos_phase)
            qpos[3:7] = self._yaw_quat(0.0)
            for side, value, sign in (("left", sin_phase, 1.0), ("right", -sin_phase, -1.0)):
                swing = max(0.0, value)
                self._set_joint(qpos, f"{side}_hip_pitch_joint", -0.08 * value)
                self._set_joint(qpos, f"{side}_hip_roll_joint", sign * (0.06 + 0.24 * swing))
                self._set_joint(qpos, f"{side}_hip_yaw_joint", 0.06 * value)
                self._set_joint(qpos, f"{side}_knee_joint", 0.24 + 0.42 * swing)
                self._set_joint(qpos, f"{side}_ankle_pitch_joint", -0.10 - 0.08 * swing)
                self._set_joint(qpos, f"{side}_ankle_roll_joint", -sign * (0.05 + 0.16 * swing))
            self._set_joint(qpos, "waist_roll_joint", 0.05 * sin_phase)
            self._set_joint(qpos, "left_shoulder_pitch_joint", -0.22 * sin_phase)
            self._set_joint(qpos, "right_shoulder_pitch_joint", 0.22 * sin_phase)
            self._set_joint(qpos, "left_elbow_joint", 0.50)
            self._set_joint(qpos, "right_elbow_joint", 0.50)
        elif gait == "turn":
            yaw = 0.030 * frame_index
            qpos[2] = 0.81 + 0.010 * max(0.0, cos_phase)
            qpos[3:7] = self._yaw_quat(yaw)
            for side, value, sign in (("left", sin_phase, 1.0), ("right", -sin_phase, -1.0)):
                swing = max(0.0, value)
                self._set_joint(qpos, f"{side}_hip_pitch_joint", -0.22 * value)
                self._set_joint(qpos, f"{side}_hip_yaw_joint", sign * (0.10 + 0.18 * swing))
                self._set_joint(qpos, f"{side}_knee_joint", 0.24 + 0.42 * swing)
                self._set_joint(qpos, f"{side}_ankle_pitch_joint", -0.12 - 0.10 * swing)
                self._set_joint(qpos, f"{side}_ankle_roll_joint", -sign * 0.06 * swing)
            self._set_joint(qpos, "waist_yaw_joint", 0.08 * sin_phase)
            self._set_joint(qpos, "left_shoulder_pitch_joint", -0.30 * sin_phase)
            self._set_joint(qpos, "right_shoulder_pitch_joint", 0.30 * sin_phase)
            self._set_joint(qpos, "left_elbow_joint", 0.52)
            self._set_joint(qpos, "right_elbow_joint", 0.52)
        else:
            raise ValueError(f"unknown gait: {gait}")

        return qpos

    def apply_gait_frame(self, frame_index, gait):
        self.data.qpos[:] = self._g1_gait_pose(frame_index, gait)
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        return self.data.qpos[0], self.data.qpos[1]

    def apply_run_frame(self, frame_index):
        x, _y = self.apply_gait_frame(frame_index, "run")
        return x

    def render(self, x, y=0.0, distance=None, elevation=None):
        if distance is not None:
            self.camera.distance = distance
        if elevation is not None:
            self.camera.elevation = elevation
        self.camera.lookat[:] = [x + 0.18, y, 0.82]
        self.renderer.update_scene(self.data, camera=self.camera)
        return Image.fromarray(self.renderer.render())


def draw_small_label(img, title, subtitle, accent):
    draw = ImageDraw.Draw(img)
    draw_round(draw, (258, 198, 462, 258), (8, 14, 22), accent, radius=12)
    draw.text((274, 209), title, font=FONT_SMALL, fill=TEXT)
    draw.text((274, 232), subtitle, font=font(12), fill=MUTED)


def mujoco_unitree_g1_gait_gallery(scene=None):
    owns_scene = scene is None
    if owns_scene:
        scene = MujocoUnitreeG1Scene()
    gait_specs = [
        ("walk", "Forward walk", "/cmd_vel x=0.20", GREEN),
        ("run", "Forward run", "semantic/run", YELLOW),
        ("sidestep", "Sidestep", "body pose lateral", BLUE),
        ("turn", "Turn-in-place", "yaw command", PURPLE),
    ]
    frames = []
    try:
        for frame in range(48):
            tiles = []
            for gait, title, subtitle, accent in gait_specs:
                x, y = scene.apply_gait_frame(frame, gait)
                tile = scene.render(x, y, distance=2.28, elevation=-15).resize(
                    (480, 270),
                    Image.Resampling.LANCZOS,
                )
                draw_small_label(tile, title, subtitle, accent)
                tiles.append(tile)
            canvas = Image.new("RGB", SIZE, BG)
            for index, tile in enumerate(tiles):
                canvas.paste(tile, ((index % 2) * 480, (index // 2) * 270))
            frames.append(canvas)
    finally:
        if owns_scene:
            scene.close()
    save_gif("mujoco_unitree_g1_gait_gallery.gif", frames, duration=80)


def mujoco_unitree_g1_runtime(scene=None):
    owns_scene = scene is None
    if owns_scene:
        scene = MujocoUnitreeG1Scene()
    frames = []
    try:
        for frame in range(54):
            x = scene.apply_run_frame(frame)
            img = scene.render(x)
            draw = ImageDraw.Draw(img)
            draw_round(draw, (26, 24, 405, 112), (8, 14, 22), PURPLE, radius=18)
            draw.text((48, 40), "MuJoCo Unitree G1", font=FONT_SUB, fill=TEXT)
            draw.text((50, 76), "walking_zoo runtime target", font=FONT_SMALL, fill=MUTED)
            overlay_panel(
                img,
                "Humanoid runtime",
                [
                    ("input", "semantic/run", BLUE),
                    ("mode", "RUNNING", YELLOW),
                    ("model", "Unitree G1", GREEN),
                    ("sim", "MuJoCo", PURPLE),
                ],
                PURPLE,
            )
            frames.append(img)
    finally:
        if owns_scene:
            scene.close()
    save_gif("mujoco_unitree_g1_runtime.gif", frames, duration=70)


def runtime_flow():
    frames = []
    boxes = [
        ((44, 175, 228, 285), "Nav2", "/cmd_vel", BLUE),
        ((44, 325, 228, 435), "Teleop / VLA", "intent", PURPLE),
        ((312, 235, 548, 365), "walking_zoo", "runtime manager", GREEN),
        ((632, 165, 890, 275), "Safety", "limit / watchdog / estop", YELLOW),
        ((632, 325, 890, 435), "Adapter Hub", "mock / Unitree / future", BLUE),
    ]
    for active in range(8):
        img = Image.new("RGB", SIZE, BG)
        draw = ImageDraw.Draw(img)
        draw.text((42, 30), "walking_zoo", font=FONT_TITLE, fill=TEXT)
        draw.text((44, 78), "ROS2-native Walking Runtime & Adapter Hub", font=FONT_SUB, fill=MUTED)
        for idx, (box, title, sub, color) in enumerate(boxes):
            fill = PANEL_2 if idx <= active // 2 else PANEL
            outline = color if idx <= active // 2 else LINE
            draw_round(draw, box, fill, outline)
            text_center(draw, (box[0], box[1] + 14, box[2], box[1] + 58), title, color, FONT_SUB)
            text_center(draw, (box[0], box[1] + 58, box[2], box[3] - 12), sub, MUTED, FONT_SMALL)
        p0 = min(1.0, max(0.0, (active - 1) / 2))
        arrow(draw, (228, 230), (312, 290), BLUE, progress=p0)
        arrow(draw, (228, 380), (312, 310), PURPLE, progress=p0)
        arrow(draw, (548, 300), (632, 220), YELLOW, progress=min(1.0, max(0.0, (active - 3) / 2)))
        arrow(draw, (760, 275), (760, 325), GREEN, progress=min(1.0, max(0.0, (active - 5) / 2)))
        frames.append(img)
    save_gif("walking_zoo_runtime_flow.gif", frames, duration=760)


def nav2_bridge():
    frames = []
    labels = [
        ((56, 235, 232, 335), "Nav2", "/cmd_vel", BLUE),
        ((306, 235, 512, 335), "Bridge", "TwistStamped", GREEN),
        ((586, 235, 868, 335), "Runtime", "/walking_zoo/cmd_vel", YELLOW),
    ]
    for step in range(7):
        img = Image.new("RGB", SIZE, BG)
        draw = ImageDraw.Draw(img)
        draw.text((42, 30), "Nav2 Bridge", font=FONT_TITLE, fill=TEXT)
        draw.text((44, 78), "use Nav2 with walking robots through ROS2 topics", font=FONT_SUB, fill=MUTED)
        for idx, (box, title, sub, color) in enumerate(labels):
            draw_round(draw, box, PANEL_2 if idx <= step // 2 else PANEL, color if idx <= step // 2 else LINE)
            text_center(draw, (box[0], box[1] + 12, box[2], box[1] + 52), title, color, FONT_SUB)
            text_center(draw, (box[0], box[1] + 52, box[2], box[3] - 8), sub, MUTED, FONT_SMALL)
        arrow(draw, (232, 285), (306, 285), BLUE, progress=min(1.0, step / 2))
        arrow(draw, (512, 285), (586, 285), GREEN, progress=min(1.0, max(0.0, (step - 3) / 2)))
        draw.text((86, 394), "Nav2 owns where to go. walking_zoo owns how to walk safely.", font=FONT_BODY, fill=TEXT)
        frames.append(img)
    save_gif("nav2_cmd_vel_bridge.gif", frames, duration=760)


def safety_gate():
    frames = []
    commands = [
        ("input", "x=1.20  y=-0.90  yaw=2.00", BLUE),
        ("velocity limiter", "x=0.30  y=-0.20  yaw=0.50", YELLOW),
        ("adapter command", "sanitized velocity accepted", GREEN),
        ("estop gate", "motion blocked", RED),
    ]
    for idx, (_title, _value, _color) in enumerate(commands):
        img = Image.new("RGB", SIZE, BG)
        draw = ImageDraw.Draw(img)
        draw.text((42, 30), "Safety Pipeline", font=FONT_TITLE, fill=TEXT)
        draw.text((44, 78), "every command is checked before adapter dispatch", font=FONT_SUB, fill=MUTED)
        x0 = 90
        for j, (label, text, item_color) in enumerate(commands):
            y0 = 145 + j * 82
            fill = PANEL_2 if j <= idx else PANEL
            outline = item_color if j <= idx else LINE
            draw_round(draw, (x0, y0, 870, y0 + 58), fill, outline, radius=16)
            draw.text((120, y0 + 16), label, font=FONT_BODY, fill=item_color if j <= idx else MUTED)
            draw.text((360, y0 + 16), text, font=FONT_CODE, fill=TEXT if j <= idx else MUTED)
            if j < 3:
                arrow(draw, (480, y0 + 58), (480, y0 + 80), LINE, width=3, progress=1.0 if j < idx else 0.0)
        draw.text((100, 470), "Default limits are conservative. Real motion remains opt-in.", font=FONT_SMALL, fill=MUTED)
        frames.extend([img] * 2)
    save_gif("safety_pipeline.gif", frames, duration=620)


def adapter_hub():
    frames = []
    robots = [
        ("Mock", "works out of the box", GREEN),
        ("Unitree Go2", "profile + SDK2 stub", BLUE),
        ("Unitree G1/H1", "humanoid profiles", PURPLE),
        ("Future adapters", "Digit / Figure / ANYmal", YELLOW),
    ]
    for active in range(len(robots) + 2):
        img = Image.new("RGB", SIZE, BG)
        draw = ImageDraw.Draw(img)
        draw.text((42, 30), "Adapter Hub", font=FONT_TITLE, fill=TEXT)
        draw.text((44, 78), "bring your own robot SDK behind one contract", font=FONT_SUB, fill=MUTED)
        draw_round(draw, (330, 185, 630, 355), PANEL_2, GREEN, radius=20)
        text_center(draw, (340, 205, 620, 255), "WalkingAdapter", GREEN, FONT_SUB)
        text_center(draw, (340, 262, 620, 315), "pluginlib contract", TEXT, FONT_BODY)
        positions = [(70, 160, 260, 245), (70, 345, 260, 430), (700, 160, 890, 245), (700, 345, 890, 430)]
        for i, (robot, sub, color) in enumerate(robots):
            box = positions[i]
            visible = i < active
            draw_round(draw, box, PANEL_2 if visible else PANEL, color if visible else LINE, radius=16)
            text_center(draw, (box[0], box[1] + 8, box[2], box[1] + 45), robot, color if visible else MUTED, FONT_BODY)
            text_center(draw, (box[0] + 6, box[1] + 43, box[2] - 6, box[3] - 8), sub, MUTED, FONT_SMALL)
            if visible:
                sx = box[2] if box[0] < 330 else box[0]
                ex = 330 if box[0] < 330 else 630
                arrow(draw, (sx, (box[1] + box[3]) // 2), (ex, 270), color, width=4, progress=1.0)
        frames.append(img)
    save_gif("adapter_hub.gif", frames, duration=760)


def vla_path():
    frames = []
    steps = [
        ("VLA intent", "approach table", PURPLE),
        ("SemanticAction", "bounded command", BLUE),
        ("Nav2 / Runtime", "plan + execute", GREEN),
        ("Safety", "admit or block", YELLOW),
        ("Adapter", "robot SDK hidden", TEXT),
    ]
    for active in range(len(steps) + 1):
        img = Image.new("RGB", SIZE, BG)
        draw = ImageDraw.Draw(img)
        draw.text((42, 30), "VLA-Ready Runtime", font=FONT_TITLE, fill=TEXT)
        draw.text((44, 78), "semantic intent never bypasses safety", font=FONT_SUB, fill=MUTED)
        for i, (title, sub, color) in enumerate(steps):
            x0 = 48 + i * 178
            box = (x0, 228, x0 + 148, 330)
            visible = i < active
            draw_round(draw, box, PANEL_2 if visible else PANEL, color if visible else LINE, radius=15)
            text_center(draw, (box[0], box[1] + 12, box[2], box[1] + 50), title, color if visible else MUTED, FONT_SMALL)
            text_center(draw, (box[0] + 4, box[1] + 50, box[2] - 4, box[3] - 10), sub, TEXT if visible else MUTED, FONT_SMALL)
            if i < len(steps) - 1:
                arrow(draw, (box[2], 279), (box[2] + 30, 279), LINE, width=3, progress=1.0 if i + 1 < active else 0.0)
        draw.text((62, 405), "VLA is a command source, not a privileged controller.", font=FONT_BODY, fill=TEXT)
        frames.append(img)
    save_gif("vla_semantic_runtime.gif", frames, duration=760)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    g1_scene = MujocoUnitreeG1Scene()
    try:
        mujoco_unitree_g1_runtime(g1_scene)
        mujoco_unitree_g1_gait_gallery(g1_scene)
    finally:
        g1_scene.close()
    pybullet_laikago_runtime()
    pybullet_laikago_estop()
    runtime_flow()
    nav2_bridge()
    safety_gate()
    adapter_hub()
    vla_path()


if __name__ == "__main__":
    main()
