#!/usr/bin/env python3
"""Render README GIFs from lightweight walking simulations.

These assets are documentation-only. They are generated from deterministic
gait, footstep, and runtime-state simulations so the README does not depend on
hand-animated toy poses or a heavyweight simulator.
"""

from pathlib import Path
import math

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "assets" / "readme"
SIZE = (960, 540)

BG = (9, 14, 22)
PANEL = (18, 28, 40)
PANEL_2 = (26, 40, 56)
GRID = (33, 47, 62)
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


def world_to_px(x, z, camera_x, ground_y=430, scale=205):
    return int(160 + (x - camera_x) * scale), int(ground_y - z * scale)


def two_link_ik(hip, foot, l1, l2, knee_sign=1.0):
    hx, hz = hip
    fx, fz = foot
    dx = fx - hx
    dz = fz - hz
    distance = float(np.hypot(dx, dz))
    distance = max(1e-5, min(distance, l1 + l2 - 1e-5))
    midpoint = ((hx + fx) * 0.5, (hz + fz) * 0.5)
    a = (l1 * l1 - l2 * l2 + distance * distance) / (2.0 * distance)
    h = math.sqrt(max(0.0, l1 * l1 - a * a))
    ux = dx / distance
    uz = dz / distance
    px = hx + a * ux
    pz = hz + a * uz
    knee = (px - knee_sign * h * uz, pz + knee_sign * h * ux)
    return knee


def gait_foot_relative(t, offset, step_length=0.34, step_time=0.62, duty=0.63):
    phase = (t / step_time + offset) % 1.0
    if phase < duty:
        s = phase / duty
        x = 0.5 * step_length - step_length * s
        z = 0.0
        contact = True
    else:
        s = (phase - duty) / (1.0 - duty)
        x = -0.5 * step_length + step_length * (3.0 * s * s - 2.0 * s * s * s)
        z = 0.095 * math.sin(math.pi * s)
        contact = False
    return x, z, contact


def draw_grid_ground(draw, ground_y=430):
    for x in range(40, 920, 60):
        draw.line((x, 134, x, ground_y + 20), fill=GRID, width=1)
    for y in range(160, ground_y + 1, 48):
        draw.line((40, y, 920, y), fill=GRID, width=1)
    draw.line((40, ground_y, 920, ground_y), fill=(92, 116, 132), width=4)


def draw_link(draw, p0, p1, color, width=8):
    draw.line((p0[0], p0[1], p1[0], p1[1]), fill=color, width=width)
    r = max(3, width // 2)
    draw.ellipse((p0[0] - r, p0[1] - r, p0[0] + r, p0[1] + r), fill=color)
    draw.ellipse((p1[0] - r, p1[1] - r, p1[0] + r, p1[1] + r), fill=color)


def draw_biped(draw, t, root_x, camera_x, color=GREEN, stopped=False):
    hip_z = 0.88 + (0.0 if stopped else 0.025 * math.sin(2.0 * math.pi * t / 0.62))
    torso_z = hip_z + 0.34
    head_z = torso_z + 0.21
    l1 = 0.43
    l2 = 0.44
    foot_offsets = [("L", 0.00, -0.045, BLUE), ("R", 0.50, 0.045, GREEN)]
    hip_world = (root_x, hip_z)
    hip_px = world_to_px(*hip_world, camera_x)
    torso_px = world_to_px(root_x + 0.02 * math.sin(t * 2.0), torso_z, camera_x)
    head_px = world_to_px(root_x + 0.03 * math.sin(t * 2.0), head_z, camera_x)

    contacts = []
    for label, offset, lateral, leg_color in foot_offsets:
        rel_x, rel_z, contact = gait_foot_relative(t, offset)
        if stopped:
            rel_x = -0.08 if label == "L" else 0.11
            rel_z = 0.0
            contact = True
        foot_world = (root_x + rel_x, rel_z)
        knee_world = two_link_ik(hip_world, foot_world, l1, l2, knee_sign=-1.0)
        foot_px = world_to_px(*foot_world, camera_x)
        knee_px = world_to_px(*knee_world, camera_x)
        shade = leg_color if contact else (120, 150, 180)
        draw_link(draw, hip_px, knee_px, shade, width=8)
        draw_link(draw, knee_px, foot_px, shade, width=8)
        draw.line((foot_px[0] - 22, foot_px[1], foot_px[0] + 26, foot_px[1]), fill=TEXT, width=5)
        if contact:
            contacts.append((foot_px[0], foot_px[1], label))

    draw_link(draw, hip_px, torso_px, color, width=12)
    draw_round(
        draw,
        (torso_px[0] - 42, torso_px[1] - 60, torso_px[0] + 44, torso_px[1] + 36),
        (28, 42, 54),
        color,
        radius=14,
        width=3,
    )
    draw_round(
        draw,
        (head_px[0] - 28, head_px[1] - 24, head_px[0] + 28, head_px[1] + 28),
        (34, 50, 64),
        color,
        radius=11,
        width=3,
    )
    draw.ellipse((torso_px[0] - 7, torso_px[1] - 6, torso_px[0] + 7, torso_px[1] + 8), fill=YELLOW)
    draw.ellipse((head_px[0] - 14, head_px[1] - 2, head_px[0] - 8, head_px[1] + 4), fill=BLUE)
    draw.ellipse((head_px[0] + 8, head_px[1] - 2, head_px[0] + 14, head_px[1] + 4), fill=BLUE)
    for x, y, label in contacts:
        draw.text((x - 6, y + 9), label, font=FONT_SMALL, fill=MUTED)


def draw_state_panel(draw, title, rows, accent=GREEN):
    draw_round(draw, (642, 136, 890, 314), PANEL_2, accent, radius=18)
    text_center(draw, (662, 150, 870, 194), title, accent, FONT_BODY)
    y = 205
    for key, value, color in rows:
        draw.text((668, y), key, font=FONT_SMALL, fill=MUTED)
        draw.text((790, y), value, font=FONT_SMALL, fill=color)
        y += 28


def simulated_biped_runtime():
    frames = []
    ts = np.linspace(0.0, 3.8, 38)
    speed = 0.34
    for t in ts:
        root_x = speed * t
        camera_x = max(0.0, root_x - 0.32)
        img, draw = base("walking_zoo simulation", "kinematic biped gait driven by runtime state")
        draw_grid_ground(draw)
        draw_biped(draw, t, root_x, camera_x, GREEN)
        draw_state_panel(
            draw,
            "ROS2 runtime",
            [
                ("input", "/cmd_vel", BLUE),
                ("safety", "passed", YELLOW),
                ("adapter", "mock", GREEN),
                ("state", "WALKING", PURPLE),
            ],
            GREEN,
        )
        draw.text((58, 464), "Simulated gait: alternating support, swing foot trajectory, two-link IK, COM marker", font=FONT_SMALL, fill=MUTED)
        frames.append(img)
    save_gif("simulated_biped_runtime.gif", frames, duration=80)


def simulated_estop_stop():
    frames = []
    ts = np.linspace(0.0, 4.0, 40)
    speed = 0.34
    estop_t = 2.45
    decel = 1.1
    for t in ts:
        if t <= estop_t:
            root_x = speed * t
            velocity = speed
            stopped = False
        else:
            tau = min(t - estop_t, speed / decel)
            root_x = speed * estop_t + speed * tau - 0.5 * decel * tau * tau
            velocity = max(0.0, speed - decel * tau)
            stopped = velocity < 0.03
        camera_x = max(0.0, root_x - 0.32)
        img, draw = base("e-stop simulation", "runtime gate blocks motion and the gait settles")
        draw_grid_ground(draw)
        draw_biped(draw, t, root_x, camera_x, RED if stopped else GREEN, stopped=stopped)
        draw_state_panel(
            draw,
            "Safety gate",
            [
                ("velocity", f"{velocity:.2f} m/s", GREEN if not stopped else RED),
                ("estop", "active" if t >= estop_t else "false", RED if t >= estop_t else GREEN),
                ("adapter", "blocked" if t >= estop_t else "accepted", RED if t >= estop_t else GREEN),
                ("state", "ESTOPPED" if stopped else "WALKING", RED if stopped else PURPLE),
            ],
            RED if t >= estop_t else GREEN,
        )
        draw.text((58, 464), "Simulated stop profile: command gate closes, velocity decays, feet return to support", font=FONT_SMALL, fill=MUTED)
        frames.append(img)
    save_gif("simulated_estop_stop.gif", frames, duration=80)


def draw_quadruped(draw, t, root_x, camera_x):
    ground_y = 430
    body_z = 0.54 + 0.015 * math.sin(4.0 * math.pi * t)
    body = world_to_px(root_x, body_z, camera_x, ground_y, 260)
    draw_round(draw, (body[0] - 92, body[1] - 42, body[0] + 92, body[1] + 36), (28, 42, 54), GREEN, radius=18, width=3)
    draw_round(draw, (body[0] + 74, body[1] - 30, body[0] + 122, body[1] + 18), (34, 50, 64), GREEN, radius=12, width=3)
    legs = [
        ("FL", 0.00, 0.30, BLUE),
        ("RR", 0.00, -0.30, BLUE),
        ("FR", 0.50, 0.30, PURPLE),
        ("RL", 0.50, -0.30, PURPLE),
    ]
    for name, offset, rel_body_x, color in legs:
        rel_x, foot_z, contact = gait_foot_relative(t, offset, step_length=0.26, step_time=0.44, duty=0.58)
        hip_world = (root_x + rel_body_x, body_z - 0.07)
        foot_world = (root_x + rel_body_x + rel_x, foot_z)
        knee_world = two_link_ik(hip_world, foot_world, 0.30, 0.32, knee_sign=-1.0)
        hip_px = world_to_px(*hip_world, camera_x, ground_y, 260)
        knee_px = world_to_px(*knee_world, camera_x, ground_y, 260)
        foot_px = world_to_px(*foot_world, camera_x, ground_y, 260)
        leg_color = color if contact else (118, 145, 172)
        draw_link(draw, hip_px, knee_px, leg_color, 6)
        draw_link(draw, knee_px, foot_px, leg_color, 6)
        draw.line((foot_px[0] - 16, foot_px[1], foot_px[0] + 20, foot_px[1]), fill=TEXT, width=4)
        if contact:
            draw.text((foot_px[0] - 12, foot_px[1] + 7), name, font=FONT_SMALL, fill=MUTED)


def simulated_quadruped_trot():
    frames = []
    ts = np.linspace(0.0, 2.8, 34)
    speed = 0.48
    for t in ts:
        root_x = speed * t
        camera_x = max(0.0, root_x - 0.35)
        img, draw = base("quadruped trot simulation", "Go2-style velocity command through the same runtime path")
        draw_grid_ground(draw)
        draw_quadruped(draw, t, root_x, camera_x)
        draw_state_panel(
            draw,
            "RobotProfile",
            [
                ("family", "quadruped", BLUE),
                ("gait", "trot", GREEN),
                ("cmd", "vx=0.48", YELLOW),
                ("state", "WALKING", PURPLE),
            ],
            GREEN,
        )
        frames.append(img)
    save_gif("simulated_quadruped_trot.gif", frames, duration=80)


def simulated_footstep_plan():
    frames = []
    footsteps = []
    for i in range(10):
        footsteps.append((0.18 + i * 0.18, 0.10 if i % 2 == 0 else -0.10, "L" if i % 2 == 0 else "R"))
    for frame in range(32):
        progress = frame / 31
        img, draw = base("footstep plan simulation", "preview COM and alternating support contacts")
        draw_round(draw, (58, 130, 902, 455), (7, 12, 18), LINE, radius=16)
        scale = 350
        ox, oy = 120, 292
        for gx in np.linspace(0.0, 2.0, 11):
            x = int(ox + gx * scale)
            draw.line((x, 150, x, 430), fill=GRID, width=1)
        for gy in np.linspace(-0.3, 0.3, 7):
            y = int(oy - gy * scale)
            draw.line((80, y, 870, y), fill=GRID, width=1)
        draw.line((80, oy, 870, oy), fill=(90, 112, 130), width=2)
        visible_count = max(1, int(progress * len(footsteps)))
        for idx, (x_m, y_m, side) in enumerate(footsteps[:visible_count]):
            px = int(ox + x_m * scale)
            py = int(oy - y_m * scale)
            color = BLUE if side == "L" else GREEN
            draw_round(draw, (px - 28, py - 14, px + 28, py + 14), PANEL_2, color, radius=7)
            text_center(draw, (px - 28, py - 14, px + 28, py + 14), side, color, FONT_SMALL)
            if idx > 0:
                prev = footsteps[idx - 1]
                draw.line((int(ox + prev[0] * scale), int(oy - prev[1] * scale), px, py), fill=LINE, width=2)
        com_x = 0.18 + progress * (footsteps[-1][0] - 0.18)
        com_y = 0.04 * math.sin(progress * math.pi * 5)
        com = (int(ox + com_x * scale), int(oy - com_y * scale))
        draw.ellipse((com[0] - 10, com[1] - 10, com[0] + 10, com[1] + 10), fill=YELLOW)
        draw.text((com[0] + 14, com[1] - 9), "COM", font=FONT_SMALL, fill=YELLOW)
        draw_state_panel(
            draw,
            "Runtime API",
            [
                ("action", "FootstepPlan", BLUE),
                ("support", "alternating", GREEN),
                ("preview", "COM trace", YELLOW),
                ("safety", "feasibility", PURPLE),
            ],
            BLUE,
        )
        frames.append(img)
    save_gif("simulated_footstep_plan.gif", frames, duration=90)


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
        img, draw = base("walking_zoo", "ROS2-native Walking Runtime & Adapter Hub")
        for idx, (box, title, sub, color) in enumerate(boxes):
            fill = PANEL_2 if idx <= active // 2 else PANEL
            outline = color if idx <= active // 2 else LINE
            draw_round(draw, box, fill, outline)
            text_center(draw, (box[0], box[1] + 14, box[2], box[1] + 58), title, color, FONT_SUB)
            text_center(draw, (box[0], box[1] + 58, box[2], box[3] - 12), sub, MUTED, FONT_SMALL)
        p = min(1.0, max(0.0, (active - 1) / 2))
        arrow(draw, (228, 230), (312, 290), BLUE, progress=p)
        arrow(draw, (228, 380), (312, 310), PURPLE, progress=p)
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
        img, draw = base("Nav2 Bridge", "use Nav2 with walking robots through ROS2 topics")
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
    for idx, (title, value, color) in enumerate(commands):
        img, draw = base("Safety Pipeline", "every command is checked before adapter dispatch")
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
        img, draw = base("Adapter Hub", "bring your own robot SDK behind one contract")
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
        img, draw = base("VLA-Ready Runtime", "semantic intent never bypasses safety")
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
    simulated_biped_runtime()
    simulated_estop_stop()
    simulated_quadruped_trot()
    simulated_footstep_plan()
    runtime_flow()
    nav2_bridge()
    safety_gate()
    adapter_hub()
    vla_path()


if __name__ == "__main__":
    main()
