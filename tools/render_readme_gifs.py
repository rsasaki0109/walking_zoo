#!/usr/bin/env python3
from pathlib import Path
import math

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "assets" / "readme"
SIZE = (960, 540)
BG = (10, 16, 24)
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


def save_gif(name, frames, duration=760):
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

def draw_robot(draw, cx, ground_y, phase, scale=1.0, color=GREEN, stopped=False):
    body_w = int(96 * scale)
    body_h = int(70 * scale)
    head = int(28 * scale)
    hip_y = ground_y - int(86 * scale)
    body_box = (
        cx - body_w // 2,
        hip_y - body_h,
        cx + body_w // 2,
        hip_y,
    )
    draw_round(draw, body_box, (28, 42, 52), color, radius=int(14 * scale), width=max(2, int(3 * scale)))
    head_box = (
        cx - head,
        body_box[1] - int(42 * scale),
        cx + head,
        body_box[1] + int(14 * scale),
    )
    draw_round(draw, head_box, (32, 48, 62), color, radius=int(10 * scale), width=max(2, int(3 * scale)))
    eye_y = head_box[1] + int(23 * scale)
    draw.ellipse((cx - int(13 * scale), eye_y, cx - int(7 * scale), eye_y + int(6 * scale)), fill=BLUE)
    draw.ellipse((cx + int(7 * scale), eye_y, cx + int(13 * scale), eye_y + int(6 * scale)), fill=BLUE)

    swing = 0.0 if stopped else math.sin(phase) * int(26 * scale)
    lift = 0.0 if stopped else abs(math.sin(phase)) * int(13 * scale)
    hips = [cx - int(28 * scale), cx + int(28 * scale)]
    feet = [
        (cx - int(38 * scale) + swing, ground_y - lift),
        (cx + int(38 * scale) - swing, ground_y - (int(13 * scale) - lift)),
    ]
    knees = [
        (cx - int(36 * scale) + swing * 0.45, hip_y + int(38 * scale)),
        (cx + int(36 * scale) - swing * 0.45, hip_y + int(38 * scale)),
    ]
    for hip_x, knee, foot in zip(hips, knees, feet):
        draw.line((hip_x, hip_y, knee[0], knee[1]), fill=color, width=max(5, int(7 * scale)))
        draw.line((knee[0], knee[1], foot[0], foot[1]), fill=color, width=max(5, int(7 * scale)))
        draw.line((foot[0] - int(16 * scale), foot[1], foot[0] + int(18 * scale), foot[1]), fill=TEXT, width=max(4, int(5 * scale)))

    arm_swing = 0.0 if stopped else math.sin(phase + math.pi) * int(22 * scale)
    shoulder_y = body_box[1] + int(22 * scale)
    for side in (-1, 1):
        sx = cx + side * body_w // 2
        hand = (sx + side * int(24 * scale), shoulder_y + int(52 * scale) + side * arm_swing)
        elbow = (sx + side * int(15 * scale), shoulder_y + int(28 * scale) - side * arm_swing * 0.35)
        draw.line((sx, shoulder_y, elbow[0], elbow[1]), fill=color, width=max(4, int(6 * scale)))
        draw.line((elbow[0], elbow[1], hand[0], hand[1]), fill=color, width=max(4, int(6 * scale)))

    if not stopped:
        for offset in (70, 115, 160):
            x = cx - offset + (phase % (2 * math.pi)) / (2 * math.pi) * 44
            draw.line((x, ground_y + int(12 * scale), x + int(32 * scale), ground_y + int(12 * scale)), fill=(42, 62, 78), width=2)


def walking_robot_runtime():
    frames = []
    for i in range(18):
        progress = i / 17
        img, draw = base("walking_zoo in motion", "mock robot walking through the ROS2 runtime")
        ground_y = 410
        draw.line((40, ground_y + 4, 920, ground_y + 4), fill=(54, 74, 88), width=4)
        cx = int(160 + progress * 420)
        phase = progress * math.pi * 6
        draw_robot(draw, cx, ground_y, phase, scale=1.0, color=GREEN)

        stages = [
            ((55, 455, 235, 510), "Nav2 /cmd_vel", BLUE),
            ((270, 455, 450, 510), "Safety passed", YELLOW),
            ((485, 455, 665, 510), "Mock adapter", GREEN),
            ((700, 455, 900, 510), "WalkingState=WALKING", PURPLE),
        ]
        for idx, (box, label, color) in enumerate(stages):
            active = progress >= idx / len(stages) - 0.02
            draw_round(draw, box, PANEL_2 if active else PANEL, color if active else LINE, radius=13)
            text_center(draw, box, label, color if active else MUTED, FONT_SMALL)
            if idx < len(stages) - 1:
                arrow(draw, (box[2], 482), (stages[idx + 1][0][0], 482), LINE, width=3, progress=1.0 if active else 0.0)

        draw_round(draw, (650, 145, 885, 290), PANEL_2, GREEN, radius=18)
        text_center(draw, (662, 160, 873, 205), "Live mock demo", GREEN, FONT_BODY)
        draw.text((678, 214), "adapter_connected: true", font=FONT_SMALL, fill=TEXT)
        draw.text((678, 242), "locomotion_state: WALKING", font=FONT_SMALL, fill=TEXT)
        frames.append(img)
    save_gif("walking_robot_runtime.gif", frames, duration=90)


def walking_robot_estop():
    frames = []
    for i in range(16):
        stopped = i >= 10
        img, draw = base("Safety gate in action", "the robot walks, then e-stop blocks motion")
        ground_y = 405
        draw.line((52, ground_y + 4, 908, ground_y + 4), fill=(54, 74, 88), width=4)
        cx = 220 + min(i, 10) * 38
        color = RED if stopped else GREEN
        draw_robot(draw, cx, ground_y, i * 0.75, scale=1.0, color=color, stopped=stopped)

        draw_round(draw, (610, 155, 860, 275), PANEL_2, RED if stopped else GREEN, radius=18)
        if stopped:
            text_center(draw, (620, 170, 850, 215), "E-STOP ACTIVE", RED, FONT_SUB)
            text_center(draw, (620, 225, 850, 260), "adapter command blocked", TEXT, FONT_SMALL)
        else:
            text_center(draw, (620, 170, 850, 215), "Walking", GREEN, FONT_SUB)
            text_center(draw, (620, 225, 850, 260), "safe command accepted", TEXT, FONT_SMALL)

        draw_round(draw, (92, 455, 868, 510), PANEL, LINE, radius=14)
        label = "/walking_zoo/state: ESTOPPED" if stopped else "/walking_zoo/state: WALKING"
        draw.text((120, 472), label, font=FONT_CODE, fill=RED if stopped else GREEN)
        frames.append(img)
    save_gif("walking_robot_estop.gif", frames, duration=110)


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
        img, draw = base(
            "walking_zoo",
            "ROS2-native Walking Runtime & Adapter Hub",
        )
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
    save_gif("walking_zoo_runtime_flow.gif", frames)


def mock_runtime_state():
    frames = []
    states = [
        ("Lifecycle configure", "adapter: mock", "state: IDLE", BLUE),
        ("Lifecycle activate", "adapter: mock", "state: STANDING", GREEN),
        ("/cmd_vel x=0.2 z=0.1", "safety: passed", "state: WALKING", GREEN),
        ("/walking_zoo/estop", "safety: blocked", "state: ESTOPPED", RED),
    ]
    for i, state in enumerate(states):
        img, draw = base("Mock Runtime Demo", "real ROS2 runtime behavior without hardware")
        draw_round(draw, (58, 135, 902, 455), (3, 8, 14), LINE, radius=16)
        draw.text((88, 160), "$ ros2 launch walking_zoo_bringup mock_runtime.launch.py", font=FONT_CODE, fill=GREEN)
        lines = [
            "[runtime] loaded walking_zoo_mock_adapter/MockWalkingAdapter",
            f"[runtime] {state[0]}",
            f"[adapter] {state[1]}",
            f"[state]   {state[2]}",
        ]
        y = 210
        for j, line in enumerate(lines):
            color = state[3] if j == len(lines) - 1 else TEXT
            draw.text((88, y), line, font=FONT_CODE, fill=color)
            y += 42
        draw_round(draw, (670, 170, 855, 365), PANEL_2, state[3], radius=18)
        text_center(draw, (680, 185, 845, 230), "WalkingState", TEXT, FONT_BODY)
        text_center(draw, (680, 235, 845, 315), state[2].replace("state: ", ""), state[3], FONT_TITLE)
        frames.extend([img] * 2)
    save_gif("mock_runtime_state.gif", frames, duration=620)


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
    save_gif("nav2_cmd_vel_bridge.gif", frames)


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
        for j, (t, v, c) in enumerate(commands):
            y0 = 145 + j * 82
            fill = PANEL_2 if j <= idx else PANEL
            outline = c if j <= idx else LINE
            draw_round(draw, (x0, y0, 870, y0 + 58), fill, outline, radius=16)
            draw.text((120, y0 + 16), t, font=FONT_BODY, fill=c if j <= idx else MUTED)
            draw.text((360, y0 + 16), v, font=FONT_CODE, fill=TEXT if j <= idx else MUTED)
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
    save_gif("adapter_hub.gif", frames)


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
    save_gif("vla_semantic_runtime.gif", frames)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    walking_robot_runtime()
    walking_robot_estop()
    runtime_flow()
    mock_runtime_state()
    nav2_bridge()
    safety_gate()
    adapter_hub()
    vla_path()


if __name__ == "__main__":
    main()
