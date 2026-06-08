#!/usr/bin/env python3
"""MuJoCo Unitree G1 software-in-the-loop sim driven by a gait_lab controller.

This is the companion sim behind ``locomotion_ros2_gait_lab_sil/GaitLabSilAdapter``.
The C++ adapter is a thin ROS bridge inside the runtime; the actual physics and
the learned gait live *here*, so MuJoCo stays an optional dependency of this one
node (never of locomotion_ros2 itself).

It closes the experiment → product loop: the reinforcement-learned ``rl-residual``
gait validated in ``experiments/gait_lab`` now runs as a live simulated robot
behind the real runtime/safety pipeline. The adapter forwards the runtime's
safety-filtered velocity (and lifecycle control) here; this node steps the G1
through the gait_lab controller and publishes the resulting WalkingState back, so
the runtime reports — and the visualizer shows — a robot that genuinely walks.

    # bridge topics (published/subscribed by the C++ adapter):
    #   <- /gait_lab_sil/command_velocity  (geometry_msgs/TwistStamped)
    #   <- /gait_lab_sil/control           (std_msgs/String lifecycle signal)
    #   -> /gait_lab_sil/robot_state       (locomotion_ros2_msgs/WalkingState)

    ros2 run locomotion_ros2_examples gait_lab_sil_sim.py --ros-args -p controller:=rl-residual

gait_lab lives outside the ROS packages; point this node at it with
LOCOMOTION_ROS2_GAIT_LAB_PATH (it also tries the in-repo location automatically).
"""

import os
import sys
from pathlib import Path

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, TwistStamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import TransformBroadcaster
from locomotion_ros2_msgs.msg import WalkingState


LEG_ACTUATORS = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
]


def _locate_gait_lab() -> str:
    candidates = []
    env = os.environ.get("LOCOMOTION_ROS2_GAIT_LAB_PATH")
    if env:
        candidates.append(Path(env))
    # In-repo location relative to this source file (src/<pkg>/scripts/ -> repo root).
    here = Path(__file__).resolve()
    for up in (3, 4, 2):
        if len(here.parents) > up:
            candidates.append(here.parents[up] / "experiments" / "gait_lab")
    candidates.append(Path.cwd() / "experiments" / "gait_lab")
    for c in candidates:
        if (c / "gait_lab" / "__init__.py").exists():
            return str(c)
    raise RuntimeError(
        "Could not find experiments/gait_lab. Set LOCOMOTION_ROS2_GAIT_LAB_PATH to the "
        "gait_lab checkout (the directory containing the 'gait_lab' package)."
    )


class GaitLabSilSim(Node):
    """Steps a MuJoCo G1 through a gait_lab controller, bridged to the runtime."""

    def __init__(self):
        super().__init__("gait_lab_sil_sim")
        self.declare_parameter("controller", "rl-residual")
        self.declare_parameter("substeps", 10)        # mj_steps per control tick
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("move_threshold", 0.02)  # |cmd| below this = hold
        self.declare_parameter("fall_height", 0.5)
        self.declare_parameter("render", False)
        # Capture rendered frames to disk (for the live ROS-driven filmstrip): a
        # directory to write numbered PNGs into, and how many control ticks to skip
        # between saved frames.
        self.declare_parameter("save_frames_dir", "")
        self.declare_parameter("frame_stride", 4)
        # Odometry / TF: Nav2 needs a continuous odom->base_link transform and an
        # /odom topic to navigate. The MuJoCo base pose is real world odometry.
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        # When true, physics runs here and a separate gait_lab_sil_gait_controller
        # node publishes ros2_control joint commands (B3 split path).
        self.declare_parameter("ros2_control_split", False)
        self.declare_parameter(
            "joint_commands_topic", "/gait_lab_sil/ros2_control/joint_commands")
        self.declare_parameter(
            "joint_states_topic", "/gait_lab_sil/ros2_control/joint_states")
        # When true, each joint command runs all substeps (ros2_control forward path).
        self.declare_parameter("batch_substeps_per_command", False)
        self.declare_parameter("steer_yaw_ramp_rate", 0.15)

        self.ros2_control_split = bool(self.get_parameter("ros2_control_split").value)
        self.batch_substeps_per_command = bool(
            self.get_parameter("batch_substeps_per_command").value)
        controller_name = self.get_parameter("controller").value
        self.substeps = int(self.get_parameter("substeps").value)
        self.move_threshold = float(self.get_parameter("move_threshold").value)
        self.fall_height = float(self.get_parameter("fall_height").value)

        sys.path.insert(0, _locate_gait_lab())
        os.environ.setdefault("MUJOCO_GL", "egl")
        import mujoco  # noqa: E402
        from gait_lab import CONTROLLERS, Command, G1Model  # noqa: E402

        self._mujoco = mujoco
        self._Command = Command
        self.model = G1Model()
        controllers = {c.name: c for c in CONTROLLERS()}
        if not self.ros2_control_split:
            if controller_name not in controllers:
                raise RuntimeError(
                    f"unknown controller {controller_name!r}; "
                    f"choices: {sorted(controllers)}")
            self.controller = controllers[controller_name]
        else:
            self.controller = None
        self.controller_name = controller_name
        self.steer_shaping = controller_name.startswith("rl-steerable")
        self.steer_yaw_ramp_rate = float(
            self.get_parameter("steer_yaw_ramp_rate").value)
        self._shaped_yaw = 0.0

        # Robot/lifecycle state.
        self.active = False
        self.estop = False
        self.fallen = False
        self.cmd_speed = 0.0
        self.cmd_lateral = 0.0
        self.cmd_yaw = 0.0
        # A velocity command persists until the next command or an explicit stop —
        # the runtime sends it once and expects the adapter to hold it (the
        # stale-command watchdog lives in the runtime's safety pipeline, not here).
        self.moving = False
        # gait_t is a *gait-local* clock: it advances only while walking and resets
        # to 0 each time walking (re)starts, so the controller's CPG phase always
        # begins from the same standing condition it was trained/verified under.
        # Advancing it during a stand-hold (as a naive global clock would) desyncs
        # the learned policy's phase from reality and topples the robot.
        self.gait_t = 0.0
        self.walking_prev = False
        # A pending external shove (world-frame base velocity kick, m/s), applied
        # once on the next walking tick — for push-recovery benchmarking through
        # the runtime. None = no shove pending.
        self._pending_push = None
        self.stand = self.model.stand_targets.copy()
        self.model.reset()

        self.renderer = None
        if bool(self.get_parameter("render").value):
            self._init_render()

        from rclpy.qos import QoSProfile, DurabilityPolicy
        self.create_subscription(
            TwistStamped, "gait_lab_sil/command_velocity", self._on_cmd, 10)
        # Latched control: pick up the last lifecycle signal (e.g. "activate" sent
        # while MuJoCo was still loading) on join.
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, "gait_lab_sil/control", self._on_control, latched)
        # External shove for push-recovery benchmarking: a world-frame base
        # velocity kick (m/s) applied on the next walking tick.
        self.create_subscription(Vector3, "gait_lab_sil/push", self._on_push, 10)
        self.state_pub = self.create_publisher(WalkingState, "gait_lab_sil/robot_state", 10)

        self._joint_commands = None
        if self.ros2_control_split:
            commands_topic = self.get_parameter("joint_commands_topic").value
            states_topic = self.get_parameter("joint_states_topic").value
            self.create_subscription(
                JointState, commands_topic, self._on_joint_commands, 10)
            self.joint_state_pub = self.create_publisher(JointState, states_topic, 10)
            snapshot_qos = QoSProfile(depth=1)
            snapshot_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.physics_snapshot_pub = self.create_publisher(
                Float64MultiArray, "gait_lab_sil/physics_snapshot", snapshot_qos)
        else:
            states_topic = None

        self.publish_odom = bool(self.get_parameter("publish_odom").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, "gait_lab_sil/odom", 10)
            self.tf_broadcaster = TransformBroadcaster(self)

        hz = float(self.get_parameter("control_hz").value)
        self._split_steps_this_tick = 0
        if self.ros2_control_split:
            self.timer = None
            # Nav2 needs a continuous odom->base_link TF; split mode only steps
            # physics on joint commands, so publish state/odom on a timer too.
            self._split_publish_timer = self.create_timer(
                1.0 / hz, self._split_periodic_publish)
            self._publish_ros2_control_joint_states()
            if self.publish_odom:
                self._publish_odom()
        else:
            self.timer = self.create_timer(1.0 / hz, self._tick)
        mode = "ros2_control split" if self.ros2_control_split else "monolithic"
        self.get_logger().info(
            f"gait_lab SIL sim up ({mode}): controller={controller_name}, "
            f"{self.substeps} mj_steps/tick @ {hz:.0f} Hz")

    # -- bridge inputs -----------------------------------------------------
    def _on_cmd(self, msg: TwistStamped):
        self.cmd_speed = msg.twist.linear.x
        self.cmd_lateral = msg.twist.linear.y
        self.cmd_yaw = msg.twist.angular.z
        mag = abs(self.cmd_speed) + abs(self.cmd_lateral) + abs(self.cmd_yaw)
        self.moving = mag > self.move_threshold

    def _on_push(self, msg: Vector3):
        # Latch the shove; applied on the next walking tick (after any rising-edge
        # rehome, so it is not immediately zeroed).
        self._pending_push = (float(msg.x), float(msg.y))
        self.get_logger().info(
            f"gait_lab SIL push queued: ({msg.x:+.2f}, {msg.y:+.2f}) m/s base kick")

    def _on_control(self, msg: String):
        signal = msg.data
        if signal == "activate":
            self._reset_robot()
            self.active = True
            self.estop = False
            if self.ros2_control_split:
                self._publish_ros2_control_joint_states()
                if self.publish_odom:
                    self._publish_odom()
                self._publish_state(False)
        elif signal == "deactivate":
            self.active = False
            self.moving = False
        elif signal == "estop":
            self.estop = True
            self.moving = False
        elif signal == "clear_fault":
            self.estop = False
            if self.fallen:                # a cleared fault re-stands the robot
                self._reset_robot()
        elif signal in ("stop_normal", "stop_quick"):
            self.cmd_speed = self.cmd_lateral = self.cmd_yaw = 0.0
            self.moving = False
            self._shaped_yaw = 0.0
        self.get_logger().info(f"gait_lab SIL control: {signal}")

    def _on_joint_commands(self, msg: JointState):
        if not self.ros2_control_split:
            self._joint_commands = dict(zip(msg.name, msg.position))
            return
        self._joint_commands = dict(zip(msg.name, msg.position))
        self._step_ros2_control_split()

    def _reset_robot(self):
        self.model.reset()
        if self.controller is not None:
            self.controller.reset(self.model)
        self.gait_t = 0.0
        self.walking_prev = False
        self.fallen = False
        self._shaped_yaw = 0.0
        self._joint_commands = None

    def _rehome_posture(self):
        """Reset joints/orientation/velocity to the stand keyframe, keeping x/y."""
        d = self.model.data
        x, y = float(d.qpos[0]), float(d.qpos[1])
        d.qpos[:] = self.model.stand_qpos
        d.qpos[0], d.qpos[1] = x, y
        d.qvel[:] = 0.0
        self._mujoco.mj_forward(self.model.model, d)

    # -- sim loop ----------------------------------------------------------
    def _commanded_to_move(self) -> bool:
        return self.moving

    def _effective_command(self):
        raw_vx = float(self.cmd_speed)
        raw_yaw = float(self.cmd_yaw)
        if not self.steer_shaping or not self.moving:
            self._shaped_yaw = 0.0
            return self._Command(forward_speed=raw_vx, yaw_rate=raw_yaw)
        step_dt = self.model.timestep
        max_delta = self.steer_yaw_ramp_rate * step_dt
        delta = max(-max_delta, min(max_delta, raw_yaw - self._shaped_yaw))
        self._shaped_yaw += delta
        return self._Command(forward_speed=raw_vx, yaw_rate=self._shaped_yaw)

    def _split_periodic_publish(self):
        """Keep joint_states, odom, TF, and WalkingState fresh between commands."""
        self._publish_ros2_control_joint_states()
        if self.publish_odom:
            self._publish_odom()
        walking = (self.active and not self.estop and not self.fallen
                   and self._commanded_to_move())
        self._publish_state(walking)

    def _publish_ros2_control_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(LEG_ACTUATORS)
        positions = []
        velocities = []
        m = self.model.model
        for name in LEG_ACTUATORS:
            act = self.model.actuator(name)
            qadr = self.model._act_qadr[act]
            dofadr = m.jnt_dofadr[m.actuator_trnid[act, 0]]
            positions.append(float(self.model.data.qpos[qadr]))
            velocities.append(float(self.model.data.qvel[dofadr]))
        msg.position = positions
        msg.velocity = velocities
        self.joint_state_pub.publish(msg)
        snap = Float64MultiArray()
        snap.data = (
            [float(v) for v in self.model.data.qpos]
            + [float(v) for v in self.model.data.qvel]
        )
        self.physics_snapshot_pub.publish(snap)

    def _tick(self):
        walking = self.active and not self.estop and not self.fallen \
            and self._commanded_to_move()
        if not self.ros2_control_split:
            self._tick_monolithic(walking)

        if float(self.model.data.qpos[2]) < self.fall_height:
            self.fallen = True
        if self.renderer is not None:
            self._render()
        if self.publish_odom:
            self._publish_odom()
        self._publish_state(walking)

    def _tick_monolithic(self, walking: bool):
        if walking and not self.walking_prev:
            self._rehome_posture()
            self.controller.reset(self.model)
            self.gait_t = 0.0
            self._shaped_yaw = 0.0
        self.walking_prev = walking

        if self._pending_push is not None and walking:
            kx, ky = self._pending_push
            self.model.data.qvel[0] += kx
            self.model.data.qvel[1] += ky
            self._pending_push = None
            self.get_logger().info(
                f"gait_lab SIL push applied: ({kx:+.2f}, {ky:+.2f}) m/s")

        for _ in range(self.substeps):
            if walking:
                cmd = self._effective_command()
                obs = self.model.observe(self.gait_t)
                ctrl = self.controller.update(obs, cmd)
                self.gait_t += self.model.timestep
            else:
                ctrl = self.stand
            self.model.data.ctrl[:] = ctrl
            self.model.step()

    def _apply_split_joint_commands(self):
        if self._joint_commands is None:
            self.model.data.ctrl[:] = self.stand
            return
        for name in LEG_ACTUATORS:
            if name in self._joint_commands and self.model.has_actuator(name):
                self.model.data.ctrl[self.model.actuator(name)] = float(
                    self._joint_commands[name])

    def _finish_ros2_control_tick(self, walking: bool):
        if float(self.model.data.qpos[2]) < self.fall_height:
            self.fallen = True
        self._publish_ros2_control_joint_states()
        if self.renderer is not None:
            self._render()
        if self.publish_odom:
            self._publish_odom()
        self._publish_state(walking)

    def _step_ros2_control_split(self):
        walking = self.active and not self.estop and not self.fallen \
            and self._commanded_to_move()
        if self.batch_substeps_per_command:
            if walking and not self.walking_prev:
                self._rehome_posture()
            self.walking_prev = walking
            if self._pending_push is not None and walking:
                kx, ky = self._pending_push
                self.model.data.qvel[0] += kx
                self.model.data.qvel[1] += ky
                self._pending_push = None
                self.get_logger().info(
                    f"gait_lab SIL push applied: ({kx:+.2f}, {ky:+.2f}) m/s")
            self._apply_split_joint_commands()
            for _ in range(self.substeps):
                self.model.step()
            self._finish_ros2_control_tick(walking)
            return

        if walking and not self.walking_prev and self._split_steps_this_tick == 0:
            self._rehome_posture()
        if self._split_steps_this_tick == 0:
            self.walking_prev = walking
            if self._pending_push is not None and walking:
                kx, ky = self._pending_push
                self.model.data.qvel[0] += kx
                self.model.data.qvel[1] += ky
                self._pending_push = None
                self.get_logger().info(
                    f"gait_lab SIL push applied: ({kx:+.2f}, {ky:+.2f}) m/s")
        self._apply_split_joint_commands()
        self.model.step()
        self._split_steps_this_tick += 1
        if self._split_steps_this_tick >= self.substeps:
            self._split_steps_this_tick = 0
            self._finish_ros2_control_tick(walking)

    # -- odometry / TF -----------------------------------------------------
    def _publish_odom(self):
        """Publish the MuJoCo base pose as odom->base_link TF + an Odometry msg.

        The free-joint base qpos (world x/y/z + quaternion) is genuine odometry;
        Nav2 localises and navigates off this. Linear velocity is rotated into the
        base frame (the Odometry twist convention)."""
        d = self.model.data
        px, py, pz = float(d.qpos[0]), float(d.qpos[1]), float(d.qpos[2])
        qw, qx, qy, qz = (float(d.qpos[3]), float(d.qpos[4]),
                          float(d.qpos[5]), float(d.qpos[6]))
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        wvx, wvy = float(d.qvel[0]), float(d.qvel[1])
        cy, sy = math.cos(-yaw), math.sin(-yaw)
        bvx = cy * wvx - sy * wvy        # world linear vel -> base frame
        bvy = sy * wvx + cy * wvy
        wz = float(d.qvel[5])
        stamp = self.get_clock().now().to_msg()

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = px
        tf.transform.translation.y = py
        tf.transform.translation.z = pz
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = bvx
        odom.twist.twist.linear.y = bvy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

    # -- bridge output -----------------------------------------------------
    def _publish_state(self, walking: bool):
        s = WalkingState()
        s.header.stamp = self.get_clock().now().to_msg()
        base_h = float(self.model.data.qpos[2])
        rpy = self.model.observe(self.gait_t).torso_rpy
        # Cast through float(): rpy is a numpy array, so naive comparisons yield
        # numpy.bool_, which the ROS message C extension rejects (it asserts a
        # native Python bool on the wire).
        # Walking gaits oscillate more than a static stand; keep Nav2 cmd_vel flowing.
        roll_lim, pitch_lim = (0.55, 0.55) if walking else (0.4, 0.4)
        upright = (abs(float(rpy[0])) < roll_lim
                   and abs(float(rpy[1])) < pitch_lim)

        s.lifecycle_state = (
            WalkingState.LIFECYCLE_ESTOPPED if self.estop else
            WalkingState.LIFECYCLE_ACTIVE if self.active else
            WalkingState.LIFECYCLE_INACTIVE)
        if self.estop:
            s.locomotion_state = WalkingState.STATE_ESTOPPED
        elif self.fallen:
            s.locomotion_state = WalkingState.STATE_FALLEN
        elif walking:
            s.locomotion_state = WalkingState.STATE_WALKING
        elif self.active:
            s.locomotion_state = WalkingState.STATE_STANDING
        else:
            s.locomotion_state = WalkingState.STATE_IDLE
        s.locomotion_mode = WalkingState.MODE_WALK
        s.support_phase = WalkingState.SUPPORT_DOUBLE
        s.is_balanced = bool(base_h > 0.6 and upright and not self.fallen)
        s.is_fallen = bool(self.fallen)
        s.estop_active = bool(self.estop)
        s.adapter_connected = True
        s.active_adapter = "locomotion_ros2_gait_lab_sil/GaitLabSilAdapter"
        s.active_robot_model = "unitree_g1"
        s.status_text = (
            f"gait_lab[{self.controller_name}] sim: base_h={base_h:.2f}m "
            f"{'walking' if walking else 'standing'}")
        self.state_pub.publish(s)

    # -- optional live view ------------------------------------------------
    def _init_render(self):
        try:
            import mujoco
            self.renderer = mujoco.Renderer(self.model.model, height=480, width=640)
            self._mj = mujoco
            self._cam = mujoco.MjvCamera()
            self._cam.distance = 3.0
            self._cam.elevation = -18.0
            self._cam.azimuth = 120.0
            self._frame_dir = str(self.get_parameter("save_frames_dir").value)
            self._frame_stride = max(1, int(self.get_parameter("frame_stride").value))
            self._frame_i = 0
            self._frames = []          # in-memory ring buffer of recent frames
            self._frame_cap = 240
            if self._frame_dir:
                os.makedirs(self._frame_dir, exist_ok=True)
                import numpy as np      # noqa: E402
                from gait_lab.pngio import save_png  # noqa: E402
                self._np = np
                self._save_png = save_png
            self.get_logger().info(
                "gait_lab SIL render: on (offscreen)"
                + (f", filmstrip -> {self._frame_dir}/filmstrip.png" if self._frame_dir else ""))
        except Exception as exc:                     # noqa: BLE001
            self.get_logger().warn(f"render unavailable: {exc}")
            self.renderer = None

    def _render(self):
        self._cam.lookat[:] = [self.model.data.qpos[0], self.model.data.qpos[1], 0.6]
        self.renderer.update_scene(self.model.data, camera=self._cam)
        pixels = self.renderer.render()
        if not self._frame_dir:
            return
        if self._frame_i % self._frame_stride == 0:
            self._frames.append(pixels.copy())
            if len(self._frames) > self._frame_cap:
                self._frames.pop(0)
            # Refresh a rolling filmstrip of the recent motion every so often, so
            # the latest ROS-driven trajectory is always captured on disk.
            if len(self._frames) >= 10 and len(self._frames) % 12 == 0:
                self._write_filmstrip()
        self._frame_i += 1

    def _write_filmstrip(self, cols: int = 10):
        np = self._np
        idx = np.linspace(0, len(self._frames) - 1, cols).round().astype(int)
        h, w, _ = self._frames[0].shape
        gap = 4
        strip = np.full((h, cols * w + (cols - 1) * gap, 3), 255, np.uint8)
        for k, i in enumerate(idx):
            x = k * (w + gap)
            strip[:, x:x + w] = self._frames[i]
        # Orange ribbon on top (the rl gait colour from the comparison montage).
        ribbon = np.zeros((10, strip.shape[1], 3), np.uint8)
        ribbon[:, :] = (255, 87, 34)
        image = np.vstack([ribbon, strip])
        self._save_png(os.path.join(self._frame_dir, "filmstrip.png"), image)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = GaitLabSilSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
