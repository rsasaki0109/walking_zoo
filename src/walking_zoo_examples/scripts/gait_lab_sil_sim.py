#!/usr/bin/env python3
"""MuJoCo Unitree G1 software-in-the-loop sim driven by a gait_lab controller.

This is the companion sim behind ``walking_zoo_gait_lab_sil/GaitLabSilAdapter``.
The C++ adapter is a thin ROS bridge inside the runtime; the actual physics and
the learned gait live *here*, so MuJoCo stays an optional dependency of this one
node (never of walking_zoo itself).

It closes the experiment → product loop: the reinforcement-learned ``rl-residual``
gait validated in ``experiments/gait_lab`` now runs as a live simulated robot
behind the real runtime/safety pipeline. The adapter forwards the runtime's
safety-filtered velocity (and lifecycle control) here; this node steps the G1
through the gait_lab controller and publishes the resulting WalkingState back, so
the runtime reports — and the visualizer shows — a robot that genuinely walks.

    # bridge topics (published/subscribed by the C++ adapter):
    #   <- /gait_lab_sil/command_velocity  (geometry_msgs/TwistStamped)
    #   <- /gait_lab_sil/control           (std_msgs/String lifecycle signal)
    #   -> /gait_lab_sil/robot_state       (walking_zoo_msgs/WalkingState)

    ros2 run walking_zoo_examples gait_lab_sil_sim.py --ros-args -p controller:=rl-residual

gait_lab lives outside the ROS packages; point this node at it with
WALKING_ZOO_GAIT_LAB_PATH (it also tries the in-repo location automatically).
"""

import os
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from walking_zoo_msgs.msg import WalkingState


def _locate_gait_lab() -> str:
    candidates = []
    env = os.environ.get("WALKING_ZOO_GAIT_LAB_PATH")
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
        "Could not find experiments/gait_lab. Set WALKING_ZOO_GAIT_LAB_PATH to the "
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
        if controller_name not in controllers:
            raise RuntimeError(
                f"unknown controller {controller_name!r}; "
                f"choices: {sorted(controllers)}")
        self.controller = controllers[controller_name]
        self.controller_name = controller_name

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
        self.state_pub = self.create_publisher(WalkingState, "gait_lab_sil/robot_state", 10)

        hz = float(self.get_parameter("control_hz").value)
        self.timer = self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f"gait_lab SIL sim up: controller={controller_name}, "
            f"{self.substeps} mj_steps/tick @ {hz:.0f} Hz")

    # -- bridge inputs -----------------------------------------------------
    def _on_cmd(self, msg: TwistStamped):
        self.cmd_speed = msg.twist.linear.x
        self.cmd_lateral = msg.twist.linear.y
        self.cmd_yaw = msg.twist.angular.z
        mag = abs(self.cmd_speed) + abs(self.cmd_lateral) + abs(self.cmd_yaw)
        self.moving = mag > self.move_threshold

    def _on_control(self, msg: String):
        signal = msg.data
        if signal == "activate":
            self._reset_robot()
            self.active = True
            self.estop = False
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
        self.get_logger().info(f"gait_lab SIL control: {signal}")

    def _reset_robot(self):
        self.model.reset()
        self.controller.reset(self.model)
        self.gait_t = 0.0
        self.walking_prev = False
        self.fallen = False

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

    def _tick(self):
        walking = self.active and not self.estop and not self.fallen \
            and self._commanded_to_move()
        if walking and not self.walking_prev:
            # Rising edge: re-home the posture + zero velocity (keeping world x/y)
            # and start the gait fresh (phase 0). This makes the gait begin from
            # exactly the nominal stand condition the policy was verified to walk
            # the full horizon from — without it, the slightly-varying held-stand
            # pose occasionally tips this learned (not bulletproof) gait over.
            self._rehome_posture()
            self.controller.reset(self.model)
            self.gait_t = 0.0
        self.walking_prev = walking

        cmd = self._Command(forward_speed=float(self.cmd_speed))
        for _ in range(self.substeps):
            if walking:
                obs = self.model.observe(self.gait_t)
                ctrl = self.controller.update(obs, cmd)
                self.gait_t += self.model.timestep
            else:
                ctrl = self.stand            # hold the standing pose
            self.model.data.ctrl[:] = ctrl
            self.model.step()

        if float(self.model.data.qpos[2]) < self.fall_height:
            self.fallen = True
        if self.renderer is not None:
            self._render()
        self._publish_state(walking)

    # -- bridge output -----------------------------------------------------
    def _publish_state(self, walking: bool):
        s = WalkingState()
        s.header.stamp = self.get_clock().now().to_msg()
        base_h = float(self.model.data.qpos[2])
        rpy = self.model.observe(self.gait_t).torso_rpy
        # Cast through float(): rpy is a numpy array, so naive comparisons yield
        # numpy.bool_, which the ROS message C extension rejects (it asserts a
        # native Python bool on the wire).
        upright = abs(float(rpy[0])) < 0.4 and abs(float(rpy[1])) < 0.4

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
        s.active_adapter = "walking_zoo_gait_lab_sil/GaitLabSilAdapter"
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
            self.get_logger().info("gait_lab SIL render: on (offscreen)")
        except Exception as exc:                     # noqa: BLE001
            self.get_logger().warn(f"render unavailable: {exc}")
            self.renderer = None

    def _render(self):
        self._cam.lookat[:] = [self.model.data.qpos[0], self.model.data.qpos[1], 0.6]
        self.renderer.update_scene(self.model.data, camera=self._cam)
        self.renderer.render()

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
