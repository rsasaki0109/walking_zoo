#!/usr/bin/env python3
"""gait_lab policy node for the ros2_control-split SIL path.

When ``ros2_control_split`` is enabled on ``gait_lab_sil_sim.py``, physics runs in
the sim node and this node owns the gait_lab controller. It subscribes to the
sim's joint states (to keep its model observation in sync), publishes position
actuator targets on ``/gait_lab_sil/ros2_control/joint_commands``, and handles
the same ``/gait_lab_sil/command_velocity`` / ``/gait_lab_sil/control`` bridge
inputs as the monolithic sim.
"""

import os
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

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
    here = Path(__file__).resolve()
    for up in (3, 4, 2):
        if len(here.parents) > up:
            candidates.append(here.parents[up] / "experiments" / "gait_lab")
    candidates.append(Path.cwd() / "experiments" / "gait_lab")
    for candidate in candidates:
        if (candidate / "gait_lab" / "__init__.py").exists():
            return str(candidate)
    raise RuntimeError(
        "Could not find experiments/gait_lab. Set LOCOMOTION_ROS2_GAIT_LAB_PATH."
    )


class GaitLabSilGaitController(Node):
    def __init__(self):
        super().__init__("gait_lab_sil_gait_controller")
        self.declare_parameter("controller", "rl-residual")
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("substeps", 10)
        self.declare_parameter("move_threshold", 0.02)
        self.declare_parameter("joint_commands_topic", "/gait_lab_sil/ros2_control/joint_commands")
        self.declare_parameter("joint_states_topic", "/gait_lab_sil/ros2_control/joint_states")
        self.declare_parameter("use_ros2_control_forward", False)
        self.declare_parameter("use_embedded_rl_policy", False)
        self.declare_parameter(
            "ros2_control_forward_topic", "/gait_lab_sil_gait_forward/commands")
        self.declare_parameter(
            "embedded_rl_observation_topic",
            "/gait_lab_sil_rl_residual/observation")
        self.declare_parameter(
            "embedded_rl_feedforward_topic",
            "/gait_lab_sil_rl_residual/feedforward")
        self.declare_parameter("steer_yaw_ramp_rate", 0.15)

        controller_name = self.get_parameter("controller").value
        self.substeps = int(self.get_parameter("substeps").value)
        self.move_threshold = float(self.get_parameter("move_threshold").value)
        self.use_ros2_control_forward = bool(
            self.get_parameter("use_ros2_control_forward").value)
        self.use_embedded_rl_policy = bool(
            self.get_parameter("use_embedded_rl_policy").value)
        commands_topic = self.get_parameter("joint_commands_topic").value
        forward_topic = self.get_parameter("ros2_control_forward_topic").value
        embedded_obs_topic = self.get_parameter("embedded_rl_observation_topic").value
        embedded_ff_topic = self.get_parameter("embedded_rl_feedforward_topic").value

        sys.path.insert(0, _locate_gait_lab())
        from gait_lab import CONTROLLERS, Command, G1Model  # noqa: E402

        self._Command = Command
        self.model = G1Model()
        controllers = {c.name: c for c in CONTROLLERS()}
        if controller_name not in controllers:
            raise RuntimeError(
                f"unknown controller {controller_name!r}; choices: {sorted(controllers)}")
        self.controller = controllers[controller_name]
        self.controller_name = controller_name
        self.stand = self.model.stand_targets.copy()
        self.steer_shaping = controller_name.startswith("rl-steerable")
        self.steer_yaw_ramp_rate = float(
            self.get_parameter("steer_yaw_ramp_rate").value)
        self._shaped_yaw = 0.0

        self.active = False
        self.estop = False
        self.fallen = False
        self.cmd_speed = 0.0
        self.cmd_lateral = 0.0
        self.cmd_yaw = 0.0
        self.moving = False
        self.gait_t = 0.0
        self.walking_prev = False
        self._pending_push = None
        self._state_synced = False
        self._last_forward_positions = None
        from rclpy.qos import DurabilityPolicy, QoSProfile

        self.create_subscription(TwistStamped, "gait_lab_sil/command_velocity", self._on_cmd, 10)
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, "gait_lab_sil/control", self._on_control, latched)
        self.create_subscription(Vector3, "gait_lab_sil/push", self._on_push, 10)
        snapshot_qos = QoSProfile(depth=1)
        snapshot_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Float64MultiArray, "gait_lab_sil/physics_snapshot",
            self._on_physics_snapshot, snapshot_qos)
        self.joint_cmd_pub = None
        self.forward_cmd_pub = None
        self.embedded_obs_pub = None
        self.embedded_ff_pub = None
        if self.use_embedded_rl_policy:
            if not hasattr(self.controller, "feedforward_and_observation"):
                raise RuntimeError(
                    f"controller {controller_name!r} does not support embedded RL policy")
            self.embedded_obs_pub = self.create_publisher(
                Float64MultiArray, embedded_obs_topic, 10)
            self.embedded_ff_pub = self.create_publisher(
                Float64MultiArray, embedded_ff_topic, 10)
        elif self.use_ros2_control_forward:
            self.forward_cmd_pub = self.create_publisher(
                Float64MultiArray, forward_topic, 10)
        else:
            self.joint_cmd_pub = self.create_publisher(JointState, commands_topic, 10)

        hz = float(self.get_parameter("control_hz").value)
        if self.use_embedded_rl_policy:
            path = "embedded C++ RL residual"
        elif self.use_ros2_control_forward:
            path = "ros2_control forward"
        else:
            path = "direct joint_commands"
        self.get_logger().info(
            f"gait_lab SIL gait controller up: policy={controller_name} "
            f"({path}, physics_snapshot-driven, ~{hz:.0f} Hz)")

    def _on_cmd(self, msg: TwistStamped):
        self.cmd_speed = msg.twist.linear.x
        self.cmd_lateral = msg.twist.linear.y
        self.cmd_yaw = msg.twist.angular.z
        mag = abs(self.cmd_speed) + abs(self.cmd_lateral) + abs(self.cmd_yaw)
        self.moving = mag > self.move_threshold

    def _on_push(self, msg: Vector3):
        self._pending_push = (float(msg.x), float(msg.y))

    def _on_control(self, msg: String):
        signal = msg.data
        if signal == "activate":
            self._reset_robot()
            self._state_synced = False
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
            if self.fallen:
                self._reset_robot()
        elif signal in ("stop_normal", "stop_quick"):
            self.cmd_speed = self.cmd_lateral = self.cmd_yaw = 0.0
            self.moving = False
            self._shaped_yaw = 0.0
        self.get_logger().info(f"gait_lab SIL gait control: {signal}")

    def _reset_robot(self):
        self.model.reset()
        self.controller.reset(self.model)
        self.gait_t = 0.0
        self.walking_prev = False
        self.fallen = False
        self._state_synced = False

    def _on_physics_snapshot(self, msg: Float64MultiArray):
        nq = int(self.model.model.nq)
        nv = int(self.model.model.nv)
        expected = nq + nv
        if len(msg.data) < expected:
            self.get_logger().warning(
                f"physics_snapshot too short: {len(msg.data)} < {expected}")
            return
        d = self.model.data
        d.qpos[:] = msg.data[:nq]
        d.qvel[:] = msg.data[nq:expected]
        self._mujoco_forward()
        self._state_synced = True
        self._tick()

    def _mujoco_forward(self):
        import mujoco
        mujoco.mj_forward(self.model.model, self.model.data)

    def _rehome_posture(self):
        d = self.model.data
        x, y = float(d.qpos[0]), float(d.qpos[1])
        d.qpos[:] = self.model.stand_qpos
        d.qpos[0], d.qpos[1] = x, y
        d.qvel[:] = 0.0
        self._mujoco_forward()

    def _commanded_to_move(self) -> bool:
        return self.moving

    def _effective_command(self, dt: float | None = None):
        raw_vx = float(self.cmd_speed)
        raw_yaw = float(self.cmd_yaw)
        if not self.steer_shaping or not self.moving:
            self._shaped_yaw = 0.0
            return self._Command(forward_speed=raw_vx, yaw_rate=raw_yaw)
        step_dt = self.model.timestep if dt is None else dt
        max_delta = self.steer_yaw_ramp_rate * step_dt
        delta = max(-max_delta, min(max_delta, raw_yaw - self._shaped_yaw))
        self._shaped_yaw += delta
        return self._Command(forward_speed=raw_vx, yaw_rate=self._shaped_yaw)

    def _tick(self):
        if not self._state_synced:
            return

        walking = self.active and not self.estop and not self.fallen and self._commanded_to_move()
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

        if self.use_embedded_rl_policy:
            # Physics runs in gait_lab_sil_sim; advance the policy decimation counter
            # once per MuJoCo substep (as update() does) but never mj_step locally.
            if walking:
                ctrl = self.stand
                policy_obs = None
                refresh = False
                for _ in range(self.substeps):
                    cmd = self._effective_command()
                    obs = self.model.observe(self.gait_t)
                    ctrl, step_obs, refresh = self.controller.feedforward_and_observation(
                        obs, cmd)
                    if step_obs is not None:
                        policy_obs = step_obs
                    self.gait_t += self.model.timestep
                if policy_obs is not None:
                    self._publish_embedded_rl(ctrl, policy_obs)
                else:
                    self._publish_embedded_ff(ctrl)
            else:
                self._publish_embedded_rl_stand()
            return

        for _ in range(self.substeps):
            if walking:
                cmd = self._effective_command()
                obs = self.model.observe(self.gait_t)
                ctrl = self.controller.update(obs, cmd)
                self._publish_joint_command(ctrl)
                self.gait_t += self.model.timestep
            else:
                self._publish_joint_command(self.stand)
            self.model.data.ctrl[:] = ctrl if walking else self.stand
            self.model.step()

        if float(self.model.data.qpos[2]) < 0.5:
            self.fallen = True

    def _publish_embedded_rl(self, ctrl, policy_obs):
        ff = [float(ctrl[self.model.actuator(name)]) for name in LEG_ACTUATORS]
        obs_msg = Float64MultiArray()
        obs_msg.data = [float(v) for v in policy_obs]
        ff_msg = Float64MultiArray()
        ff_msg.data = ff
        self.embedded_obs_pub.publish(obs_msg)
        self.embedded_ff_pub.publish(ff_msg)

    def _publish_embedded_ff(self, ctrl):
        ff_msg = Float64MultiArray()
        ff_msg.data = [float(ctrl[self.model.actuator(name)]) for name in LEG_ACTUATORS]
        self.embedded_ff_pub.publish(ff_msg)

    def _publish_embedded_rl_stand(self):
        ff = [float(self.stand[self.model.actuator(name)]) for name in LEG_ACTUATORS]
        ff_msg = Float64MultiArray()
        ff_msg.data = ff
        self.embedded_ff_pub.publish(ff_msg)

    def _publish_joint_command(self, ctrl):
        positions = [float(ctrl[self.model.actuator(name)]) for name in LEG_ACTUATORS]
        if self.forward_cmd_pub is not None:
            self._last_forward_positions = positions
            msg = Float64MultiArray()
            msg.data = positions
            self.forward_cmd_pub.publish(msg)
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(LEG_ACTUATORS)
        msg.position = positions
        self.joint_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaitLabSilGaitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
