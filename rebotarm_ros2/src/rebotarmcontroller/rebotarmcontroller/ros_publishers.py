from __future__ import annotations

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from sensor_msgs.msg import JointState

_GRIPPER_MAX_WIDTH = 0.09


def _gripper_motor_to_joint_position(
    position: float,
    open_position: float,
    close_position: float,
) -> float:
    span = open_position - close_position
    ratio = 0.0 if span == 0.0 else (position - close_position) / span
    return max(0.0, min(_GRIPPER_MAX_WIDTH * 0.5, ratio * _GRIPPER_MAX_WIDTH * 0.5))


class JointStatePublisher:
    def __init__(self, node, hardware, namespace: str, rate_hz: float) -> None:
        self._node = node
        self._hardware = hardware
        self._publisher = node.create_publisher(
            JointState,
            f"/{namespace}/joint_states",
            node.sensor_qos,
            callback_group=node.reentrant_group,
        )
        self._joint_state_publishers = {
            name: node.create_publisher(
                JointMotorState,
                f"/{namespace}/joints/{name}/state",
                node.sensor_qos,
                callback_group=node.reentrant_group,
            )
            for name in hardware.joint_names
        }
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_publisher = node.create_publisher(
            ArmStatus,
            f"/{namespace}/arm_status",
            latched_qos,
            callback_group=node.reentrant_group,
        )
        self._gripper_state_publisher = None
        if hardware.has_gripper:
            self._gripper_state_publisher = node.create_publisher(
                JointMotorState,
                f"/{namespace}/gripper/state",
                node.sensor_qos,
                callback_group=node.reentrant_group,
            )
        period = 1.0 / max(float(rate_hz), 1.0)
        # Mutually-exclusive group: on the reentrant group an overrunning
        # publish is dispatched again concurrently and two unlocked CAN reads
        # race each other and the control loop.
        self._timer = node.create_timer(
            period,
            self.publish,
            callback_group=node.joint_state_group,
        )
        self._lock_skips = 0
        self.publish_status()

    # How long publish() may wait for the command lock. A skipped 100 Hz
    # sample is harmless; stealing the control loop's CAN reply frame is not.
    _LOCK_WAIT = 0.002

    def publish(self) -> None:
        # Every CAN-touching read happens inside one command-lock window so it
        # can never interleave with the control loop's own bus traffic.
        gripper_state = None
        with self._hardware.cmd_lock_window(self._LOCK_WAIT) as acquired:
            if not acquired:
                self._lock_skips += 1
                if self._lock_skips % 200 == 0:
                    self._node.get_logger().warn(
                        f"joint state publisher skipped {self._lock_skips} "
                        "cycles waiting for the command lock"
                    )
                return
            try:
                pos, vel, effort = self._hardware.get_joint_state()
                if self._gripper_state_publisher is not None:
                    gripper_state = self._hardware.get_gripper_state()
            except Exception as exc:
                self._node.get_logger().warn(f"joint state read failed: {exc}")
                return

        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        status_codes = self._hardware.get_joint_status_codes()
        # Piggyback on feedback the control loop already requested (request=False)
        # so publishing temperature adds no extra CAN traffic.
        try:
            t_mos, t_rotor = self._hardware.get_joint_temperatures(request=False)
        except Exception:
            t_mos = t_rotor = [float("nan")] * len(self._hardware.joint_names)
        for i, name in enumerate(self._hardware.joint_names):
            motor_msg = JointMotorState()
            motor_msg.header = msg.header
            motor_msg.joint_name = name
            motor_msg.position = float(pos[i])
            motor_msg.velocity = float(vel[i])
            motor_msg.torque = float(effort[i])
            motor_msg.status_code = int(status_codes[i])
            motor_msg.temperature_mos = (
                float(t_mos[i]) if i < len(t_mos) else float("nan")
            )
            motor_msg.temperature_rotor = (
                float(t_rotor[i]) if i < len(t_rotor) else float("nan")
            )
            self._joint_state_publishers[name].publish(motor_msg)

        msg.name = self._hardware.joint_names
        msg.position = [float(v) for v in pos]
        msg.velocity = [float(v) for v in vel]
        msg.effort = [float(v) for v in effort]

        if self._gripper_state_publisher is not None and gripper_state is not None:
            g_pos, g_vel, g_torque, g_status = gripper_state
            joint_pos = _gripper_motor_to_joint_position(
                float(g_pos),
                self._hardware.gripper_open_position,
                self._hardware.gripper_close_position,
            )
            joint_vel = (
                _gripper_motor_to_joint_position(
                    float(g_pos + g_vel),
                    self._hardware.gripper_open_position,
                    self._hardware.gripper_close_position,
                )
                - joint_pos
            )
            msg.name.extend(["gripper_joint1", "gripper_joint2"])
            msg.position.extend([joint_pos, joint_pos])
            msg.velocity.extend([joint_vel, joint_vel])
            msg.effort.extend([float(g_torque), float(g_torque)])

            gripper_msg = JointMotorState()
            gripper_msg.header = msg.header
            gripper_msg.joint_name = "gripper"
            gripper_msg.position = float(g_pos)
            gripper_msg.velocity = float(g_vel)
            gripper_msg.torque = float(g_torque)
            gripper_msg.status_code = int(g_status)
            g_t_mos, g_t_rotor = self._hardware.get_gripper_temperatures(request=False)
            gripper_msg.temperature_mos = g_t_mos
            gripper_msg.temperature_rotor = g_t_rotor
            self._gripper_state_publisher.publish(gripper_msg)

        self._publisher.publish(msg)

    def publish_status(self, *, read_hardware: bool = True) -> None:
        msg = ArmStatus()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.mode = self._hardware.mode
        msg.enabled = self._hardware.enabled
        msg.control_loop_active = self._hardware.control_loop_active
        msg.state_machine = self._hardware.state_machine
        msg.joint_names = self._hardware.joint_names
        if read_hardware:
            msg.per_joint_status_code = self._hardware.get_joint_status_codes()
        else:
            msg.per_joint_status_code = [0 for _ in self._hardware.joint_names]
        msg.error_codes = self._hardware.error_codes
        self._status_publisher.publish(msg)
