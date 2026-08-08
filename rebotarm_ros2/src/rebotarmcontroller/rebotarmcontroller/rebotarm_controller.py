from __future__ import annotations

import signal

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .hardware_manager import HardwareManager
from .motor_passthrough import MotorPassthrough
from .ros_actions import ArmActions
from .chunk_executor import ChunkExecutor
from .ros_publishers import JointStatePublisher
from .ros_services import ArmServices
from .thermal_guard import ThermalGuard


class reBotArmController(Node):
    def __init__(self) -> None:
        super().__init__("reBotArmController")

        self.reentrant_group = ReentrantCallbackGroup()
        self.slow_group = MutuallyExclusiveCallbackGroup()
        # One mutually-exclusive group PER high-rate timer. On the reentrant
        # group, a tick that overruns its period is dispatched AGAIN on another
        # executor thread: overlapping joint-state reads race on the CAN
        # handle, and overlapping policy ticks race on the executor's
        # _last_target (an older sample can win the write and command the arm
        # backwards). Separate groups keep the two timers concurrent with each
        # other while serializing each with itself.
        self.joint_state_group = MutuallyExclusiveCallbackGroup()
        self.policy_tick_group = MutuallyExclusiveCallbackGroup()
        self.sensor_qos = qos_profile_sensor_data

        self.declare_parameter("hardware_config", "")
        self.declare_parameter("model", "")
        self.declare_parameter("channel", "")
        self.declare_parameter("joint_state_rate", 100.0)
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("cmd_arbitration", "reject")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("ee_frame_id", "end_link")
        self.declare_parameter("disable_after_safe_home", True)
        # --- gripper compliance --------------------------------------------
        # The gripper close is a pure MIT spring toward the mechanical limit,
        # so before these knobs existed any grasped object saw
        # kp * (remaining stroke) of torque demand -- saturated at the rs-00
        # MIT frame limit (~14 Nm) -- indefinitely. close_gripper_grasp()
        # ramps the target at gripper_close_speed, declares a grasp when the
        # fingers stop (moved < gripper_stall_dpos over gripper_stall_window
        # while the ramp ran >0.1 rad ahead), then re-anchors the target so
        # the steady squeeze is ~gripper_hold_torque. gripper_kp/gripper_kd
        # override the SDK yaml's 50/4. All six are live-tunable with
        # `ros2 param set` (kp/kd/hold_torque affect even an ongoing hold's
        # stiffness; the re-anchor distance updates on the next close).
        self.declare_parameter("gripper_kp", 20.0)
        self.declare_parameter("gripper_kd", 2.0)
        self.declare_parameter("gripper_hold_torque", 2.0)
        self.declare_parameter("gripper_close_speed", 3.0)
        self.declare_parameter("gripper_stall_window", 0.15)
        self.declare_parameter("gripper_stall_dpos", 0.01)
        # Thermal protection. NOTE: these defaults are CONSERVATIVE PLACEHOLDERS
        # and have not been checked against a RobStride datasheet -- verify the
        # real limits for rs-00/rs-06 before relying on them.
        self.declare_parameter("thermal_enabled", True)
        self.declare_parameter("thermal_warn_c", 60.0)
        self.declare_parameter("thermal_critical_c", 75.0)
        self.declare_parameter("thermal_poll_hz", 1.0)
        self.declare_parameter("thermal_consecutive_to_trip", 3)
        # Optional Fahrenheit overrides. Thresholds are Celsius internally
        # (SI/REP-103); set these > 0 to specify the limits in F instead and
        # they take precedence over thermal_warn_c / thermal_critical_c.
        self.declare_parameter("thermal_warn_f", 0.0)
        self.declare_parameter("thermal_critical_f", 0.0)

        # --- learned-policy action-chunk execution -------------------------
        # Subscribes to chunks and upsamples them to the control rate with
        # ACT-style temporal ensembling. Creating it does NOT start motion --
        # it stays inactive until /rebotarm/policy/start is called.
        self.declare_parameter("policy_enabled", True)
        self.declare_parameter("policy_chunk_topic", "")
        # 100 Hz, not 200. Each tick takes _cmd_lock to write the joint target,
        # and the 500 Hz MIT loop skips any cycle where it cannot get that lock
        # promptly (see HardwareManager._endpos_loop_cb). Chunks are interpolated
        # with a cubic Hermite, so sampling that curve at 100 Hz instead of 200 Hz
        # is imperceptible -- while a dropped torque command is not. Matches the
        # trajectory executor's rate. Raise it on a host with CPU headroom.
        self.declare_parameter("policy_control_hz", 100.0)
        self.declare_parameter("policy_dt", 0.1)
        # ACT's published scheme is m=0.01 favor=oldest -- near-uniform weights
        # across DOZENS of overlapping policy chunks. The teleop publishes
        # short 3-point chunks where at most TWO overlap, and the older one is
        # in its zero-velocity hold tail: near-uniform weighting then blends a
        # live segment 50/50 against a stopped one and SNAPS to full amplitude
        # when the old chunk expires -- a position + velocity-FF staircase at
        # exactly the publish rate. m=5.0 favor=newest gives the newest
        # covering chunk ~99% of the weight ("use the live command") while
        # keeping the mechanism for genuine multi-chunk policies, which should
        # set m back toward 0.01.
        self.declare_parameter("policy_ensemble_m", 5.0)
        self.declare_parameter("policy_ensemble_favor", "newest")
        # Add your measured inference latency here if chunks are stamped with
        # the OBSERVATION time rather than the first-action time.
        self.declare_parameter("policy_chunk_time_offset", 0.0)
        self.declare_parameter("policy_stale_timeout", 0.5)
        # Safety clamps. max_joint_velocity was 1.0, which throttled teleop
        # asymmetrically: near the ready pose, sideways/forward Cartesian
        # motion costs ~0.05 rad of joint1/joint2 per cm (vertical much less),
        # so horizontal moves pinned the clamp while vertical ones did not.
        # 2.0 rad/s is still 5x under the lowest URDF joint velocity limit
        # (10 rad/s), and MIT mode has no firmware speed limit either way --
        # the executor step clamp and the teleop's own budget stay the
        # effective limiters.
        self.declare_parameter("policy_max_joint_velocity", 2.0)
        self.declare_parameter("policy_max_position_jump", 0.35)
        self.declare_parameter("policy_joint_limits_lower", [])
        self.declare_parameter("policy_joint_limits_upper", [])

        hardware_config = self.get_parameter("hardware_config").value or None
        model = str(self.get_parameter("model").value or "")
        channel = str(self.get_parameter("channel").value or "")
        self.arm_namespace = str(self.get_parameter("arm_namespace").value or "rebotarm").strip("/")
        joint_state_rate = float(self.get_parameter("joint_state_rate").value)
        cmd_arbitration = str(self.get_parameter("cmd_arbitration").value or "reject")
        self.disable_after_safe_home = bool(
            self.get_parameter("disable_after_safe_home").value
        )
        if cmd_arbitration not in ("reject", "preempt"):
            self.get_logger().warn(
                f"unsupported cmd_arbitration={cmd_arbitration!r}; using 'reject'"
            )
            cmd_arbitration = "reject"

        self.hardware = HardwareManager(
            hardware_config=hardware_config,
            model=model,
            channel=channel,
        )
        self._apply_gripper_params()
        self.hardware.connect()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.joint_state_publisher = JointStatePublisher(
            self,
            self.hardware,
            self.arm_namespace,
            joint_state_rate,
        )
        self.arm_services = ArmServices(self, self.hardware, self.arm_namespace)
        self.arm_actions = ArmActions(self, self.hardware, self.arm_namespace)
        self.motor_passthrough = MotorPassthrough(
            self,
            self.hardware,
            self.arm_namespace,
            cmd_arbitration,
        )

        self.thermal_guard = None
        if bool(self.get_parameter("thermal_enabled").value):
            warn_c = float(self.get_parameter("thermal_warn_c").value)
            critical_c = float(self.get_parameter("thermal_critical_c").value)
            warn_f = float(self.get_parameter("thermal_warn_f").value)
            critical_f = float(self.get_parameter("thermal_critical_f").value)
            if warn_f > 0.0:
                warn_c = (warn_f - 32.0) * 5.0 / 9.0
            if critical_f > 0.0:
                critical_c = (critical_f - 32.0) * 5.0 / 9.0
            if critical_c <= warn_c:
                self.get_logger().warn(
                    f"thermal_critical_c ({critical_c}) must exceed thermal_warn_c "
                    f"({warn_c}); raising critical to warn + 15"
                )
                critical_c = warn_c + 15.0
            self.thermal_guard = ThermalGuard(
                self,
                self.hardware,
                self.arm_namespace,
                warn_c=warn_c,
                critical_c=critical_c,
                poll_hz=float(self.get_parameter("thermal_poll_hz").value),
                consecutive_to_trip=int(
                    self.get_parameter("thermal_consecutive_to_trip").value
                ),
            )

        self.chunk_executor = None
        if bool(self.get_parameter("policy_enabled").value):
            chunk_topic = str(self.get_parameter("policy_chunk_topic").value or "")
            if not chunk_topic:
                chunk_topic = f"/{self.arm_namespace}/policy/action_chunk"
            self.chunk_executor = ChunkExecutor(
                self,
                self.hardware,
                self.arm_namespace,
                chunk_topic=chunk_topic,
                control_hz=float(self.get_parameter("policy_control_hz").value),
                policy_dt=float(self.get_parameter("policy_dt").value),
                ensemble_m=float(self.get_parameter("policy_ensemble_m").value),
                ensemble_favor=str(
                    self.get_parameter("policy_ensemble_favor").value
                ),
                chunk_time_offset=float(
                    self.get_parameter("policy_chunk_time_offset").value
                ),
                stale_timeout=float(self.get_parameter("policy_stale_timeout").value),
                max_joint_velocity=float(
                    self.get_parameter("policy_max_joint_velocity").value
                ),
                max_position_jump=float(
                    self.get_parameter("policy_max_position_jump").value
                ),
                joint_limits_lower=list(
                    self.get_parameter("policy_joint_limits_lower").value or []
                ),
                joint_limits_upper=list(
                    self.get_parameter("policy_joint_limits_upper").value or []
                ),
            )

        # The SDK's kinematics model is chosen by its own config, not by model:=,
        # and it is what the 500 Hz loop's gravity feedforward runs on. A mismatch
        # is silent and presents as droop/jitter/heat, so surface it.
        warning = getattr(self.hardware, "kinematics_warning", None)
        if warning:
            self.get_logger().error(warning)
        else:
            self.get_logger().info(
                "SDK kinematics model (gravity feedforward): "
                f"{getattr(self.hardware, 'kinematics_model', 'unknown')}")

        self.get_logger().info(
            f"reBotArmController started: namespace=/{self.arm_namespace}, "
            f"joints={self.hardware.joint_names}"
        )

    _GRIPPER_PARAM_MAP = {
        "gripper_kp": "kp",
        "gripper_kd": "kd",
        "gripper_hold_torque": "hold_torque",
        "gripper_close_speed": "close_speed",
        "gripper_stall_window": "stall_window",
        "gripper_stall_dpos": "stall_dpos",
    }

    def _apply_gripper_params(self) -> None:
        self.hardware.configure_gripper(
            **{
                kwarg: float(self.get_parameter(param).value)
                for param, kwarg in self._GRIPPER_PARAM_MAP.items()
            }
        )

    def _on_set_parameters(self, params) -> SetParametersResult:
        """Live tuning: forward gripper_* parameter changes to the hardware."""
        updates = {}
        for param in params:
            kwarg = self._GRIPPER_PARAM_MAP.get(param.name)
            if kwarg is None:
                continue
            value = float(param.value)
            if value <= 0.0:
                return SetParametersResult(
                    successful=False,
                    reason=f"{param.name} must be > 0",
                )
            updates[kwarg] = value
        if updates:
            self.hardware.configure_gripper(**updates)
            self.get_logger().info(f"gripper tuning updated: {updates}")
        return SetParametersResult(successful=True)

    def publish_arm_status(self, *, read_hardware: bool = True) -> None:
        self.joint_state_publisher.publish_status(read_hardware=read_hardware)

    def shutdown(self) -> None:
        self.hardware.shutdown(
            disable_after_safe_home=self.disable_after_safe_home,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = reBotArmController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # "Never just stop" -- SIGTERM (plain `kill`, systemd, launch teardown) would
    # otherwise terminate the process outright, leaving the motors latched on the
    # last setpoint in an arbitrary pose. Turn it into a normal shutdown so the
    # finally-block below runs safe_home() first.
    # NOTE: SIGKILL (kill -9) CANNOT be intercepted by any process. That path
    # still leaves the motors holding their last commanded position.
    def _graceful(signum, _frame):
        node.get_logger().warn(
            f"signal {signum} received -- returning to zero before shutdown"
        )
        raise KeyboardInterrupt

    try:
        signal.signal(signal.SIGTERM, _graceful)
    except (ValueError, OSError):
        pass

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # hardware.shutdown() runs safe_home() (drive to zero) and only then
        # disables. Every non-SIGKILL exit funnels through here.
        # Shield it: a second SIGINT/SIGTERM (ros2 launch forwards signals,
        # impatient double Ctrl-C) used to raise KeyboardInterrupt INSIDE
        # safe_home, killing the process with the motors still enabled and
        # latched at torque. Once shutdown starts, further signals are
        # ignored -- SIGKILL remains the only (unsafe) way to cut it short.
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, signal.SIG_IGN)
            except (ValueError, OSError):
                pass
        try:
            node.shutdown()
        except Exception as exc:
            node.get_logger().error(f"shutdown failed: {exc}")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
