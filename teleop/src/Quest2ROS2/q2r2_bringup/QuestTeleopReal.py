"""
Quest Teleoperation - REAL HARDWARE (reBot B601-RS, CAN/PCAN)
------------------------------------------------------------
Teleoperates the reBot B601 **RS** variant from a Quest controller by streaming
Cartesian targets to the `reBotArmController` driver. Real-robot counterpart to
QuestIKBridge (which targets the mock/RViz stack).

READ THIS BEFORE RUNNING.

WRITE PATH (and why it is this one)
The driver's per-joint low-level topics (`/joints/<j>/cmd/pos_vel`) are NOT
usable for coordinated motion: `send_joint_pos_vel_cmd` re-reads the measured
joint vector and overrides only the single joint named in that message, so
publishing six messages per cycle makes each one cancel the previous joint's
target. Instead we stream the driver's `move_to_pose_ik` service, which sets the
whole joint target atomically via the driver's own IK and lets its 500 Hz
endpose loop do the actuation. `_require_idle` permits IDLE and
LOWLEVEL_STREAMING, so it is safe to call repeatedly.

SAFETY MODEL
1. Deadman trigger. Moves ONLY while the trigger is held past
   `deadman_threshold`. Release => streaming stops, arm holds. No toggle mode.
2. Watchdog. Quest data stale for `watchdog_timeout` (headset asleep, WiFi
   drop, app closed) => stop and hold. Motion never continues on stale data.
3. Cartesian rate clamp. Each target is clamped to `max_cart_step` metres from
   the arm's CURRENT MEASURED end-effector pose (not from the previous target,
   which would let error accumulate). Speed is therefore bounded near
   max_cart_step * rate  (default 0.015 m * 20 Hz = 0.30 m/s) and the driver's
   internal joint target can never be far from where the arm actually is.
4. Singularity / reachability guard. Before commanding, the same target is run
   through MoveIt `/compute_ik`. If it fails, or implies a joint jump over
   `max_ik_jump`, the target is REJECTED and nothing is sent. Near a
   singularity a small Cartesian step can imply an enormous joint motion.
5. Post-command jump check. `move_to_pose_ik` returns the joint solution it
   accepted. If that turns out to be further than `max_ik_jump` from measured,
   we immediately re-command the arm's current measured pose to pull the target
   back, and disengage. This matters on the RS profile: it runs
   `arm_control_mode: mit`, where torque is proportional to target error and
   there is NO firmware velocity ceiling.
6. Hard joint-limit guard. The MoveIt guard solution is checked against the
   URDF limits with a margin before anything is sent.
7. Park on exit. On ANY exit (Ctrl-C, SIGTERM, exception, normal shutdown) the
   arm returns to ALL JOINTS ZERO via the driver's `safe_home` (velocity-ramped
   at 0.5 rad/s, closes the gripper first), waits for it, then disables.
   Fallback if safe_home fails: a slow FollowJointTrajectory to zeros.
   NOTE: `kill -9` cannot be intercepted. Always exit with Ctrl-C.

WHY IT DOES NOT START AT ZERO
On RS, all-zeros puts joint2/joint3 exactly on their LOWER limit (0.0), where IK
has no valid direction and returns NO_IK_SOLUTION for every target. So we move
to `ready_positions` (joint2/joint3 positive) first, and park back at zero on
exit - which is what you asked for.

PREREQUISITES  (RS variant => model:=rs everywhere, and CAN must be up)
    # 0. bring up the CAN interface. RS uses SocketCAN, not /dev/ttyACM*.
    #    can1 = the PCAN USB board;  can0 = the Jetson's built-in mttcan.
    #    Use whichever the arm is actually wired to - check with `candump`.
    $ sudo ip link set can1 up type can bitrate 1000000
    $ ip -br link show can1        # expect UP
    # 1. driver + robot_state_publisher, RS model, on that channel
    $ ros2 launch rebotarm_bringup bringup.launch.py model:=rs channel:=can1
    # 2. MoveIt for the IK guard + RViz, RS model
    $ ros2 launch rebotarm_moveit_config hardware.launch.py model:=rs
    # 3. the Quest link
    $ ros2 launch ros_tcp_endpoint endpoint.py

RUN
    # ALWAYS dry-run first: sends nothing, exercises every check.
    $ ros2 run q2r2_bringup QuestTeleopReal --ros-args -p dry_run:=true

    # First real run: translation only, small steps, arm unloaded.
    $ ros2 run q2r2_bringup QuestTeleopReal --ros-args \
        -p track_orientation:=false -p max_cart_step:=0.008 -p position_scale:=0.2

    # Normal:
    $ ros2 run q2r2_bringup QuestTeleopReal
"""
import json
import math
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import tf_transformations

from quest2ros.msg import OVR2ROSInputs

try:
    from rebotarm_msgs.srv import GripperCommand as RebotGripperCommand
    from rebotarm_msgs.srv import MoveToPoseIK
except ImportError as exc:  # pragma: no cover
    print(f"[FATAL] rebotarm_msgs not found: {exc}\n"
          "        source ~/rebotarm_ros2/install/setup.bash first.", file=sys.stderr)
    raise

# Revolute joint limits for the RS variant, from
# rebotarm_bringup/description/urdf/00-arm-rs_asm-v3.urdf.
# joint2/joint3 are one-sided and POSITIVE-going on RS: [0, 3.14].
# The DM variant is sign-flipped ([-3.14, 0]) with joint4 = [-1.87, 1.57], so if
# you ever run a DM arm, override via joint_limits_lower/joint_limits_upper.
# Consequence: all-zeros sits on the joint2/joint3 LOWER limit, which is why the
# ready pose must drive them positive before IK can solve anything.
JOINT_LIMITS_LOWER = [-2.80, 0.00, 0.00, -1.57, -1.57, -3.14]
JOINT_LIMITS_UPPER = [2.80, 3.14, 3.14, 1.57, 1.57, 3.14]
READY_POSITIONS = [0.0, 1.0, 1.0, 0.0, 0.5, 0.0]


class QuestTeleopReal(Node):
    def __init__(self):
        super().__init__("quest_teleop_real")

        p = self.declare_parameter
        p("side", "right")
        p("arm_namespace", "rebotarm")
        p("group_name", "arm")
        p("base_frame_id", "base_link")
        p("ee_link", "gripper_tcp")
        p("arm_joints", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        p("joint_limits_lower", JOINT_LIMITS_LOWER)
        p("joint_limits_upper", JOINT_LIMITS_UPPER)

        p("dry_run", False)
        p("require_confirmation", True)
        # Dry run only: publish the simulated joint state so RViz renders the
        # ghost arm following your controller with no hardware in the loop.
        p("publish_sim_joint_states", True)

        # "chunk"   - publish action chunks to the driver's ChunkExecutor, which
        #             upsamples them at policy_control_hz (100 Hz) with
        #             cubic-Hermite interpolation AND velocity feedforward.
        #             Smooth. This is the right path.
        # "service"  - stream the move_to_pose_ik service. Kept for comparison and
        #             for a driver without the chunk executor. Jerky by construction:
        #             a 20 Hz staircase of position-only setpoints into the MIT
        #             loop, with qd_target left STALE (whatever was last latched),
        #             so kd acts against a wrong velocity reference. It also runs
        #             its whole Pinocchio IK solve while holding the driver's
        #             command lock, starving the MIT loop for the duration.
        p("command_mode", "chunk")
        p("chunk_tail", 0.02)         # s of HOLD past the target (dropout cushion)
        p("max_cmd_lag", 0.25)        # rad; commanded-vs-measured divergence limit

        # Hand-to-robot translation gain. 0.3 felt like wading through mud
        # once the speed limits were fixed; 0.6 means ~15 cm of hand travel
        # for ~9 cm of tool travel. Rotation gain (orientation_scale) is
        # separate and unchanged. Note the interplay with max_cart_step:
        # commanded speed saturates at max_cart_step*rate (0.3 m/s default)
        # no matter the scale, so raising the scale mostly shortens the hand
        # travel needed, not the top speed.
        p("position_scale", 1.0)
        # Signed axis remap for TRANSLATION, applied to the hand-position
        # delta before scaling: pos_axes[i] names which incoming axis (and
        # sign) feeds ROS axis i (x=forward, y=left, z=up). Example:
        # ["-z", "x", "-y"]. Default is identity (passthrough).
        #
        # IMPORTANT: the quest2ros app publishes poses in a RELATIVE frame
        # that, until aligned, FOLLOWS THE HEADSET - measured on 2026-08-07:
        # controller stationary on a table, head-only motion swung the
        # published pose 35 cm / 92 deg (and the twist topic is the
        # derivative of the same head-relative pose, so it is equally
        # coupled). No static remap can fix that. FIRST align the app's
        # frame: hold the controller in your normal driving grip and press
        # A+B (right controller; X+Y for left). Only then calibrate this
        # remap, one axis at a time, against the RViz ghost in a dry run.
        p("pos_axes", ["x", "y", "z"])
        # Frame-jump guard: a human hand under fine control does not exceed
        # ~1 m/s; the head-coupled frame swings the APPARENT hand position at
        # 2-4 m/s when the operator looks around. Two consecutive raw
        # samples above this speed while engaged -> disengage and hold.
        p("max_hand_speed", 2.0)      # m/s; <= 0 disables the guard
        p("max_cart_step", 0.015)     # m per command, from the last COMMANDED pose
        # Joint-space speed budget. This MUST stay below the driver's
        # policy_max_joint_velocity (default 2.0 rad/s as of Aug 7), because
        # the chunk executor clamps its own output to that:
        #     max_step = max_joint_velocity * elapsed
        # Sizing: max_cart_step 0.015 m at 20 Hz (0.3 m/s Cartesian cap) costs
        # ~1.26 rad/s near the ready pose vertically (joint2, ~0.042 rad/cm)
        # and ~1.44 rad/s sideways (joint1, ~0.048 rad/cm). The old 0.8 budget
        # sat BELOW both, so every horizontal move was scaled down (~16% of
        # commands hit vel_clamp in practice) and left/right/forward felt dead
        # while up/down felt fine. 1.5 clears normal motion in every direction
        # - the Cartesian cap becomes the binding limit, so speed is isotropic
        # - while still catching singularity blowups, where a millimetre of
        # Cartesian motion costs an unbounded amount of joint angle.
        p("max_joint_velocity", 1.5)
        p("max_ik_jump", 0.30)        # rad; larger => reject (singularity)
        # Collision checking in the IK guard. OFF by default: the joint-limit and
        # jump checks below are the ones that protect the hardware, whereas
        # avoid_collisions returns -31 for the WHOLE pose whenever move_group's
        # monitored state looks self-colliding - which on the real robot it can,
        # from a gripper value the simulation never has. Turn it on if you add
        # real obstacles to the planning scene.
        p("avoid_collisions", False)
        p("diagnose_ik", True)        # on rejection, retry to identify the cause
        p("rate", 20.0)
        p("filter_window_size", 8)
        p("deadman_threshold", 0.5)
        p("watchdog_timeout", 0.3)
        p("track_orientation", True)
        p("max_ang_step", 0.05)       # rad of orientation change per command
        # How the hand's rotation delta is applied to the tool.
        #
        # "tool" (default) - the delta is taken in the HAND's own frame and
        #     applied in the TOOL's own frame. Twist your wrist about your hand's
        #     long axis and the tool twists about its long axis. This needs no
        #     knowledge of how the Quest world frame relates to base_link,
        #     because the anchor absorbs it: at the moment you squeeze,
        #     hand-frame == tool-frame by definition.
        # "base" - the delta is taken in the Quest WORLD frame and applied in
        #     base_link. Intuitive only if the two frames happen to be aligned,
        #     which nothing here guarantees (Unity is left-handed Y-up, ROS is
        #     right-handed Z-up, and the headset's origin depends on where you
        #     were standing). Kept because it is the right model once you have
        #     measured the alignment and put it in hand_axes_rpy.
        p("orientation_frame", "tool")
        # Fixed axis remap applied to the delta before it is used, as RPY radians.
        # Use it when rotating your hand about one axis turns the tool about a
        # different axis: watch the RViz ghost in a dry run, one axis at a time,
        # and rotate this until they agree.
        p("hand_axes_rpy", [0.0, 0.0, 0.0])
        # Fraction of your hand's rotation the tool follows. 0.5 halves every
        # angle, which makes the wrist much harder to drive into a flip.
        p("orientation_scale", 1.0)
        p("limit_margin", 0.05)

        p("ready_positions", READY_POSITIONS)
        p("ready_duration", 5.0)
        p("park_timeout", 25.0)

        g = lambda n: self.get_parameter(n).value
        self.side = str(g("side"))
        self.ns = str(g("arm_namespace"))
        self.group_name = str(g("group_name"))
        self.base_frame_id = str(g("base_frame_id"))
        self.ee_link = str(g("ee_link"))
        self.arm_joints = list(g("arm_joints"))
        self.lim_lo = np.array([float(v) for v in g("joint_limits_lower")])
        self.lim_hi = np.array([float(v) for v in g("joint_limits_upper")])
        self.dry_run = bool(g("dry_run"))
        self.require_confirmation = bool(g("require_confirmation"))
        self.publish_sim_joint_states = bool(g("publish_sim_joint_states"))
        self.command_mode = str(g("command_mode")).lower()
        if self.command_mode not in ("chunk", "service"):
            raise ValueError(f"command_mode must be 'chunk' or 'service', "
                             f"got {self.command_mode!r}")
        self.chunk_tail = float(g("chunk_tail"))
        self.max_cmd_lag = float(g("max_cmd_lag"))
        self.position_scale = float(g("position_scale"))
        self._pos_map = []
        for spec in [str(s).strip().lower() for s in g("pos_axes")]:
            sign = -1.0 if spec.startswith("-") else 1.0
            ax = spec.lstrip("+-")
            if ax not in ("x", "y", "z"):
                raise ValueError(f"pos_axes entries must be like 'x' or '-z', got {spec!r}")
            self._pos_map.append(("xyz".index(ax), sign))
        if len(self._pos_map) != 3:
            raise ValueError("pos_axes must have exactly 3 entries")
        self.max_hand_speed = float(g("max_hand_speed"))
        self.max_cart_step = float(g("max_cart_step"))
        self.max_joint_velocity = float(g("max_joint_velocity"))
        self.max_ik_jump = float(g("max_ik_jump"))
        self.avoid_collisions = bool(g("avoid_collisions"))
        self.diagnose_ik = bool(g("diagnose_ik"))
        self.rate = float(g("rate"))
        self.window = int(g("filter_window_size"))
        self.deadman_threshold = float(g("deadman_threshold"))
        self.watchdog_timeout = float(g("watchdog_timeout"))
        self.track_orientation = bool(g("track_orientation"))
        self.max_ang_step = float(g("max_ang_step"))
        self.orientation_frame = str(g("orientation_frame")).lower()
        if self.orientation_frame not in ("tool", "base"):
            raise ValueError("orientation_frame must be 'tool' or 'base', got "
                             f"{self.orientation_frame!r}")
        self.hand_axes_rpy = [float(v) for v in g("hand_axes_rpy")]
        self.orientation_scale = float(g("orientation_scale"))
        self._hand_axes_q = tf_transformations.quaternion_from_euler(
            *self.hand_axes_rpy)
        self.limit_margin = float(g("limit_margin"))
        self.ready_positions = [float(v) for v in g("ready_positions")]
        self.ready_duration = float(g("ready_duration"))
        self.park_timeout = float(g("park_timeout"))

        # ---- state -----------------------------------------------------
        self._lock = threading.Lock()
        self._quest_pos = None
        self._quest_ori = None
        self._pos_hist = []
        self._ori_hist = []
        self._last_quest_stamp = 0.0
        self._last_inputs_stamp = 0.0
        self._engaged = False
        self._anchor_hand_pos = None
        self._anchor_hand_ori = None
        self._anchor_robot_pos = None
        self._anchor_robot_ori = None
        # Last COMMANDED state. In chunk mode every chunk continues from here
        # rather than from the measured pose: restarting each chunk at the
        # measured position would snap the setpoint back to the arm every tick,
        # cancelling the tracking error that MIT mode needs to produce torque
        # (a sawtooth setpoint, which is jitter). Divergence is bounded by
        # max_cmd_lag instead.
        self._cmd_q = None
        self._cmd_pos = None
        self._cmd_ori = None
        self._last_chunk_time = None   # wall time of the last published chunk
        self._last_lag = 0.0           # latest commanded-vs-measured gap (rad)
        self._prev_raw_pos = None      # frame-jump guard state
        self._prev_raw_stamp = 0.0
        self._speed_strikes = 0
        self._frame_jump = None        # reason string when the guard trips
        self._sim_ready = None
        self._sim_publishing = False
        self._policy_active = False
        self._joints = {}
        self._joints_stamp = 0.0
        self._upper_prev = False
        self._gripper_closed = False
        self._armed = False
        self._parked = False
        self._teleop_enabled = False
        self._stats = {"sent": 0, "guard_reject": 0, "jump_reject": 0,
                       "limit_reject": 0, "driver_reject": 0, "recover": 0,
                       "lag_reject": 0, "lag_hold": 0, "vel_clamp": 0,
                       "overrun": 0}
        self._max_period = 0.0         # worst observed loop period (s)

        # ---- interfaces ------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        cbg = ReentrantCallbackGroup()

        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=cbg)
        self.fk_client = self.create_client(
            GetPositionFK, "/compute_fk", callback_group=cbg)
        self.move_ik_cli = self.create_client(
            MoveToPoseIK, f"/{self.ns}/move_to_pose_ik", callback_group=cbg)
        self.enable_cli = self.create_client(
            Trigger, f"/{self.ns}/enable", callback_group=cbg)
        self.disable_cli = self.create_client(
            Trigger, f"/{self.ns}/disable", callback_group=cbg)
        self.safe_home_cli = self.create_client(
            Trigger, f"/{self.ns}/safe_home", callback_group=cbg)
        self.gripper_open_cli = self.create_client(
            RebotGripperCommand, f"/{self.ns}/gripper/open", callback_group=cbg)
        self.gripper_close_cli = self.create_client(
            RebotGripperCommand, f"/{self.ns}/gripper/close", callback_group=cbg)
        # Gripper commands travel over services, which rosbag cannot record.
        # Mirror every commanded state change (True = close) onto a topic so
        # demo episodes contain the gripper ACTION, not just its state.
        self.gripper_cmd_pub = self.create_publisher(Bool, "~/gripper_cmd", 10)
        self.traj_client = ActionClient(
            self, FollowJointTrajectory, f"/{self.ns}/follow_joint_trajectory",
            callback_group=cbg)

        # Chunk-mode command path. The driver's ChunkExecutor owns the smoothing:
        # cubic Hermite between our waypoints at policy_control_hz (100 Hz default),
        # analytic derivative as velocity feedforward, temporal ensembling across
        # overlapping chunks (favor=newest, so the live chunk dominates),
        # joint-limit clip and a per-tick velocity clamp.
        self.chunk_pub = self.create_publisher(
            JointTrajectory, f"/{self.ns}/policy/action_chunk", 10,
            callback_group=cbg)
        self.policy_start_cli = self.create_client(
            Trigger, f"/{self.ns}/policy/start", callback_group=cbg)
        self.policy_stop_cli = self.create_client(
            Trigger, f"/{self.ns}/policy/stop", callback_group=cbg)

        # Created ONLY in dry_run, so this node can never compete with the real
        # driver for the topic robot_state_publisher and RViz are reading.
        # RELIABLE on purpose: robot_state_publisher subscribes reliably, and a
        # BEST_EFFORT publisher would be silently dropped (the same QoS trap the
        # driver falls into).
        self.sim_joint_pub = None
        if self.dry_run and self.publish_sim_joint_states:
            self.sim_joint_pub = self.create_publisher(
                JointState, f"/{self.ns}/joint_states",
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
                callback_group=cbg)

        # Live diagnostics: the _stats counters used to be printed only at
        # exit, so a session rejecting 90% of its ticks looked identical to a
        # healthy one until Ctrl-C. 1 Hz JSON on ~/diagnostics.
        self.diag_pub = self.create_publisher(
            String, "~/diagnostics", 10, callback_group=cbg)
        self.create_timer(1.0, self._publish_diagnostics, callback_group=cbg)

        self.create_subscription(PoseStamped, f"/q2r_{self.side}_hand_pose",
                                 self._on_pose, 10, callback_group=cbg)
        self.create_subscription(OVR2ROSInputs, f"/q2r_{self.side}_hand_inputs",
                                 self._on_inputs, 10, callback_group=cbg)
        # The driver publishes joint states with BEST_EFFORT (sensor QoS). A
        # RELIABLE subscription is INCOMPATIBLE with that and silently receives
        # nothing ("offering incompatible QoS ... No messages will be received").
        # BEST_EFFORT here is compatible with both best-effort and reliable
        # publishers, so it works against the driver and the mock stack alike.
        joint_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, f"/{self.ns}/joint_states",
                                 self._on_joints, joint_qos, callback_group=cbg)
        self.create_subscription(JointState, "/joint_states",
                                 self._on_joints, joint_qos, callback_group=cbg)

    # ================================================================
    # subscriptions
    # ================================================================

    def _on_joints(self, msg):
        with self._lock:
            for n, q in zip(msg.name, msg.position):
                self._joints[n] = q
            self._joints_stamp = time.time()

    def _on_pose(self, msg):
        o, pt = msg.pose.orientation, msg.pose.position
        ori = np.array([o.x, o.y, o.z, o.w])
        norm = float(np.linalg.norm(ori))
        if not (0.99 < norm < 1.01):
            self.get_logger().warn(
                f"rejecting pose: quaternion norm {norm:.4f} != 1 (bad decode?)",
                throttle_duration_sec=2.0)
            return
        pos = np.array([pt.x, pt.y, pt.z])
        with self._lock:
            now = time.time()
            self._last_quest_stamp = now
            # Frame-jump guard: the app's relative frame follows the headset
            # until aligned (A+B), so looking around slews the apparent hand
            # position far faster than a real hand moves. Two consecutive
            # over-speed samples while engaged -> flag for disengage (the
            # teleop loop acts on it; publishing from here would race it).
            if self.max_hand_speed > 0.0 and self._prev_raw_pos is not None:
                dt_s = now - self._prev_raw_stamp
                if 1e-3 < dt_s < 0.1:
                    speed = float(np.linalg.norm(pos - self._prev_raw_pos)) / dt_s
                    if speed > self.max_hand_speed:
                        self._speed_strikes += 1
                        if self._speed_strikes >= 2 and self._engaged:
                            self._frame_jump = (
                                f"hand pose moving at {speed:.1f} m/s "
                                f"(> {self.max_hand_speed:.1f}) - head motion or "
                                "tracking jump, not a hand. Align the app frame "
                                "(A+B on the right controller), then re-squeeze.")
                    else:
                        self._speed_strikes = 0
            self._prev_raw_pos = pos
            self._prev_raw_stamp = now
            self._pos_hist.append(pos)
            self._ori_hist.append(ori)
            if len(self._pos_hist) > self.window:
                self._pos_hist.pop(0)
                self._ori_hist.pop(0)
            if len(self._pos_hist) < self.window:
                return
            self._quest_pos = np.mean(self._pos_hist, axis=0)
            ref = self._ori_hist[0]   # sign-align: q and -q are one rotation
            aligned = [q if float(np.dot(q, ref)) >= 0.0 else -q for q in self._ori_hist]
            avg = np.mean(aligned, axis=0)
            n = float(np.linalg.norm(avg))
            self._quest_ori = (avg / n) if n > 1e-9 else None

    def _on_inputs(self, msg):
        engage = msg.press_index >= self.deadman_threshold
        fire_gripper = False
        brake_q = None
        with self._lock:
            self._last_inputs_stamp = time.time()
            if not self._teleop_enabled:
                self._upper_prev = msg.button_upper
                return
            if engage and not self._engaged:
                self._engaged = True
                self._anchor_hand_pos = None
                self._speed_strikes = 0
                self._frame_jump = None
                self.get_logger().info(">>> TRIGGER HELD - arm ENGAGED (re-anchoring)")
            elif not engage and self._engaged:
                self._engaged = False
                self._anchor_hand_pos = None
                brake_q = None if self._cmd_q is None else self._cmd_q.copy()
                self.get_logger().info("<<< trigger released - arm HOLDING")
            fire_gripper = msg.button_upper and not self._upper_prev
            self._upper_prev = msg.button_upper
        if brake_q is not None:
            self._stop_moving(brake_q)
        if fire_gripper:
            self._toggle_gripper()

    # ================================================================
    # helpers
    # ================================================================

    def _measured_q(self):
        with self._lock:
            if any(j not in self._joints for j in self.arm_joints):
                return None, 0.0
            return (np.array([self._joints[j] for j in self.arm_joints]),
                    self._joints_stamp)

    def _measured_ee(self, q=None):
        """End-effector pose from MoveIt FK on the measured joints.

        NOT from TF. This used to do a TF lookup, and that was the bug that made
        teleop work in simulation and fail on hardware: TF was 50 mm away from
        FK of the very same joint values (measured), so every IK request paired a
        position and an orientation that no configuration near the seed actually
        has. IK could only satisfy both by flipping the wrist -- joint6 = +3.12
        from a seed of 0.0 -- which the guard then rejected, every single tick.

        Using the same model as the IK solver makes the anchor pose consistent
        with the seed by construction, so TF being stale, doubly-published or
        QoS-starved can no longer affect control. TF stays in use only for the
        preflight cross-check, where a disagreement is reported rather than
        silently steering the arm.
        """
        if q is None:
            q, _ = self._measured_q()
            if q is None:
                return None, None
        got = self._fk(q)
        if got is None:
            self.get_logger().warn("FK unavailable - is move_group running?",
                                   throttle_duration_sec=2.0)
            return None, None
        return got

    def _tf_ee(self):
        """TF's opinion of the EE pose. Diagnostics only - see _measured_ee()."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame_id, self.ee_link, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF unavailable: {e}", throttle_duration_sec=2.0)
            return None, None
        t, r = tf.transform.translation, tf.transform.rotation
        return (np.array([t.x, t.y, t.z]), np.array([r.x, r.y, r.z, r.w]))

    def _within_limits(self, q, expand=0.0):
        """Limit check. `expand` widens the window on both sides.

        Commanded targets are checked with expand=0 so they stay `limit_margin`
        clear of the hard stops. The arm's MEASURED position is checked with a
        generous expand, because a one-sided joint at rest legitimately sits on
        its limit - RS joint2/joint3 have lower=0.0 and home is ~0.0, which the
        margin would otherwise flag as illegal (and encoder zeroing can even
        read slightly negative).
        """
        lo = self.lim_lo + self.limit_margin - expand
        hi = self.lim_hi - self.limit_margin + expand
        bad = [self.arm_joints[i] for i in range(len(q))
               if q[i] < lo[i] or q[i] > hi[i]]
        return (len(bad) == 0), bad

    @staticmethod
    def _scale_rotation(q, s):
        """Same rotation axis, angle multiplied by s."""
        if abs(s - 1.0) < 1e-9:
            return q
        q = np.asarray(q, dtype=float)
        if q[3] < 0.0:                 # canonical hemisphere, so the angle is
            q = -q                     # the short way round before scaling
        w = max(-1.0, min(1.0, float(q[3])))
        angle = 2.0 * math.acos(w)
        axis = q[:3]
        n = float(np.linalg.norm(axis))
        if n < 1e-9 or angle < 1e-9:
            return np.array([0.0, 0.0, 0.0, 1.0])
        axis = axis / n
        half = 0.5 * angle * s
        return np.concatenate([axis * math.sin(half), [math.cos(half)]])

    def _map_orientation(self, hand_ori, a_ho, a_ro):
        """Hand orientation -> tool orientation target, anchored.

        Both modes reduce to the anchor pose at the moment of the squeeze
        (delta == identity), so re-anchoring never jumps.

        "tool":  delta = inv(anchor_hand) * hand        (in the HAND's frame)
                 target = anchor_tool * delta           (in the TOOL's frame)
        "base":  delta = hand * inv(anchor_hand)        (in the QUEST world frame)
                 target = delta * anchor_tool           (in base_link)

        The axis remap is applied by CONJUGATION (R * delta * inv(R)), which
        re-expresses the same rotation about re-oriented axes - it must not be a
        plain multiply, which would instead add a constant rotation and break the
        anchor identity.
        """
        inv_a_ho = tf_transformations.quaternion_inverse(a_ho)
        if self.orientation_frame == "tool":
            delta = tf_transformations.quaternion_multiply(inv_a_ho, hand_ori)
        else:
            delta = tf_transformations.quaternion_multiply(hand_ori, inv_a_ho)

        if any(abs(v) > 1e-12 for v in self.hand_axes_rpy):
            R = self._hand_axes_q
            delta = tf_transformations.quaternion_multiply(
                tf_transformations.quaternion_multiply(R, delta),
                tf_transformations.quaternion_inverse(R))

        delta = self._scale_rotation(delta, self.orientation_scale)

        if self.orientation_frame == "tool":
            target = tf_transformations.quaternion_multiply(a_ro, delta)
        else:
            target = tf_transformations.quaternion_multiply(delta, a_ro)
        return np.asarray(target) / np.linalg.norm(target)

    def _limit_rotation(self, q_from, q_to):
        """Slerp q_to back toward q_from so the step is <= max_ang_step radians."""
        return self._slerp_limited(q_from, q_to, self.max_ang_step)

    def _limit_rotation_to(self, q_from, q_to, ratio):
        """Slerp q_to back toward q_from, keeping `ratio` of the angle."""
        a = np.asarray(q_from, dtype=float)
        b = np.asarray(q_to, dtype=float)
        dot = float(np.dot(a, b))
        if dot < 0.0:
            b, dot = -b, -dot
        angle = 2.0 * math.acos(max(-1.0, min(1.0, dot)))
        return self._slerp_limited(a, b, angle * ratio)

    def _slerp_limited(self, q_from, q_to, max_angle):
        """Slerp q_to toward q_from so the geodesic step is <= max_angle."""
        a = np.asarray(q_from, dtype=float)
        b = np.asarray(q_to, dtype=float)
        dot = float(np.dot(a, b))
        if dot < 0.0:                      # same rotation, opposite sign
            b, dot = -b, -dot
        dot = max(-1.0, min(1.0, dot))
        angle = 2.0 * math.acos(dot)       # geodesic angle between the two
        if angle <= max_angle or angle < 1e-9:
            return b / np.linalg.norm(b)
        t = max_angle / angle
        s = math.sin(angle / 2.0)
        if s < 1e-9:
            return a / np.linalg.norm(a)
        w0 = math.sin((1.0 - t) * angle / 2.0) / s
        w1 = math.sin(t * angle / 2.0) / s
        out = w0 * a + w1 * b
        return out / np.linalg.norm(out)

    def _fk(self, q):
        """Forward kinematics via MoveIt. Returns (pos, quat) or None."""
        req = GetPositionFK.Request()
        req.header.frame_id = self.base_frame_id
        req.fk_link_names = [self.ee_link]
        js = JointState()
        js.name = list(self.arm_joints)
        js.position = [float(v) for v in q]
        req.robot_state.joint_state = js
        res = self._call_sync(self.fk_client, req, 2.0, "compute_fk")
        if res is None or not res.pose_stamped:
            return None
        p = res.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w]))

    def _sim_state(self):
        """Stand-in for the measured state during a dry run.

        A dry run never executes the ready-pose move, so the arm stays wherever
        it is - in practice all-zeros, where joint2/joint3 sit on their lower
        limit and almost any Cartesian delta needs them to go NEGATIVE. MoveIt
        then rejects every single target with -31 and the rehearsal tells you
        nothing (it does not even reach the point of printing a target).

        So: simulate perfect tracking from the ready pose. Before the first
        command that is FK(ready_positions); afterwards it is whatever we last
        commanded, which is exactly what the real arm would be converging on.
        """
        with self._lock:
            if self._cmd_q is not None:
                return (self._cmd_q.copy(), self._cmd_pos.copy(),
                        self._cmd_ori.copy())
        if self._sim_ready is None:
            q = np.array(self.ready_positions)
            got = self._fk(q)
            if got is None:
                self.get_logger().error(
                    "dry run needs /compute_fk to simulate the ready pose",
                    throttle_duration_sec=5.0)
                return None
            self._sim_ready = (q, got[0], got[1])
            self.get_logger().info(
                f"dry run simulating from ready pose {np.round(q, 3).tolist()} "
                f"-> ee {np.round(got[0], 4).tolist()}")
        return tuple(v.copy() for v in self._sim_ready)

    def _driver_instance_count(self):
        """How many reBotArmController instances are on the graph.

        More than one is CATASTROPHIC and not obvious from any single terminal.
        Each instance opens its own 500 Hz MIT control loop on the same CAN bus
        with its own _q_target, so two of them fight over the same motor IDs:
        one drives to a new target while the other keeps commanding the pose it
        last latched, at kp=150 with no velocity ceiling. The arm thrashes with
        nobody having touched the controller. It also shows up as
        "more than one action server for /rebotarm/follow_joint_trajectory".
        """
        names = [n for n, _ns in self.get_node_names_and_namespaces()
                 if n == "reBotArmController"]
        # Discount our own simulated publisher, or a dry run looks like a driver.
        pubs = self.count_publishers(f"/{self.ns}/joint_states")
        if self.sim_joint_pub is not None:
            pubs -= 1
        return max(len(names), pubs)

    def _call_sync(self, client, request, timeout, label, quiet=False):
        """Blocking call, safe from a worker thread (the executor spins elsewhere)."""
        if not client.wait_for_service(timeout_sec=2.0):
            if not quiet:
                self.get_logger().error(f"{label}: service unavailable")
            return None
        future = client.call_async(request)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if future.done():
                try:
                    return future.result()
                except Exception as e:
                    self.get_logger().error(f"{label} raised: {e}")
                    return None
            time.sleep(0.005)
        if not quiet:
            self.get_logger().error(f"{label}: timed out after {timeout:.2f}s")
        return None

    def _pose_msg(self, pos, quat):
        pose = Pose()
        pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        return pose

    def _toggle_gripper(self):
        if self.dry_run:
            self.get_logger().info("[DRY RUN] would toggle gripper")
            self._gripper_closed = not self._gripper_closed
            return
        cli = self.gripper_open_cli if self._gripper_closed else self.gripper_close_cli
        want = "OPEN" if self._gripper_closed else "CLOSE"
        req = RebotGripperCommand.Request()
        req.position = 0.0
        # 4 s: the driver's grasp-aware close ramps the target (~1.7 s for a
        # full stroke at the default 3 rad/s) instead of stepping it.
        req.timeout = 4.0
        res = self._call_sync(cli, req, 6.0, f"gripper {want}")
        # Flip on ANY completed service call, success or not. The gripper is
        # commanded toward `want` either way, and the old success-only flip
        # wedged the toggle: closing on an object reported "timeout"
        # (success=False), so _gripper_closed stayed False and the button
        # could only ever send CLOSE again -- never OPEN -- while actually
        # holding something. Only a dead/unreachable service leaves the
        # state untouched.
        if res is not None:
            self._gripper_closed = not self._gripper_closed
            self.gripper_cmd_pub.publish(Bool(data=self._gripper_closed))
            detail = getattr(res, "message", "") or ""
            pos = float(getattr(res, "reached_position", 0.0))
            if res.success:
                self.get_logger().info(
                    f"gripper -> {want} OK ({detail}, at {pos:.2f} rad)")
            else:
                self.get_logger().warn(
                    f"gripper -> {want} reported '{detail}' at {pos:.2f} rad; "
                    "assuming commanded state anyway")
        else:
            self.get_logger().warn(f"gripper {want} failed (service unreachable)")

    def _send_joint_trajectory(self, positions, duration, label):
        """One coordinated joint-space move via the driver's action server."""
        if self.dry_run:
            self.get_logger().info(
                f"[DRY RUN] {label}: would move to "
                f"{[round(float(v), 3) for v in positions]} over {duration:.1f}s")
            return True
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"{label}: follow_joint_trajectory unavailable")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(self.arm_joints)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.time_from_start = Duration(sec=int(duration),
                                      nanosec=int((duration % 1.0) * 1e9))
        goal.trajectory.points = [pt]

        send_future = self.traj_client.send_goal_async(goal)
        deadline = time.time() + 5.0
        while time.time() < deadline and not send_future.done():
            time.sleep(0.02)
        if not send_future.done():
            self.get_logger().error(f"{label}: goal send timed out")
            return False
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{label}: goal rejected by driver")
            return False

        result_future = handle.get_result_async()
        deadline = time.time() + duration + 10.0
        while time.time() < deadline and not result_future.done():
            time.sleep(0.05)
        if not result_future.done():
            self.get_logger().warn(f"{label}: result timed out")
            return False
        self.get_logger().info(f"{label}: complete")
        return True

    # ================================================================
    # preflight / confirm / ready / park
    # ================================================================

    def preflight(self):
        self.get_logger().info("=== PREFLIGHT ===")
        ok = True

        def check(label, good, detail=""):
            nonlocal ok
            self.get_logger().info(f"  [{'PASS' if good else 'FAIL'}] {label} {detail}")
            ok = ok and bool(good)
            return good

        check("MoveIt /compute_ik (IK guard)",
              self.ik_client.wait_for_service(timeout_sec=5.0),
              "(ros2 launch rebotarm_moveit_config hardware.launch.py model:=rs)")

        # Do this before anything else can move the arm. A dry run is fine with
        # NO driver (that is the recommended way to run one); what is never fine
        # is two of them.
        n_drv = self._driver_instance_count()
        if self.dry_run:
            check("no duplicate driver", n_drv <= 1, f"({n_drv} running)")
        else:
            check("exactly one driver instance", n_drv == 1,
                  f"(found {n_drv}) -> two drivers FIGHT over the same motors. "
                  "pkill -f reBotArmController, then relaunch driver.launch.py once"
                  if n_drv != 1 else "")

        if self.dry_run:
            self.get_logger().info("  [SKIP] driver interfaces (dry_run)")
            # Start simulating NOW, before the joint-state and TF checks below.
            # With no driver there is nothing publishing joint states, so
            # robot_state_publisher cannot compute TF for any movable joint and
            # base_link -> gripper_tcp does not exist at all. Publishing the
            # simulated state first is what makes those checks meaningful (and
            # is what puts the ghost arm in RViz).
            self._start_sim_publishing()
            if self.sim_joint_pub is not None:
                for _ in range(20):
                    self._publish_sim_joints()
                    time.sleep(0.05)
        elif self.command_mode == "chunk":
            check(f"/{self.ns}/policy/start (chunk write path)",
                  self.policy_start_cli.wait_for_service(timeout_sec=5.0),
                  "(driver needs policy_enabled:=true - it is by default)")
            check(f"/{self.ns}/policy/stop",
                  self.policy_stop_cli.wait_for_service(timeout_sec=5.0))
            check(f"/{self.ns}/enable",
                  self.enable_cli.wait_for_service(timeout_sec=5.0))
            check(f"/{self.ns}/safe_home",
                  self.safe_home_cli.wait_for_service(timeout_sec=5.0),
                  "REQUIRED for park-on-exit")
            check(f"/{self.ns}/follow_joint_trajectory",
                  self.traj_client.wait_for_server(timeout_sec=5.0),
                  "(ready pose + park fallback)")
        else:
            check(f"/{self.ns}/move_to_pose_ik (write path)",
                  self.move_ik_cli.wait_for_service(timeout_sec=5.0),
                  "(ros2 launch rebotarm_bringup driver.launch.py model:=rs)")
            check(f"/{self.ns}/enable",
                  self.enable_cli.wait_for_service(timeout_sec=5.0))
            check(f"/{self.ns}/safe_home",
                  self.safe_home_cli.wait_for_service(timeout_sec=5.0),
                  "REQUIRED for park-on-exit")
            check(f"/{self.ns}/follow_joint_trajectory",
                  self.traj_client.wait_for_server(timeout_sec=5.0),
                  "(ready pose + park fallback)")

        deadline = time.time() + 6.0
        while time.time() < deadline:
            q, _ = self._measured_q()
            with self._lock:
                have = self._quest_pos is not None
            if q is not None and have:
                break
            time.sleep(0.1)

        q, _ = self._measured_q()
        check("simulated joint states echoed back" if self._sim_publishing
              else "joint states for all arm joints", q is not None,
              f"({np.round(q, 3).tolist()})" if q is not None
              else "(none seen)" if not self.dry_run else
              "(none seen - is move_group up? the simulation needs /compute_fk)")

        with self._lock:
            qa = time.time() - self._last_quest_stamp if self._last_quest_stamp else None
            ia = time.time() - self._last_inputs_stamp if self._last_inputs_stamp else None
        check("Quest pose stream", qa is not None and qa < 1.0,
              f"(age {qa:.2f}s)" if qa is not None else "(no data)")
        check("Quest inputs stream", ia is not None and ia < 1.0,
              f"(age {ia:.2f}s)" if ia is not None else "(no data)")

        ee, _ = self._measured_ee(q) if q is not None else (None, None)
        check(f"FK {self.base_frame_id}->{self.ee_link} (control reference)",
              ee is not None,
              f"({np.round(ee, 4).tolist()})" if ee is not None else
              "(needs /compute_fk from move_group)")

        # Cross-check, not a gate. Control uses FK; TF only has to be sane enough
        # for RViz. A large disagreement means something is wrong with TF (a
        # second robot_state_publisher, a stale or QoS-starved subscription) and
        # it used to silently wreck teleop, so it is worth surfacing loudly.
        if ee is not None:
            tf_ee, _ = self._tf_ee()
            if tf_ee is None:
                self.get_logger().warn(
                    f"  [WARN] TF {self.base_frame_id}->{self.ee_link} missing "
                    "- RViz will not render correctly (control is unaffected)")
            else:
                gap = float(np.linalg.norm(tf_ee - ee))
                msg = (f"  [{'ok' if gap < 0.01 else 'WARN'}] FK vs TF agreement: "
                       f"{gap * 1000:.1f} mm  (TF {np.round(tf_ee, 4).tolist()})")
                if gap < 0.01:
                    self.get_logger().info(msg)
                else:
                    self.get_logger().warn(
                        msg + " -> TF disagrees with the kinematic model. Check "
                        "for a second robot_state_publisher: "
                        "pgrep -af robot_state_publisher")

        if q is not None:
            # generous window: resting on a one-sided limit is normal
            inside, bad = self._within_limits(q, expand=self.limit_margin + 0.10)
            check("current joints within limits (+tol)", inside,
                  "" if inside else f"(outside: {bad} -> arm may be past a hard stop)")

        self.get_logger().info(
            f"  command_mode={self.command_mode}"
            + ("  (chunks -> driver Hermite upsampling + velocity feedforward)"
               if self.command_mode == "chunk"
               else "  (position-only setpoints; EXPECT JITTER in MIT mode)"))
        self.get_logger().info(
            "  orientation: OFF (translation only)" if not self.track_orientation
            else f"  orientation: ON  frame={self.orientation_frame}"
                 f"  scale={self.orientation_scale}"
                 f"  max_ang_step={self.max_ang_step} rad"
                 f" (~{math.degrees(self.max_ang_step):.1f} deg/cmd)"
                 f"  hand_axes_rpy={self.hand_axes_rpy}")
        self.get_logger().info(
            f"  bounds: scale={self.position_scale}  max_cart_step={self.max_cart_step} m"
            f"  -> ~{self.max_cart_step * self.rate:.2f} m/s max"
            f"  max_ik_jump={self.max_ik_jump} rad")
        self.get_logger().info(
            "  pos_axes=" + str(["%s%s" % ("" if s > 0 else "-", "xyz"[i])
                                 for i, s in self._pos_map])
            + f"  frame-jump guard: "
            + (f"{self.max_hand_speed:.1f} m/s" if self.max_hand_speed > 0 else "OFF")
            + "  (REMINDER: align the app frame with A+B on the right "
              "controller before driving - unaligned, it follows your head)")
        self.get_logger().info("=== PREFLIGHT " + ("OK ===" if ok else "FAILED ==="))
        return ok

    def confirm(self):
        if not self.require_confirmation:
            return True
        dry = "  (DRY RUN - nothing will be sent)" if self.dry_run else ""
        print(f"""
============================================================
  REAL ROBOT MOTION{dry}
------------------------------------------------------------
  RS variant / MIT control mode: there is NO firmware velocity
  ceiling. Safety comes from small bounded steps in this script.

  The arm will first move to the ready pose:
      {self.ready_positions}
  Then it follows your {self.side.upper()} controller WHILE THE
  TRIGGER IS HELD. Release the trigger to stop.
  Upper button toggles the gripper.

  On exit (Ctrl-C) the arm returns to ALL JOINTS ZERO.

  Clear the workspace. Keep the e-stop within reach.
------------------------------------------------------------
  Type ARM then Enter to proceed, anything else to abort.
============================================================
""", flush=True)
        try:
            answer = sys.stdin.readline().strip()
        except Exception:
            return False
        if answer != "ARM":
            self.get_logger().warn(f"aborted (got {answer!r}, expected 'ARM')")
            return False
        return True

    def goto_ready(self):
        target = np.array(self.ready_positions)
        inside, bad = self._within_limits(target)
        if not inside:
            self.get_logger().error(f"ready_positions outside limits: {bad}")
            return False
        return self._send_joint_trajectory(target, self.ready_duration, "ready pose")

    def start_policy(self):
        """Hand the arm to the driver's chunk executor.

        Nothing we publish moves anything until this succeeds - the executor
        ignores chunks while inactive, by design, so a stray publisher cannot
        grab the arm. Must be called AFTER enable and AFTER the ready pose
        (the executor stands down during TRAJ_RUNNING anyway).
        """
        if self.dry_run or self.command_mode != "chunk":
            return True
        res = self._call_sync(self.policy_start_cli, Trigger.Request(),
                              5.0, "policy/start")
        if res is None or not res.success:
            why = "no response" if res is None else res.message
            self.get_logger().error(f"could not start the chunk executor: {why}")
            return False
        self._policy_active = True
        self.get_logger().info("chunk executor active")
        return True

    def stop_policy(self):
        """Release the arm. The executor then holds position rather than coasting."""
        if not self._policy_active:
            return
        self._policy_active = False
        res = self._call_sync(self.policy_stop_cli, Trigger.Request(),
                              5.0, "policy/stop")
        if res is None or not res.success:
            self.get_logger().warn(
                "policy/stop did not confirm - safe_home will preempt it anyway")
        else:
            self.get_logger().info("chunk executor stopped")

    def park(self):
        """Return to all-joints-zero, then disable. Idempotent."""
        with self._lock:
            if self._parked:
                return
            never_armed = not self._armed
            self._parked = True
            self._teleop_enabled = False
            self._engaged = False

        if never_armed:
            self.get_logger().info("arm was never armed - nothing to park")
            return

        # Release the chunk executor first: it must not be fighting safe_home
        # for the joint target. (It also stands down on SAFE_HOMING, but that
        # relies on winning a race we do not need to run.)
        try:
            self.stop_policy()
        except Exception as e:
            self.get_logger().warn(f"policy/stop failed during park: {e}")

        self.get_logger().info("=== PARKING: returning to all joints zero ===")
        if self.dry_run:
            self.get_logger().info("[DRY RUN] would call safe_home then disable")
            return

        res = self._call_sync(self.safe_home_cli, Trigger.Request(),
                              self.park_timeout, "safe_home")
        if res is not None and res.success:
            self.get_logger().info(f"safe_home OK: {res.message}")
        else:
            self.get_logger().error(
                "safe_home failed - falling back to a slow trajectory to zero")
            self._send_joint_trajectory(np.zeros(len(self.arm_joints)),
                                        max(8.0, self.ready_duration), "park fallback")

        q, _ = self._measured_q()
        if q is not None:
            worst = float(np.max(np.abs(q)))
            self.get_logger().info(
                f"final joints: {np.round(q, 4).tolist()}  (max |q| = {worst:.4f})")
            if worst > 0.05:
                self.get_logger().error(
                    "ARM IS NOT AT ZERO - inspect it before the next run")

        res = self._call_sync(self.disable_cli, Trigger.Request(), 8.0, "disable")
        if res is not None and res.success:
            self.get_logger().info("arm disabled")
        self.get_logger().info("stats: " + "  ".join(
            f"{k}={v}" for k, v in self._stats.items())
            + f"  max_period={self._max_period * 1000:.0f}ms")

    # ================================================================
    # teleop
    # ================================================================

    def _publish_diagnostics(self):
        """1 Hz health snapshot - counters, lag, timing - while driving."""
        with self._lock:
            payload = dict(self._stats)
            payload["lag"] = round(self._last_lag, 4)
            payload["engaged"] = self._engaged
            payload["max_period_ms"] = round(self._max_period * 1000.0, 1)
        msg = String()
        msg.data = json.dumps(payload)
        self.diag_pub.publish(msg)

    def _publish_sim_joints(self):
        """Push the simulated state to /joint_states so RViz can render it."""
        if self.sim_joint_pub is None or not self._sim_publishing:
            return
        sim = self._sim_state()
        if sim is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.arm_joints)
        msg.position = [float(v) for v in sim[0]]
        self.sim_joint_pub.publish(msg)

    def _start_sim_publishing(self):
        """Decide whether to drive RViz from the simulation. Idempotent."""
        if self.sim_joint_pub is None or self._sim_publishing:
            return
        # Two publishers on one /joint_states makes RViz flicker between the real
        # arm and the simulation, which is worse than either alone.
        if self._driver_instance_count() > 0:
            self.get_logger().warn(
                "the driver is publishing joint states too - NOT publishing "
                "simulated ones (run the dry run without driver.launch.py to "
                "drive RViz from the simulation)")
            self.sim_joint_pub = None
            return
        self._sim_publishing = True
        self.get_logger().info(
            "publishing SIMULATED joint states - RViz follows your controller "
            "with no hardware in the loop")

    def teleop_forever(self):
        self._teleop_enabled = True
        self._start_sim_publishing()
        self.get_logger().info("=== TELEOP ACTIVE - hold the trigger to move ===")
        period = 1.0 / self.rate
        while rclpy.ok() and not self._parked:
            t0 = time.time()
            try:
                self._step()
                self._publish_sim_joints()
            except Exception as e:
                self.get_logger().error(f"teleop step failed, disengaging: {e}")
                with self._lock:
                    self._engaged = False
                    self._anchor_hand_pos = None
            dt = time.time() - t0
            self._max_period = max(self._max_period, dt)
            if dt > 1.5 * period:
                # No catch-up exists by design (a burst of catch-up ticks
                # would be worse), so late ticks silently lower the command
                # rate - which the chunk timing now compensates for, but it
                # still deserves to be visible.
                self._stats["overrun"] += 1
                self.get_logger().warn(
                    f"teleop loop overran: {dt * 1000:.0f} ms "
                    f"(nominal {period * 1000:.0f} ms)",
                    throttle_duration_sec=5.0)
            if dt < period:
                time.sleep(period - dt)

    def _disengage(self, why):
        with self._lock:
            self._engaged = False
            self._anchor_hand_pos = None
            hold = None if self._cmd_q is None else self._cmd_q.copy()
        self._stop_moving(hold)
        self.get_logger().error(f"DISENGAGED: {why}")

    def _stop_moving(self, hold_q):
        """Bring the setpoint to a halt wherever it currently is."""
        if hold_q is None or self.dry_run or self.command_mode != "chunk":
            return
        try:
            self._brake_chunk(hold_q)
        except Exception as e:
            self.get_logger().error(f"could not publish brake chunk: {e}")

    def _remap_pos(self, delta):
        """Apply the pos_axes signed permutation to a translation delta."""
        return np.array([sign * delta[idx] for idx, sign in self._pos_map])

    def _step(self):
        now = time.time()
        with self._lock:
            if not self._engaged:
                return
            jump = self._frame_jump
            self._frame_jump = None
            quest_age = now - self._last_quest_stamp
            inputs_age = now - self._last_inputs_stamp
            hand_pos = None if self._quest_pos is None else self._quest_pos.copy()
            hand_ori = None if self._quest_ori is None else self._quest_ori.copy()

        if jump is not None:
            self._stats["frame_jump"] = self._stats.get("frame_jump", 0) + 1
            self._disengage(f"FRAME-JUMP GUARD - {jump}")
            return

        if quest_age > self.watchdog_timeout or inputs_age > self.watchdog_timeout:
            self._disengage(f"WATCHDOG - Quest data stale "
                            f"(pose {quest_age:.2f}s, inputs {inputs_age:.2f}s)")
            return
        if hand_pos is None or hand_ori is None:
            return

        if self.dry_run:
            # Simulate perfect tracking from the ready pose instead of reading
            # the (unmoved) hardware - see _sim_state().
            sim = self._sim_state()
            if sim is None:
                return
            q_now, ee_pos, ee_ori = sim
        else:
            q_now, q_stamp = self._measured_q()
            if q_now is None or (now - q_stamp) > 1.0:
                self.get_logger().warn("joint states stale - skipping",
                                       throttle_duration_sec=1.0)
                return
            ee_pos, ee_ori = self._measured_ee(q_now)
            if ee_pos is None:
                return

        with self._lock:
            if self._anchor_hand_pos is None:
                self._anchor_hand_pos = hand_pos
                self._anchor_hand_ori = hand_ori
                self._anchor_robot_pos = ee_pos
                self._anchor_robot_ori = ee_ori
                # Re-anchoring also resyncs the commanded state to reality, so a
                # trigger release/re-squeeze is the recovery from any divergence.
                self._cmd_q = q_now.copy()
                self._cmd_pos = ee_pos.copy()
                self._cmd_ori = ee_ori.copy()
                self._last_chunk_time = None  # fresh timing baseline for vff
                self.get_logger().info(
                    f"anchored: ee={np.round(ee_pos, 3).tolist()} "
                    f"hand={np.round(hand_pos, 3).tolist()}")
                return
            a_hp, a_ho = self._anchor_hand_pos, self._anchor_hand_ori
            a_rp, a_ro = self._anchor_robot_pos, self._anchor_robot_ori
            cmd_q = self._cmd_q.copy()
            cmd_pos = self._cmd_pos.copy()
            cmd_ori = self._cmd_ori.copy()

        # The setpoint is allowed to lead the arm - that error IS the MIT torque -
        # but not without limit, or a slow/blocked joint would wind up a large
        # error and then snap when it frees.
        lag = float(np.max(np.abs(cmd_q - q_now)))
        self._last_lag = lag
        if lag > 2.0 * self.max_cmd_lag:
            # Far beyond the working range: something is actually wrong (a stalled
            # joint, a thermal derate, the executor standing down), not just a
            # fast hand. Drop out and make the operator re-anchor.
            self._stats["lag_reject"] += 1
            self._disengage(
                f"commanded target is {lag:.3f} rad ahead of the arm "
                f"(> {2.0 * self.max_cmd_lag:.2f}) - the arm is not following at "
                "all. Squeeze again to resync.")
            return
        if lag > self.max_cmd_lag:
            # Merely saturated: stop ADVANCING and let the arm close the gap.
            # Disengaging here was wrong - it forced a re-squeeze for what is a
            # normal consequence of moving your hand faster than the arm can go.
            # Holding is safe because the target is never more than one
            # max_cart_step ahead of cmd_pos, so it simply waits rather than
            # winding up and surging when the arm catches up.
            #
            # CRITICAL: keep PUBLISHING while holding. Going silent here was a
            # deadlock: ~70 ms after the last chunk expired the executor fell
            # back to holding ITS OWN last (velocity-clamped) target - which is
            # short of cmd_q - so the gap never closed and the lag froze at a
            # constant value until the operator re-squeezed. A hold chunk at
            # cmd_q keeps a live target the executor can keep stepping toward.
            self._stats["lag_hold"] += 1
            if self.command_mode == "chunk" and not self.dry_run:
                self._brake_chunk(cmd_q)
            self.get_logger().warn(
                f"arm is {lag:.3f} rad behind the target - holding the setpoint "
                "until it catches up (move your hand more slowly)",
                throttle_duration_sec=1.0)
            return

        target_pos = a_rp + self._remap_pos(hand_pos - a_hp) * self.position_scale

        # Rate-limit from the last COMMANDED pose, not the measured one. Clamping
        # against measured makes the reachable step shrink as the arm lags, which
        # is a feedback loop that oscillates instead of settling.
        ref_pos = cmd_pos if self.command_mode == "chunk" else ee_pos
        ref_ori = cmd_ori if self.command_mode == "chunk" else ee_ori
        step = target_pos - ref_pos
        n = float(np.linalg.norm(step))
        if n > self.max_cart_step:
            target_pos = ref_pos + step * (self.max_cart_step / n)

        if self.track_orientation:
            target_ori = self._map_orientation(hand_ori, a_ho, a_ro)
            # Rate-limit rotation the same way translation is limited. Without
            # this, a big hand rotation asks IK for a far-away orientation in one
            # tick, and KDL answers with a wrist flip (joint6 -> +/-pi).
            target_ori = self._limit_rotation(ref_ori, target_ori)
        else:
            target_ori = ref_ori

        # --- guard: MoveIt IK must accept it, within limits and no big jump ---
        # Seeded from the commanded solution so consecutive solves stay on the
        # same IK branch; seeding from noisy measured positions lets the solver
        # hop between solutions, which shows up as jitter.
        seed = cmd_q if self.command_mode == "chunk" else q_now
        guard = self._ik_guard(target_pos, target_ori, seed)
        if guard is None:
            return

        # --- joint-space speed limit -------------------------------------
        # The Cartesian clamp above bounds how far the TOOL moves; this bounds
        # how far the JOINTS move, which is what the driver actually limits. Pull
        # the whole step back along the line from the seed - joints and Cartesian
        # target together - so what we record as commanded stays consistent with
        # the joint target we send.
        step_limit = self.max_joint_velocity / self.rate
        dq = guard - seed
        worst = float(np.max(np.abs(dq)))
        if worst > step_limit:
            ratio = step_limit / worst
            self._stats["vel_clamp"] += 1
            guard = seed + dq * ratio
            target_pos = ref_pos + (target_pos - ref_pos) * ratio
            target_ori = self._limit_rotation_to(ref_ori, target_ori, ratio)
            self.get_logger().warn(
                f"joint speed clamp: {worst / step_limit:.1f}x over budget "
                f"({worst:.3f} > {step_limit:.3f} rad/tick) - scaled to "
                f"{ratio:.2f}. Near a singularity, or lower max_cart_step.",
                throttle_duration_sec=2.0)

        if self.dry_run:
            self.get_logger().info(
                f"[DRY RUN] would command ee -> {np.round(target_pos, 4).tolist()}",
                throttle_duration_sec=0.5)
            self._stats["sent"] += 1
            with self._lock:
                self._cmd_q = guard.copy()
                self._cmd_pos = target_pos.copy()
                self._cmd_ori = target_ori.copy()
            return

        if self.command_mode == "chunk":
            # The guard already solved IK, so reuse its solution as the chunk
            # content. The service path would solve a SECOND time inside the
            # driver, doubling per-tick latency for no benefit.
            self._publish_chunk(cmd_q, guard)
            self._stats["sent"] += 1
            with self._lock:
                self._cmd_q = guard.copy()
                self._cmd_pos = target_pos.copy()
                self._cmd_ori = target_ori.copy()
            return

        req = MoveToPoseIK.Request()
        req.target_pose = self._pose_msg(target_pos, target_ori)
        res = self._call_sync(self.move_ik_cli, req, 0.25, "move_to_pose_ik", quiet=True)
        if res is None:
            self.get_logger().warn("move_to_pose_ik no response - skipping",
                                   throttle_duration_sec=1.0)
            return
        if not res.success:
            self._stats["driver_reject"] += 1
            self.get_logger().warn(f"driver rejected target: {res.message}",
                                   throttle_duration_sec=1.0)
            return
        self._stats["sent"] += 1

        # --- post-check: what the driver actually latched as its joint target ---
        if res.q_solution:
            sol = np.array(res.q_solution[:len(self.arm_joints)])
            jump = float(np.max(np.abs(sol - q_now)))
            if jump > self.max_ik_jump:
                self._stats["recover"] += 1
                # Pull the target back to where the arm is, then stop.
                recover = MoveToPoseIK.Request()
                recover.target_pose = self._pose_msg(ee_pos, ee_ori)
                self._call_sync(self.move_ik_cli, recover, 0.5,
                                "move_to_pose_ik(recover)", quiet=True)
                self._disengage(
                    f"driver latched a {jump:.3f} rad joint jump (> {self.max_ik_jump}); "
                    "target pulled back to current pose. Squeeze again to resume.")

    # ================================================================
    # chunk command path
    # ================================================================

    def _chunk_msg(self, points):
        """points: list of (time_from_start seconds, q array, qd array)."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self.arm_joints)
        for t, q, qd in points:
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            pt.velocities = [float(v) for v in qd]
            pt.time_from_start = Duration(
                sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))
            msg.points.append(pt)
        return msg

    def _publish_chunk(self, q_from, q_to):
        """One command tick as a short chunk the driver can interpolate through.

        Three waypoints describing a constant-velocity segment:

            t=0        q_from    v=vff     (where the setpoint is now)
            t=dt       q_to      v=vff     (where it should be next tick)
            t=dt+tail  q_to      v=0       (hold, so a late chunk leaves no gap)

        The tail exists so that timing noise in this loop cannot drop the executor
        into hold-last-target for a tick, which would be a 20 Hz stutter.

        The tail HOLDS q_to at zero velocity; it must not extrapolate past it.
        An extrapolating tail (q_to + vff*tail) does blend better - overlapping
        chunks then agree on one straight line instead of averaging a moving
        chunk against a stopped one - but it RATCHETS. On every trigger release
        the last chunk's tail keeps the setpoint travelling for up to `tail`
        seconds, so each squeeze-release cycle leaves a net displacement in the
        direction you were last moving. Measured on hardware: ~1 cm per cycle,
        which walked the arm 3 cm up and inward over four squeezes with the
        operator holding still. Committing to a target we did not ask for is
        worse than mild feedforward attenuation, so: hold.

        Keep the tail just over one publish period for that reason - long enough
        to cover jitter, short enough that a stale holding tail is rarely still
        alive when the next chunk lands.

        Supplying velocities explicitly is the whole point: the driver's MIT
        torque is kp*(q_target - q) - kd*(qd - qd_target). The move_to_pose_ik
        path leaves whatever qd_target was last latched (it does NOT zero it),
        so kd acts on a stale reference; here the damping acts on velocity
        ERROR against the real segment velocity and the arm is free to move.

        Timing uses the MEASURED interval since the last chunk, not 1/rate.
        The loop has no catch-up: when a tick runs late the real period grows,
        and a vff computed against the nominal period is proportionally too
        fast - the executor overshoots the waypoint, brakes on the hold tail,
        and waits for the late chunk: overshoot/brake/wait at whatever rate
        the loop actually achieves. The tail is likewise sized from the real
        period so it cannot expire before the next chunk lands.
        """
        now = time.time()
        dt_nom = 1.0 / self.rate
        if self._last_chunk_time is None:
            dt = dt_nom
        else:
            dt = min(max(now - self._last_chunk_time, dt_nom), 0.25)
        self._last_chunk_time = now
        vff = (np.asarray(q_to) - np.asarray(q_from)) / dt
        tail = max(self.chunk_tail, 0.75 * dt)
        zero = np.zeros(len(self.arm_joints))
        self.chunk_pub.publish(self._chunk_msg([
            (0.0, q_from, vff),
            (dt, q_to, vff),
            (dt + tail, q_to, zero),
        ]))

    def _brake_chunk(self, q_hold):
        """Cancel the extrapolating tail: hold q_hold at zero velocity.

        Needed on every disengage, otherwise the last chunk's tail keeps the
        setpoint moving for up to `chunk_tail` seconds after the operator has
        released the trigger.
        """
        zero = np.zeros(len(self.arm_joints))
        self.chunk_pub.publish(self._chunk_msg([
            (0.0, q_hold, zero),
            (0.25, q_hold, zero),
        ]))

    def _ik_request(self, pos, quat, seed_q, avoid_collisions, timeout=0.03):
        """Build a GetPositionIK request.

        Arm joints only. Passing the driver's other joints through was tempting
        (to pin down the whole scene state) but wrong: the driver publishes a
        joint called `gripper`, which does not exist in the MoveIt model - that
        model has prismatic `gripper_joint1`/`gripper_joint2`. And it cannot
        matter anyway, because `gripper_end` hangs off `link6` by a FIXED joint,
        so no gripper value can move the IK tip.
        """
        seed = JointState()
        seed.name = list(self.arm_joints)
        seed.position = [float(v) for v in seed_q]

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ee_link
        req.ik_request.avoid_collisions = bool(avoid_collisions)
        req.ik_request.timeout = Duration(sec=0, nanosec=int(timeout * 1e9))
        req.ik_request.robot_state.joint_state = seed
        ps = PoseStamped()
        ps.header.frame_id = self.base_frame_id
        ps.pose = self._pose_msg(pos, quat)
        req.ik_request.pose_stamped = ps
        return req

    def _ik_guard(self, pos, quat, seed_q):
        """MoveIt IK as a veto: reachable, in-limits, and no large joint jump."""
        req = self._ik_request(pos, quat, seed_q, self.avoid_collisions)
        res = self._call_sync(self.ik_client, req, 0.3, "compute_ik", quiet=True)

        if (res is not None and res.error_code.val != 1
                and self.avoid_collisions and self.diagnose_ik):
            # Same request without collision checking. If THAT succeeds, the pose
            # is reachable and the rejection was a collision verdict, not
            # kinematics - which is the single most useful thing to know here.
            probe = self._ik_request(pos, quat, seed_q, False)
            alt = self._call_sync(self.ik_client, probe, 0.3, "compute_ik",
                                  quiet=True)
            if alt is not None and alt.error_code.val == 1:
                self.get_logger().warn(
                    "IK rejected WITH collision checking but succeeds WITHOUT it "
                    "-> this is a self-collision verdict, not an unreachable "
                    "pose. Re-run with -p avoid_collisions:=false to confirm.",
                    throttle_duration_sec=2.0)

        if res is None or res.error_code.val != 1:
            self._stats["guard_reject"] += 1
            code = "none" if res is None else res.error_code.val
            self.get_logger().warn(
                f"IK guard rejected target (code={code}) "
                f"target={np.round(pos, 4).tolist()} "
                f"seed={np.round(seed_q, 3).tolist()} "
                f"avoid_collisions={self.avoid_collisions}",
                throttle_duration_sec=1.0)
            return None

        by_name = dict(zip(res.solution.joint_state.name,
                           res.solution.joint_state.position))
        try:
            sol = np.array([float(by_name[j]) for j in self.arm_joints])
        except KeyError as e:
            self.get_logger().error(f"IK solution missing joint {e}")
            return None

        inside, bad = self._within_limits(sol)
        if not inside:
            self._stats["limit_reject"] += 1
            detail = ", ".join(
                f"{self.arm_joints[i]}={sol[i]:+.3f} "
                f"(limit {self.lim_lo[i]:+.2f}..{self.lim_hi[i]:+.2f})"
                for i in range(len(sol)) if self.arm_joints[i] in bad)
            self.get_logger().warn(
                f"IK guard: solution outside limits -> {detail}  "
                f"seed={np.round(seed_q, 3).tolist()}",
                throttle_duration_sec=1.0)
            return None

        delta = np.abs(sol - seed_q)
        jump = float(np.max(delta))
        if jump > self.max_ik_jump:
            self._stats["jump_reject"] += 1
            worst = self.arm_joints[int(np.argmax(delta))]
            self.get_logger().warn(
                f"IK guard: joint jump {jump:.3f} rad > {self.max_ik_jump} on "
                f"{worst} (wrist flip / singularity)  "
                f"seed={np.round(seed_q, 3).tolist()} "
                f"sol={np.round(sol, 3).tolist()}",
                throttle_duration_sec=1.0)
            return None
        return sol


def main(args=None):
    rclpy.init(args=args)
    node = QuestTeleopReal()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    def _on_signal(signum, _frame):
        node.get_logger().warn(f"signal {signum} received - parking")
        node.park()
        rclpy.try_shutdown()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    code = 0
    try:
        if not node.preflight():
            node.get_logger().error("preflight failed - refusing to move the arm")
            return 1
        if not node.confirm():
            return 1

        if not node.dry_run:
            res = node._call_sync(node.enable_cli, Trigger.Request(), 10.0, "enable")
            if res is None or not res.success:
                node.get_logger().error("could not enable the arm - aborting")
                return 1
            node.get_logger().info("arm enabled")
            node._armed = True     # from here every exit path must park

        if not node.goto_ready():
            node.get_logger().error("could not reach ready pose - aborting")
            return 1
        # Only after the ready-pose trajectory has finished: the chunk executor
        # stands down during TRAJ_RUNNING, and starting it earlier would mean
        # two motion sources racing for the joint target.
        if not node.start_policy():
            node.get_logger().error("could not start the chunk executor - aborting")
            return 1
        node.teleop_forever()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"unhandled error: {e}")
        code = 1
    finally:
        node.park()
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
