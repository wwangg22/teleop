"""
Quest -> Cartesian IK Bridge
----------------------------
Drives a joint-trajectory-controlled arm from Quest controller poses, for robots
that have no Cartesian end-effector controller. It reproduces the anchored
relative ("clutch") mapping used by robot_arm_controller_base.py, but instead of
publishing to a Cartesian controller's target_frame it solves IK via MoveIt's
/compute_ik service and streams a JointTrajectory to a JointTrajectoryController.

Control mapping:
- **Trigger (press_index) is a deadman**: the arm only moves while you hold it.
  Releasing it stops motion and drops the anchor, so the next squeeze re-anchors
  at wherever your hand is - no jump. This replaces the toggle-style clutch.
- **Upper button** toggles the gripper open/closed.
- Hand translation is scaled by `position_scale` (default 0.4) and clamped per
  step by `max_step`, so a fast hand movement cannot command a large jump.

Safety notes:
- Every commanded pose goes through MoveIt IK with collision checking, so
  unreachable or self-colliding targets are rejected rather than clamped.
- Designed to be run against mock_components/GenericSystem (RViz only) first.
  Verify axis directions and scaling there before pointing it at real hardware.

How to Run (RViz/mock first):
    $ ros2 launch rebotarm_moveit_config demo.launch.py
    $ ros2 run q2r2_bringup QuestIKBridge

Options:
    $ ros2 run q2r2_bringup QuestIKBridge --ros-args \
        -p side:=left -p position_scale:=0.6 -p rate:=30.0
"""
import math
import threading

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Point
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

import tf_transformations

from quest2ros.msg import OVR2ROSInputs


class QuestIKBridge(Node):
    def __init__(self):
        super().__init__("quest_ik_bridge")

        # ---- parameters ------------------------------------------------
        self.declare_parameter("side", "right")
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("ee_link", "gripper_tcp")
        self.declare_parameter("arm_joints", ["joint1", "joint2", "joint3",
                                             "joint4", "joint5", "joint6"])
        self.declare_parameter("gripper_joints", ["gripper_joint1", "gripper_joint2"])
        self.declare_parameter("arm_traj_topic", "/rebotarm_controller/joint_trajectory")
        self.declare_parameter("gripper_traj_topic", "/gripper_controller/joint_trajectory")
        self.declare_parameter("position_scale", 0.4)
        self.declare_parameter("max_step", 0.03)          # metres per command
        self.declare_parameter("rate", 25.0)              # IK/command rate (Hz)
        self.declare_parameter("filter_window_size", 8)
        self.declare_parameter("deadman_threshold", 0.5)  # trigger pull to engage
        self.declare_parameter("track_orientation", True)
        self.declare_parameter("time_from_start", 0.12)   # seconds
        self.declare_parameter("gripper_open", 0.0715)
        self.declare_parameter("gripper_closed", 0.0)
        # The mock/real arm starts at all-zeros, which puts joint2 and joint3
        # exactly on their upper limit (0.0) - IK cannot move from there and
        # returns NO_IK_SOLUTION for every target. Move somewhere valid first.
        self.declare_parameter("goto_ready_on_start", True)
        self.declare_parameter("ready_positions", [0.0, -1.0, -1.0, 0.0, 0.5, 0.0])
        self.declare_parameter("ready_duration", 3.0)

        g = lambda n: self.get_parameter(n).value
        self.side = str(g("side"))
        self.group_name = str(g("group_name"))
        self.base_frame_id = str(g("base_frame_id"))
        self.ee_link = str(g("ee_link"))
        self.arm_joints = list(g("arm_joints"))
        self.gripper_joints = list(g("gripper_joints"))
        self.position_scale = float(g("position_scale"))
        self.max_step = float(g("max_step"))
        self.rate = float(g("rate"))
        self.window = int(g("filter_window_size"))
        self.deadman_threshold = float(g("deadman_threshold"))
        self.track_orientation = bool(g("track_orientation"))
        self.time_from_start = float(g("time_from_start"))
        self.gripper_open = float(g("gripper_open"))
        self.gripper_closed = float(g("gripper_closed"))

        # ---- state -----------------------------------------------------
        self._lock = threading.Lock()
        self._quest_pos = None            # latest smoothed hand position
        self._quest_ori = None            # latest smoothed hand orientation
        self._pos_hist = []
        self._ori_hist = []
        self._engaged = False
        self._anchor_hand_pos = None
        self._anchor_hand_ori = None
        self._anchor_robot_pos = None
        self._anchor_robot_ori = None
        self._last_cmd_pos = None
        self._joint_state = None
        self._ik_inflight = False
        self._gripper_closed_state = False
        self._upper_prev = False
        self._ik_fail_streak = 0

        # ---- interfaces ------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        cb = MutuallyExclusiveCallbackGroup()
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik", callback_group=cb)

        self.create_subscription(
            PoseStamped, f"/q2r_{self.side}_hand_pose", self._on_pose, 10)
        self.create_subscription(
            OVR2ROSInputs, f"/q2r_{self.side}_hand_inputs", self._on_inputs, 10)
        # BEST_EFFORT is compatible with both the mock stack (reliable) and the
        # real driver (sensor QoS / best-effort); a RELIABLE subscription gets
        # nothing at all from the latter.
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.arm_pub = self.create_publisher(JointTrajectory, str(g("arm_traj_topic")), 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, str(g("gripper_traj_topic")), 10)
        self.marker_pub = self.create_publisher(Marker, "/visualization_marker", 10)

        self.create_timer(1.0 / self.rate, self._tick, callback_group=cb)

        self.get_logger().info("--- Quest IK Bridge ---")
        self.get_logger().info(
            f"hand=/q2r_{self.side}_hand_*  group={self.group_name}  "
            f"ee={self.ee_link}  base={self.base_frame_id}")
        self.get_logger().info(
            f"scale={self.position_scale}  max_step={self.max_step} m  rate={self.rate} Hz")
        self.get_logger().info("HOLD THE TRIGGER to move the arm; release to stop and re-anchor.")

        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                "/compute_ik not available - is move_group running? "
                "(ros2 launch rebotarm_moveit_config demo.launch.py)")

        if bool(g("goto_ready_on_start")):
            self._goto_ready(list(g("ready_positions")), float(g("ready_duration")))

    def _goto_ready(self, positions, duration):
        """Move to a pose strictly inside the joint limits, so IK can solve.

        Published from a one-shot timer because the publisher needs a moment to
        match the controller's subscription; publishing immediately in __init__
        would silently drop the message.
        """
        def _send():
            self._ready_timer.cancel()
            traj = JointTrajectory()
            traj.joint_names = self.arm_joints
            pt = JointTrajectoryPoint()
            pt.positions = [float(p) for p in positions]
            pt.time_from_start = Duration(sec=int(duration),
                                          nanosec=int((duration % 1.0) * 1e9))
            traj.points = [pt]
            self.arm_pub.publish(traj)
            self.get_logger().info(
                f"Moving to ready pose {pt.positions} over {duration:.1f}s "
                "(all-zeros sits on the joint2/joint3 limit, where IK always fails)")

        self._ready_timer = self.create_timer(1.0, _send)

    # ---- callbacks -----------------------------------------------------

    def _on_joint_state(self, msg):
        with self._lock:
            self._joint_state = msg

    def _on_pose(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        pos = np.array([p.x, p.y, p.z])
        ori = np.array([o.x, o.y, o.z, o.w])

        with self._lock:
            self._pos_hist.append(pos)
            self._ori_hist.append(ori)
            if len(self._pos_hist) > self.window:
                self._pos_hist.pop(0)
                self._ori_hist.pop(0)
            if len(self._pos_hist) < self.window:
                return

            self._quest_pos = np.mean(self._pos_hist, axis=0)

            # Sign-align the window before averaging: q and -q are the same
            # rotation, so a naive mean across a sign flip collapses to ~0.
            ref = self._ori_hist[0]
            aligned = [q if float(np.dot(q, ref)) >= 0.0 else -q for q in self._ori_hist]
            avg = np.mean(aligned, axis=0)
            n = float(np.linalg.norm(avg))
            self._quest_ori = (avg / n) if n > 1e-9 else None

    def _on_inputs(self, msg):
        engage = msg.press_index >= self.deadman_threshold
        with self._lock:
            if engage and not self._engaged:
                self._engaged = True
                self._anchor_hand_pos = None      # re-anchor on next tick
                self.get_logger().info("Trigger held - ENGAGED (re-anchoring)")
            elif not engage and self._engaged:
                self._engaged = False
                self._anchor_hand_pos = None
                self.get_logger().info("Trigger released - HOLDING")

            if msg.button_upper and not self._upper_prev:
                self._toggle_gripper()
            self._upper_prev = msg.button_upper

    # ---- helpers -------------------------------------------------------

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame_id, self.ee_link, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF {self.base_frame_id}->{self.ee_link} unavailable: {e}",
                                   throttle_duration_sec=2.0)
            return None, None
        t, r = tf.transform.translation, tf.transform.rotation
        return (np.array([t.x, t.y, t.z]),
                np.array([r.x, r.y, r.z, r.w]))

    def _toggle_gripper(self):
        target = self.gripper_open if self._gripper_closed_state else self.gripper_closed
        self._gripper_closed_state = not self._gripper_closed_state
        traj = JointTrajectory()
        traj.joint_names = self.gripper_joints
        pt = JointTrajectoryPoint()
        pt.positions = [target] * len(self.gripper_joints)
        pt.time_from_start = Duration(sec=0, nanosec=int(0.4 * 1e9))
        traj.points = [pt]
        self.gripper_pub.publish(traj)
        state = "CLOSED" if self._gripper_closed_state else "OPEN"
        self.get_logger().info(f"Gripper -> {state} ({target:.4f})")

    def _publish_marker(self, pos, quat):
        m = Marker()
        m.header.frame_id = self.base_frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "quest_ik_target"
        m.id = 1
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        m.pose.orientation.x = float(quat[0])
        m.pose.orientation.y = float(quat[1])
        m.pose.orientation.z = float(quat[2])
        m.pose.orientation.w = float(quat[3])
        m.scale.x = 0.01
        L = 0.08
        m.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=L, y=0.0, z=0.0),
                    Point(x=0.0, y=0.0, z=0.0), Point(x=0.0, y=L, z=0.0),
                    Point(x=0.0, y=0.0, z=0.0), Point(x=0.0, y=0.0, z=L)]
        m.colors = [ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
                    ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0), ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
                    ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0), ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)]
        self.marker_pub.publish(m)

    # ---- main loop -----------------------------------------------------

    def _tick(self):
        with self._lock:
            if not self._engaged or self._quest_pos is None or self._quest_ori is None:
                return
            if self._ik_inflight or self._joint_state is None:
                return

            hand_pos = self._quest_pos.copy()
            hand_ori = self._quest_ori.copy()

            # (Re-)anchor on the first tick after engaging.
            if self._anchor_hand_pos is None:
                rp, ro = self._robot_pose()
                if rp is None:
                    return
                self._anchor_hand_pos = hand_pos
                self._anchor_hand_ori = hand_ori
                self._anchor_robot_pos = rp
                self._anchor_robot_ori = ro
                self._last_cmd_pos = rp.copy()
                self.get_logger().info(
                    f"Anchored: robot={np.round(rp, 3)} hand={np.round(hand_pos, 3)}")
                return

            # Scaled hand delta since the anchor.
            delta = (hand_pos - self._anchor_hand_pos) * self.position_scale
            target_pos = self._anchor_robot_pos + delta

            # Clamp per-command motion so a fast hand cannot cause a big jump.
            step = target_pos - self._last_cmd_pos
            step_norm = float(np.linalg.norm(step))
            if step_norm > self.max_step:
                target_pos = self._last_cmd_pos + step * (self.max_step / step_norm)

            if self.track_orientation:
                q_rel = tf_transformations.quaternion_multiply(
                    hand_ori, tf_transformations.quaternion_inverse(self._anchor_hand_ori))
                target_ori = tf_transformations.quaternion_multiply(
                    q_rel, self._anchor_robot_ori)
                target_ori = target_ori / np.linalg.norm(target_ori)
            else:
                target_ori = self._anchor_robot_ori

            seed = self._joint_state
            self._ik_inflight = True

        self._publish_marker(target_pos, target_ori)

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ee_link
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout = Duration(sec=0, nanosec=int(0.02 * 1e9))
        req.ik_request.robot_state.joint_state = seed

        ps = PoseStamped()
        ps.header.frame_id = self.base_frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position = Point(x=float(target_pos[0]), y=float(target_pos[1]),
                                 z=float(target_pos[2]))
        ps.pose.orientation.x = float(target_ori[0])
        ps.pose.orientation.y = float(target_ori[1])
        ps.pose.orientation.z = float(target_ori[2])
        ps.pose.orientation.w = float(target_ori[3])
        req.ik_request.pose_stamped = ps

        future = self.ik_client.call_async(req)
        future.add_done_callback(lambda f, tp=target_pos: self._on_ik_done(f, tp))

    def _on_ik_done(self, future, target_pos):
        try:
            res = future.result()
        except Exception as e:
            with self._lock:
                self._ik_inflight = False
            self.get_logger().warn(f"IK call failed: {e}", throttle_duration_sec=2.0)
            return

        with self._lock:
            self._ik_inflight = False

            if res is None or res.error_code.val != 1:  # 1 == SUCCESS
                code = "none" if res is None else res.error_code.val
                self._ik_fail_streak += 1
                self.get_logger().warn(
                    f"IK rejected target (error_code={code}) - unreachable or in collision",
                    throttle_duration_sec=1.0)
                return

            self._ik_fail_streak = 0
            name_to_pos = dict(zip(res.solution.joint_state.name,
                                   res.solution.joint_state.position))
            try:
                positions = [float(name_to_pos[j]) for j in self.arm_joints]
            except KeyError as e:
                self.get_logger().error(f"IK solution missing joint {e}")
                return

            self._last_cmd_pos = target_pos.copy()

        traj = JointTrajectory()
        traj.joint_names = self.arm_joints
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(
            sec=int(self.time_from_start),
            nanosec=int((self.time_from_start % 1.0) * 1e9))
        traj.points = [pt]
        self.arm_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = QuestIKBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
