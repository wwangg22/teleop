from __future__ import annotations

import time

from control_msgs.action import FollowJointTrajectory, GripperCommand
import numpy as np
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rebotarm_msgs.action import MoveToPose

from .conversions import pose_to_xyz_rpy


class ArmActions:
    def __init__(self, node, hardware, namespace: str) -> None:
        self._node = node
        self._hardware = hardware
        self._namespace = namespace
        self._move_to_pose_server = ActionServer(
            node,
            MoveToPose,
            f"/{namespace}/move_to_pose",
            execute_callback=self.execute_move_to_pose,
            goal_callback=self.arm_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=node.reentrant_group,
        )
        self._follow_joint_trajectory_server = ActionServer(
            node,
            FollowJointTrajectory,
            f"/{namespace}/follow_joint_trajectory",
            execute_callback=self.execute_follow_joint_trajectory,
            goal_callback=self.arm_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=node.reentrant_group,
        )
        self._gripper_command_server = ActionServer(
            node,
            GripperCommand,
            f"/{namespace}/gripper/command",
            execute_callback=self.execute_gripper_command,
            goal_callback=self.gripper_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=node.reentrant_group,
        )

    def arm_goal_callback(self, _goal_request):
        return self._gate_goal(
            ("TRAJ_RUNNING", "GRAVITY_COMP", "SAFE_HOMING"), "arm motion"
        )

    def gripper_goal_callback(self, _goal_request):
        return self._gate_goal(("GRAVITY_COMP", "SAFE_HOMING"), "gripper")

    def _gate_goal(self, blocked, label):
        state = self._hardware.state_machine
        if state in blocked:
            self._node.get_logger().warn(f"rejecting {label} goal in state {state}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _fail_move_to_pose(self, goal_handle, result, message, *, canceled=False):
        if self._hardware.state_machine != "SAFE_HOMING":
            self._hardware.set_state_machine("IDLE")
            self._node.publish_arm_status()
        if canceled:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        result.success = False
        result.message = message
        result.final_pose = self._hardware.current_pose()
        return result

    def execute_move_to_pose(self, goal_handle):
        goal = goal_handle.request
        result = MoveToPose.Result()

        try:
            x, y, z, roll, pitch, yaw = pose_to_xyz_rpy(goal.target_pose)
            ok = self._hardware.move_to_pose_traj(
                x, y, z, roll, pitch, yaw, float(goal.duration)
            )
        except Exception as exc:
            self._hardware.hold_current_position()
            return self._fail_move_to_pose(goal_handle, result, str(exc))

        if not ok:
            return self._fail_move_to_pose(
                goal_handle, result, "trajectory planning failed"
            )
        self._node.publish_arm_status()

        deadline = time.monotonic() + max(float(goal.duration), 0.0) + 2.0
        while self._hardware.motion_active():
            if self._hardware.state_machine == "SAFE_HOMING":
                self._hardware.stop_motion()
                break
            if goal_handle.is_cancel_requested:
                self._hardware.stop_motion()
                self._hardware.hold_current_position()
                return self._fail_move_to_pose(
                    goal_handle, result, "move_to_pose canceled", canceled=True
                )
            if time.monotonic() > deadline:
                self._hardware.stop_motion()
                self._hardware.hold_current_position()
                return self._fail_move_to_pose(
                    goal_handle, result, "move_to_pose timeout"
                )
            time.sleep(0.02)

        if self._hardware.state_machine == "SAFE_HOMING":
            return self._fail_move_to_pose(
                goal_handle, result, "move_to_pose preempted by safe_home"
            )

        positions = self._hardware.get_joint_positions()
        velocities = self._hardware.get_joint_velocities()
        result.success = True
        result.message = (
            "move_to_traj accepted "
            f"positions={[float(v) for v in positions]} "
            f"velocities={[float(v) for v in velocities]}"
        )
        result.final_pose = self._hardware.current_pose()
        self._hardware.set_state_machine("IDLE")
        self._node.publish_arm_status()
        goal_handle.succeed()
        return result

    def execute_follow_joint_trajectory(self, goal_handle):
        goal = goal_handle.request
        result = FollowJointTrajectory.Result()
        trajectory = goal.trajectory
        joint_names = list(trajectory.joint_names)

        if not joint_names or not trajectory.points:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory must include joint_names and points"
            return result

        if joint_names != self._hardware.joint_names:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = (
                f"trajectory joint_names must be {self._hardware.joint_names}"
            )
            return result

        targets = [np.array(point.positions, dtype=np.float64) for point in trajectory.points]
        if any(len(target) != len(self._hardware.joint_names) for target in targets):
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "point positions length must match joint_names"
            return result

        # MoveIt's time parameterization already computes a velocity for every
        # waypoint. The original driver discarded it and interpolated positions
        # linearly, which makes commanded velocity piecewise-CONSTANT and step
        # discontinuously at each waypoint -- felt as rocking/lurching.
        # Keep the velocities so segments can be interpolated with a cubic
        # Hermite (continuous position AND velocity across waypoints).
        n_joints = len(self._hardware.joint_names)
        target_velocities = []
        for point in trajectory.points:
            if len(point.velocities) == n_joints:
                target_velocities.append(np.array(point.velocities, dtype=np.float64))
            else:
                # Planner supplied no velocities; fall back to zeros and let the
                # finite-difference path below approximate them.
                target_velocities.append(None)

        try:
            self._hardware.begin_trajectory_stream()
            self._node.publish_arm_status()
            start = time.monotonic()
            last_feedback_t = -1.0
            point_times = [
                float(point.time_from_start.sec)
                + float(point.time_from_start.nanosec) * 1e-9
                for point in trajectory.points
            ]
            if point_times[0] > 0.0:
                targets.insert(0, self._hardware.get_joint_positions().copy())
                point_times.insert(0, 0.0)
                # Synthetic start point: the arm is holding, so velocity is zero.
                target_velocities.insert(0, np.zeros(n_joints, dtype=np.float64))

            for index in range(1, len(targets)):
                q0 = targets[index - 1]
                q1 = targets[index]
                t0 = point_times[index - 1]
                t1 = max(point_times[index], t0)
                segment_dt = t1 - t0
                # Boundary velocities for the cubic Hermite. Prefer MoveIt's own
                # values; fall back to a finite difference if the planner gave none.
                if segment_dt > 1e-6:
                    finite_difference = (q1 - q0) / segment_dt
                else:
                    finite_difference = np.zeros_like(q1)
                v0 = target_velocities[index - 1]
                v1 = target_velocities[index]
                if v0 is None:
                    v0 = finite_difference
                if v1 is None:
                    v1 = finite_difference

                while True:
                    if self._hardware.state_machine == "SAFE_HOMING":
                        goal_handle.abort()
                        result.error_code = (
                            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        )
                        result.error_string = (
                            "follow_joint_trajectory preempted by safe_home"
                        )
                        return result
                    now = time.monotonic() - start
                    ratio = 1.0 if t1 <= t0 else max(0.0, min(1.0, (now - t0) / (t1 - t0)))

                    # --- Cubic Hermite on [t0, t1] -------------------------------
                    # s in [0,1]; T scales the tangents from per-second to per-s.
                    # p(s) = h00*q0 + h10*T*v0 + h01*q1 + h11*T*v1
                    # Position is C1-continuous and velocity matches MoveIt's
                    # profile exactly at every waypoint -- no step discontinuity.
                    if segment_dt > 1e-6:
                        s = ratio
                        s2 = s * s
                        s3 = s2 * s
                        h00 = 2.0 * s3 - 3.0 * s2 + 1.0
                        h10 = s3 - 2.0 * s2 + s
                        h01 = -2.0 * s3 + 3.0 * s2
                        h11 = s3 - s2
                        target = (
                            h00 * q0
                            + h10 * segment_dt * v0
                            + h01 * q1
                            + h11 * segment_dt * v1
                        )
                        # Analytic derivative -> commanded velocity feedforward.
                        desired_velocities = (
                            (6.0 * s2 - 6.0 * s) * (q0 - q1) / segment_dt
                            + (3.0 * s2 - 4.0 * s + 1.0) * v0
                            + (3.0 * s2 - 2.0 * s) * v1
                        )
                    else:
                        target = q1
                        desired_velocities = np.zeros_like(q1)

                    self._hardware.set_joint_position_velocity_target(
                        target, desired_velocities
                    )

                    # Feedback is THROTTLED and deliberately decoupled from the
                    # setpoint rate. get_joint_positions() costs one CAN param
                    # read (0x7019) PER JOINT on RobStride -- 6 round trips.
                    # Doing that every 5 ms saturates the bus alongside the
                    # 500 Hz control loop and makes each iteration overrun its
                    # budget, so trajectories finish late and MoveIt aborts them
                    # with "controller is taking too long".
                    # 20 Hz is plenty for feedback; setpoints stay at 200 Hz.
                    if now - last_feedback_t >= 0.05:
                        last_feedback_t = now
                        positions = self._hardware.get_joint_positions()
                        velocities = self._hardware.get_joint_velocities()
                        feedback = FollowJointTrajectory.Feedback()
                        feedback.header.stamp = self._node.get_clock().now().to_msg()
                        feedback.joint_names = self._hardware.joint_names
                        feedback.desired.positions = [float(v) for v in target]
                        feedback.desired.velocities = [
                            float(v) for v in desired_velocities
                        ]
                        feedback.actual.positions = [float(v) for v in positions]
                        feedback.actual.velocities = [float(v) for v in velocities]
                        feedback.error.positions = [
                            float(v) for v in target - positions
                        ]
                        feedback.error.velocities = [
                            float(v) for v in desired_velocities - velocities
                        ]
                        goal_handle.publish_feedback(feedback)

                    if goal_handle.is_cancel_requested:
                        self._hardware.hold_current_position()
                        goal_handle.canceled()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = "follow_joint_trajectory canceled"
                        return result

                    if now >= t1:
                        break
                    # 100 Hz setpoint updates (was 0.005 = 200 Hz, originally
                    # 0.02 = 50 Hz). 50 Hz held each target for 10 control cycles
                    # and stepped visibly, so this was raised to 200 Hz -- but
                    # every update takes _cmd_lock, and the 500 Hz MIT loop skips
                    # any cycle where it cannot get that lock promptly (see
                    # HardwareManager._endpos_loop_cb). On a CPU-bound host 200 Hz
                    # of lock traffic costs more in dropped control cycles than it
                    # buys in setpoint freshness.
                    #
                    # 100 Hz is the compromise: 5 control cycles per setpoint, and
                    # because the segment is interpolated with a cubic Hermite the
                    # intermediate values lie on a smooth curve either way -- the
                    # arm cannot feel the difference between sampling that curve at
                    # 100 Hz and at 200 Hz, but it can feel a missing torque
                    # command. Raise it back only on a machine with CPU headroom.
                    time.sleep(0.01)

        except Exception as exc:
            self._hardware.hold_current_position()
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = f"execution failed: {exc}"
            return result
        finally:
            if self._hardware.state_machine != "SAFE_HOMING":
                self._hardware.set_state_machine("IDLE")
                self._node.publish_arm_status()

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        positions = self._hardware.get_joint_positions()
        velocities = self._hardware.get_joint_velocities()
        result.error_string = (
            "joint target accepted "
            f"positions={[float(v) for v in positions]} "
            f"velocities={[float(v) for v in velocities]}"
        )
        return result

    def execute_gripper_command(self, goal_handle):
        goal = goal_handle.request.command
        result = GripperCommand.Result()
        feedback = GripperCommand.Feedback()

        try:
            self._hardware.set_gripper_target(goal.position)
        except Exception:
            goal_handle.abort()
            result.position = 0.0
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = False
            return result

        start = time.monotonic()
        last_pos = self._hardware.get_gripper_state()[0]
        stalled = False
        while time.monotonic() - start < 5.0:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                pos, _, effort, _ = self._hardware.get_gripper_state()
                result.position = pos
                result.effort = effort
                result.stalled = stalled
                result.reached_goal = False
                return result

            pos, _, effort, _ = self._hardware.get_gripper_state()
            reached = self._hardware.gripper_reached_target()
            stalled = abs(pos - last_pos) < 1e-4 and abs(effort) >= float(goal.max_effort)
            feedback.position = pos
            feedback.effort = effort
            feedback.stalled = stalled
            feedback.reached_goal = reached
            goal_handle.publish_feedback(feedback)
            if reached:
                break
            last_pos = pos
            time.sleep(0.05)

        pos, _, effort, _ = self._hardware.get_gripper_state()
        result.position = pos
        result.effort = effort
        result.stalled = stalled
        result.reached_goal = self._hardware.gripper_reached_target()
        goal_handle.succeed()
        return result
