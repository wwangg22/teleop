"""Mock policy: publishes action chunks so the executor can be tested without a
real model.

Emits exactly what a chunked policy emits -- a short sequence of absolute joint
targets, published slowly -- so you can verify smoothness, blending, latency
handling and the safety clamps before plugging in anything learned.

    # gentle sinusoid on ONE joint, 5 Hz chunks of 16 actions at 10 Hz spacing
    ros2 run rebotarmcontroller MockPolicy

    # every joint, faster chunks, bigger motion
    ros2 run rebotarmcontroller MockPolicy --ros-args \
        -p joints:="[0,1,2]" -p amplitude:=0.25 -p publish_hz:=10.0

Parameters mirror what a real policy would vary:
    publish_hz     how often chunks are emitted     (your policy's inference rate)
    chunk_size     actions per chunk                (policy's prediction horizon)
    policy_dt      seconds between actions          (policy's action timestep)
    amplitude      radians of motion
    period         seconds per sine cycle
    joints         which joint indices move
    stamp_mode     'now'   -> header.stamp = first action time
                   'zero'  -> unstamped, executor assumes "starts now"
                   'delay' -> simulate inference latency by stamping in the past,
                              to exercise chunk_time_offset
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MockPolicy(Node):
    def __init__(self) -> None:
        super().__init__("mock_policy")
        self.declare_parameter("topic", "/rebotarm/policy/action_chunk")
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("chunk_size", 16)
        self.declare_parameter("policy_dt", 0.1)
        self.declare_parameter("amplitude", 0.15)
        self.declare_parameter("period", 12.0)
        self.declare_parameter("joints", [0])
        self.declare_parameter("send_velocities", True)
        self.declare_parameter("stamp_mode", "now")
        self.declare_parameter("simulated_latency", 0.15)

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.chunk_size = int(self.get_parameter("chunk_size").value)
        self.policy_dt = float(self.get_parameter("policy_dt").value)
        self.amplitude = float(self.get_parameter("amplitude").value)
        self.period = max(float(self.get_parameter("period").value), 0.1)
        self.moving = [int(i) for i in self.get_parameter("joints").value]
        self.send_velocities = bool(self.get_parameter("send_velocities").value)
        self.stamp_mode = str(self.get_parameter("stamp_mode").value)
        self.simulated_latency = float(self.get_parameter("simulated_latency").value)

        self.pub = self.create_publisher(
            JointTrajectory, str(self.get_parameter("topic").value), 10
        )
        publish_hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / max(publish_hz, 0.1), self.publish_chunk)

        self.t_start = self.now_s()
        self.get_logger().info(
            f"mock policy: {publish_hz:.1f} Hz chunks of {self.chunk_size} actions "
            f"@ {self.policy_dt:.3f}s, amplitude {self.amplitude:.3f} rad "
            f"on joints {self.moving}"
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_chunk(self) -> None:
        now = self.now_s()

        # A real policy predicts from an observation taken slightly in the past.
        # 'delay' reproduces that so you can tune chunk_time_offset.
        if self.stamp_mode == "delay":
            first_action_time = now - self.simulated_latency
        else:
            first_action_time = now

        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        if self.stamp_mode == "zero":
            msg.header.stamp.sec = 0
            msg.header.stamp.nanosec = 0
        else:
            msg.header.stamp.sec = int(first_action_time)
            msg.header.stamp.nanosec = int((first_action_time % 1.0) * 1e9)

        omega = 2.0 * math.pi / self.period
        for i in range(self.chunk_size):
            t_abs = first_action_time + i * self.policy_dt
            phase = omega * (t_abs - self.t_start)

            positions = [0.0] * len(self.joint_names)
            velocities = [0.0] * len(self.joint_names)
            for j in self.moving:
                if 0 <= j < len(self.joint_names):
                    positions[j] = self.amplitude * math.sin(phase)
                    velocities[j] = self.amplitude * omega * math.cos(phase)

            point = JointTrajectoryPoint()
            point.positions = positions
            if self.send_velocities:
                point.velocities = velocities
            offset = i * self.policy_dt
            point.time_from_start.sec = int(offset)
            point.time_from_start.nanosec = int((offset % 1.0) * 1e9)
            msg.points.append(point)

        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockPolicy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
