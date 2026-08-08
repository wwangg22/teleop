"""Action-chunk executor: run a learned policy on the reBot arm.

=============================================================================
THE PROBLEM
=============================================================================
Learned manipulation policies (ACT, Diffusion Policy, pi0, SmolVLA, ...) do not
emit one action per control tick. They emit a CHUNK: a short sequence of future
actions predicted from a single observation. Chunks arrive slowly -- typically
5-10 Hz here -- because model inference is expensive.

The arm, meanwhile, wants a fresh setpoint every few milliseconds. Our MIT
control loop runs at 500 Hz and the trajectory executor streams setpoints at
200 Hz. Feeding it 5 Hz staircase steps would produce exactly the lurching we
spent time removing from the trajectory path.

So this module has to solve three problems:

  1. UPSAMPLING   5-10 Hz of chunk data  ->  200 Hz of smooth setpoints.
  2. BLENDING     Consecutive chunks overlap and DISAGREE about the same future
                  instant. Naively switching to each new chunk causes a
                  discontinuity at every chunk boundary.
  3. SAFETY       A policy is an opaque function that can emit nonsense. It must
                  never be able to command a step change straight into a
                  hard-stop.

=============================================================================
WHAT THE LITERATURE DOES
=============================================================================
* TEMPORAL ENSEMBLING (ACT, Zhao et al. 2023, RSS)
    Query the policy faster than the chunk length so several chunks overlap.
    For a given instant, average every chunk's prediction for that instant with
    exponential weights. This is a pure CONSUMER-SIDE method -- it needs nothing
    from the policy -- which is why it is what we implement here.

    ACT's exact scheme (from the reference implementation):
        w_i = exp(-m * i)        i = 0 is the OLDEST prediction
        w   = w / sum(w)
    Note the direction: ACT gives the OLDEST prediction the LARGEST weight.
    That biases toward commitment/smoothness over reactivity. m is small
    (0.01 in the paper) which makes the weighting nearly uniform.
    We implement this faithfully but expose `ensemble_favor` to invert it,
    because favouring the NEWEST chunk is more reactive and some
    implementations prefer it.

* RECEDING HORIZON (Diffusion Policy, Chi et al. 2023)
    Predict H actions, execute only the first k, replan. Equivalent to
    temporal ensembling with a hard weight of 1 on the newest chunk. You get
    this here by setting ensemble_m very large with ensemble_favor='newest'.

* REAL-TIME CHUNKING (RTC, Physical Intelligence, 2025)
    Handles INFERENCE LATENCY by freezing the actions that will execute while
    the model is still thinking, then treating the chunk overlap as an
    INPAINTING problem -- guiding the flow-matching denoiser so the new chunk
    continues the old one. This is strictly better than post-hoc blending, but
    it lives INSIDE the policy's sampling loop and cannot be done here.
    ---> If you later run a flow/diffusion policy, implement RTC in the policy
         node and publish already-continuous chunks; then set ensemble_favor to
         'newest' and ensemble_m high so this node stops blending and simply
         upsamples. See `RTC HOOK` below.

=============================================================================
HOW TO PLUG IN YOUR POLICY
=============================================================================
Publish `trajectory_msgs/JointTrajectory` on `/rebotarm/policy/action_chunk`:

    msg.header.stamp   = time the FIRST action should execute.
                         If you stamp it with the OBSERVATION time instead,
                         set the `chunk_time_offset` parameter to your
                         measured inference latency and this node will shift
                         the chunk forward for you.
    msg.joint_names    = any order; remapped to hardware order automatically.
    msg.points[i].positions        = absolute joint targets, radians. REQUIRED.
    msg.points[i].velocities       = optional. Supplied -> cubic Hermite
                                     interpolation. Omitted -> Catmull-Rom
                                     (finite-difference tangents).
    msg.points[i].time_from_start  = offset of action i from header.stamp.
                                     If all zero, `policy_dt` is assumed.

Then:
    ros2 service call /rebotarm/policy/start std_srvs/srv/Trigger
    ros2 service call /rebotarm/policy/stop  std_srvs/srv/Trigger

Nothing runs until you call start -- a policy cannot grab the arm just by
publishing.
"""

from __future__ import annotations

import math
import threading

import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory


class ChunkExecutor:
    def __init__(
        self,
        node,
        hardware,
        namespace: str,
        *,
        chunk_topic: str,
        control_hz: float,
        policy_dt: float,
        ensemble_m: float,
        ensemble_favor: str,
        chunk_time_offset: float,
        stale_timeout: float,
        max_joint_velocity: float,
        max_position_jump: float,
        joint_limits_lower: list[float] | None = None,
        joint_limits_upper: list[float] | None = None,
    ) -> None:
        self._node = node
        self._hardware = hardware
        self._joint_names = list(hardware.joint_names)
        self._n = len(self._joint_names)

        self._policy_dt = float(policy_dt)
        self._ensemble_m = float(ensemble_m)
        self._ensemble_favor = str(ensemble_favor).lower()
        self._chunk_time_offset = float(chunk_time_offset)
        self._stale_timeout = float(stale_timeout)
        self._max_joint_velocity = float(max_joint_velocity)
        self._max_position_jump = float(max_position_jump)

        # Joint limits: a policy must never be able to drive past these.
        # Default to +/- pi if not supplied.
        lower = joint_limits_lower or []
        upper = joint_limits_upper or []
        self._q_min = np.array(
            lower if len(lower) == self._n else [-math.pi] * self._n,
            dtype=np.float64,
        )
        self._q_max = np.array(
            upper if len(upper) == self._n else [math.pi] * self._n,
            dtype=np.float64,
        )

        # --- chunk buffer -------------------------------------------------
        # Each entry: dict(t0=float seconds, times=np.array, positions=(k,n),
        #                  velocities=(k,n) or None, seq=int)
        # Ordered oldest -> newest, which is the order ACT's weights assume.
        self._chunks: list[dict] = []
        self._chunk_seq = 0
        self._buffer_lock = threading.Lock()

        self._active = False
        self._last_target: np.ndarray | None = None
        self._last_command_time: float | None = None
        self._warned_no_chunk = False

        # --- ROS interface ------------------------------------------------
        node.create_subscription(
            JointTrajectory,
            chunk_topic,
            self._chunk_cb,
            10,
            callback_group=node.reentrant_group,
        )
        self._status_pub = node.create_publisher(
            String,
            f"/{namespace}/policy/status",
            10,
            callback_group=node.reentrant_group,
        )
        node.create_service(
            Trigger,
            f"/{namespace}/policy/start",
            self._start_srv,
            callback_group=node.slow_group,
        )
        node.create_service(
            Trigger,
            f"/{namespace}/policy/stop",
            self._stop_srv,
            callback_group=node.slow_group,
        )

        self._control_dt = 1.0 / max(float(control_hz), 1.0)
        # Mutually-exclusive group: a tick that overruns (blocked on the
        # command lock) must not be dispatched again concurrently -- two
        # concurrent ticks race on _last_target and the older sample can win
        # the write, commanding the arm backwards.
        self._timer = node.create_timer(
            self._control_dt,
            self._control_tick,
            callback_group=node.policy_tick_group,
        )

        node.get_logger().info(
            f"chunk executor ready (inactive): topic={chunk_topic} "
            f"control={control_hz:.0f}Hz policy_dt={policy_dt:.3f}s "
            f"ensemble_m={ensemble_m} favor={ensemble_favor}"
        )

    # ------------------------------------------------------------------
    # clock helper -- one source of truth so chunk stamps and control ticks
    # are always compared on the same timebase.
    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # chunk intake
    # ------------------------------------------------------------------

    def _chunk_cb(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return

        # --- remap joint order --------------------------------------------
        # The policy may publish joints in any order (or a subset). Build an
        # index map into hardware order; reject anything we cannot place.
        try:
            index_map = [list(msg.joint_names).index(name) for name in self._joint_names]
        except ValueError:
            self._node.get_logger().warn(
                "policy chunk joint_names do not cover the arm joints "
                f"{self._joint_names}; ignoring chunk",
                throttle_duration_sec=5.0,
            )
            return

        k = len(msg.points)
        positions = np.zeros((k, self._n), dtype=np.float64)
        velocities = np.zeros((k, self._n), dtype=np.float64)
        times = np.zeros(k, dtype=np.float64)
        has_velocity = True

        for i, point in enumerate(msg.points):
            if len(point.positions) < self._n:
                self._node.get_logger().warn(
                    "policy chunk point has too few positions; ignoring chunk",
                    throttle_duration_sec=5.0,
                )
                return
            positions[i] = [float(point.positions[j]) for j in index_map]
            if len(point.velocities) >= self._n:
                velocities[i] = [float(point.velocities[j]) for j in index_map]
            else:
                has_velocity = False
            times[i] = (
                float(point.time_from_start.sec)
                + float(point.time_from_start.nanosec) * 1e-9
            )

        # If the policy left time_from_start unset (all zeros), assume a fixed
        # cadence of policy_dt between actions -- the common case for policies
        # that emit a bare (k, n) action array.
        if k > 1 and np.allclose(times, 0.0):
            times = np.arange(k, dtype=np.float64) * self._policy_dt

        # --- chunk start time ---------------------------------------------
        # Zero stamp => "starts now". Otherwise trust the stamp, plus the
        # configured offset (use this to compensate inference latency when the
        # policy stamps chunks with the OBSERVATION time).
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        t0 = (self._now() if stamp <= 0.0 else stamp) + self._chunk_time_offset

        chunk = {
            "t0": t0,
            "times": times,
            "positions": positions,
            "velocities": velocities if has_velocity else None,
            "seq": self._chunk_seq,
        }
        with self._buffer_lock:
            self._chunk_seq += 1
            self._chunks.append(chunk)
            # Drop chunks whose horizon has fully elapsed; they can no longer
            # contribute to any future instant.
            now = self._now()
            self._chunks = [
                c for c in self._chunks if c["t0"] + c["times"][-1] >= now
            ]
            # Hard cap so a misbehaving publisher cannot grow memory forever.
            if len(self._chunks) > 32:
                self._chunks = self._chunks[-32:]

    # ------------------------------------------------------------------
    # interpolation within a single chunk
    # ------------------------------------------------------------------

    def _sample_chunk(self, chunk: dict, t: float):
        """Position and velocity predicted by ONE chunk at absolute time t.

        Returns (position, velocity) or None if t is outside the chunk.

        With velocities supplied we use a cubic Hermite (C1: position AND
        velocity continuous). Without, we synthesise tangents by central
        difference -- a Catmull-Rom spline -- which is the standard way to get
        a smooth curve through bare waypoints.
        """
        times = chunk["times"]
        local_t = t - chunk["t0"]
        if local_t < 0.0 or local_t > times[-1]:
            return None

        positions = chunk["positions"]
        k = len(times)
        if k == 1:
            return positions[0].copy(), np.zeros(self._n)

        # Locate the bracketing pair [i, i+1].
        i = int(np.searchsorted(times, local_t, side="right") - 1)
        i = max(0, min(i, k - 2))
        t0, t1 = times[i], times[i + 1]
        dt = t1 - t0
        if dt <= 1e-9:
            return positions[i + 1].copy(), np.zeros(self._n)

        q0, q1 = positions[i], positions[i + 1]

        if chunk["velocities"] is not None:
            v0, v1 = chunk["velocities"][i], chunk["velocities"][i + 1]
        else:
            # Catmull-Rom tangents: centred finite differences, clamped at ends.
            def tangent(idx: int) -> np.ndarray:
                lo = max(0, idx - 1)
                hi = min(k - 1, idx + 1)
                span = times[hi] - times[lo]
                if span <= 1e-9:
                    return np.zeros(self._n)
                return (positions[hi] - positions[lo]) / span

            v0, v1 = tangent(i), tangent(i + 1)

        # Cubic Hermite basis on s in [0, 1].
        s = (local_t - t0) / dt
        s2, s3 = s * s, s * s * s
        h00 = 2 * s3 - 3 * s2 + 1
        h10 = s3 - 2 * s2 + s
        h01 = -2 * s3 + 3 * s2
        h11 = s3 - s2
        position = h00 * q0 + h10 * dt * v0 + h01 * q1 + h11 * dt * v1
        # Analytic derivative -> velocity feedforward, same as the trajectory
        # executor. Without this the MIT kd term brakes against the motion.
        velocity = (
            (6 * s2 - 6 * s) * (q0 - q1) / dt
            + (3 * s2 - 4 * s + 1) * v0
            + (3 * s2 - 2 * s) * v1
        )
        return position, velocity

    # ------------------------------------------------------------------
    # temporal ensembling
    # ------------------------------------------------------------------

    def _ensemble(self, t: float):
        """Blend every live chunk's prediction for time t. None if no coverage."""
        with self._buffer_lock:
            chunks = list(self._chunks)

        samples = []
        for chunk in chunks:                      # oldest -> newest
            got = self._sample_chunk(chunk, t)
            if got is not None:
                samples.append(got)

        if not samples:
            return None
        if len(samples) == 1:
            return samples[0]

        # ACT: w_i = exp(-m * i) with i = 0 the OLDEST contributing prediction.
        # 'oldest' reproduces the paper (smoother, more committed).
        # 'newest' inverts it (more reactive to the latest observation).
        idx = np.arange(len(samples), dtype=np.float64)
        if self._ensemble_favor == "newest":
            idx = idx[::-1]
        weights = np.exp(-self._ensemble_m * idx)
        weights /= weights.sum()

        position = np.zeros(self._n)
        velocity = np.zeros(self._n)
        for w, (p, v) in zip(weights, samples):
            position += w * p
            velocity += w * v
        return position, velocity

    # ------------------------------------------------------------------
    # control loop
    # ------------------------------------------------------------------

    def _control_tick(self) -> None:
        if not self._active:
            return

        # Never fight another motion source. safe_home / trajectories /
        # gravity comp all take precedence; we stand down rather than
        # arbitrating mid-motion.
        state = self._hardware.state_machine
        if state in ("SAFE_HOMING", "TRAJ_RUNNING", "GRAVITY_COMP"):
            self._publish(f"standing down: state={state}")
            return

        now = self._now()
        result = self._ensemble(now)

        if result is None:
            # No chunk covers this instant: policy is late, stopped, or has
            # not started. HOLD the last commanded position -- never coast,
            # never fall back to something arbitrary.
            if self._last_command_time is not None:
                age = now - self._last_command_time
                if age > self._stale_timeout and not self._warned_no_chunk:
                    self._warned_no_chunk = True
                    self._node.get_logger().warn(
                        f"no policy chunk for {age:.2f}s -- holding position"
                    )
                    self._publish(f"stale {age:.2f}s holding")
            if self._last_target is not None:
                self._hardware.set_joint_position_velocity_target(
                    self._last_target, np.zeros(self._n)
                )
            return

        self._warned_no_chunk = False
        target, velocity = result

        # ---------------- safety layer ------------------------------------
        # Everything below assumes the policy may be wrong. These clamps are
        # the only thing between a bad chunk and the hardware.

        # 1. Joint limits.
        target = np.clip(target, self._q_min, self._q_max)

        # 2. Per-tick step limit. Converts any step discontinuity (bad chunk,
        #    first chunk after a gap, policy restart) into a bounded ramp.
        #    The budget uses the MEASURED time since the last applied command,
        #    not the nominal timer period: ticks routinely run late (blocked
        #    behind _cmd_lock), and budgeting nominal dt on a late tick
        #    silently collapses the velocity ceiling below what callers were
        #    promised. Capped at 4x nominal so a long stall cannot authorize
        #    one giant catch-up step.
        if self._last_target is not None:
            step = target - self._last_target
            if self._last_command_time is not None:
                elapsed = min(max(now - self._last_command_time,
                                  self._control_dt), 4.0 * self._control_dt)
            else:
                elapsed = self._control_dt
            max_step = self._max_joint_velocity * elapsed
            oversize = np.abs(step) > max_step
            if np.any(oversize):
                step = np.clip(step, -max_step, +max_step)
                target = self._last_target + step
                velocity = np.clip(
                    velocity, -self._max_joint_velocity, +self._max_joint_velocity
                )
        else:
            # First command after activation: refuse to jump from wherever the
            # arm physically is to wherever the policy wants. If the gap is
            # large the operator should home first.
            current = self._hardware.get_joint_positions(request=False)
            gap = float(np.max(np.abs(target - current)))
            if gap > self._max_position_jump:
                self._node.get_logger().error(
                    f"first policy target is {gap:.3f} rad from the current pose "
                    f"(limit {self._max_position_jump:.3f}); refusing to start. "
                    "Home the arm or move the policy's start pose closer."
                )
                self._publish(f"refused: first target {gap:.3f} rad away")
                self._active = False
                return
            self._last_target = current.copy()

        # 3. Velocity feedforward clamp.
        velocity = np.clip(
            velocity, -self._max_joint_velocity, +self._max_joint_velocity
        )

        self._hardware.set_joint_position_velocity_target(target, velocity)
        self._last_target = target
        self._last_command_time = now

    def _publish(self, text: str) -> None:
        try:
            msg = String()
            msg.data = text
            self._status_pub.publish(msg)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # start / stop
    # ------------------------------------------------------------------

    def _start_srv(self, _request, response):
        if self._active:
            response.success = True
            response.message = "policy execution already active"
            return response
        if not self._hardware.enabled:
            response.success = False
            response.message = "arm is disabled; call /rebotarm/enable first"
            return response

        with self._buffer_lock:
            self._chunks.clear()
        self._last_target = None
        self._last_command_time = None
        self._warned_no_chunk = False
        self._active = True
        self._node.get_logger().warn("POLICY EXECUTION ACTIVE -- the arm will move")
        self._publish("active")
        response.success = True
        response.message = "policy execution started"
        return response

    def _stop_srv(self, _request, response):
        self._active = False
        with self._buffer_lock:
            self._chunks.clear()
        # Hold wherever we are rather than going limp.
        try:
            self._hardware.hold_current_position()
        except Exception:
            pass
        self._node.get_logger().info("policy execution stopped; holding position")
        self._publish("stopped")
        response.success = True
        response.message = "policy execution stopped; holding position"
        return response

    @property
    def active(self) -> bool:
        return self._active
