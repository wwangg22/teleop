"""PolicyBridge -- the policy-agnostic rollout driver (plan §3.2).

Everything policy-SPECIFIC comes from the manifest (policy_manifest.py);
everything model-specific lives across the process boundary (policy_proto).
This module owns:

  ownership   teleop.stop() -> streamer.activate("policy") -- the same
              exclusive-source dance the runtime does, in reverse.
  start ramp  measured pose -> manifest start pose, the _move_to_ready
              pattern (slow explicit segment; respects the 0.35 rad
              first-command gate, verifies arrival).
  control     a control_hz loop consuming the current action chunk, one
              action per tick, submitted as the proven 3-point hold-tail
              segment with measured-dt velocity feedforward (the teleop's
              _publish_segment shape -- an extrapolating tail ratchets).
  replanning  a chunk covers `execute` ticks; the NEXT chunk is requested
              `replan_early` ticks before exhaustion on a dedicated
              inference thread, so the control loop never blocks on the
              socket (and torch never shares this process's GIL).
  gripper     event mode: on a threshold crossing the POLICY CLOCK FREEZES
              (no chunk consumption; the streamer holds position natively),
              the existing force-capped grasp primitive runs on a worker
              thread (stall == successful grasp, toggle on any completed
              call -- teleop._gripper_worker semantics), then the stale
              chunk is DISCARDED and a fresh one requested with last_action
              reflecting the new gripper state. Rationale: the real carrot
              close takes 1-6 s vs ~0.4 s in the demos; the scene barely
              changes during a grasp, so frozen observations are benign
              (plan §4-R2, user-confirmed design).
  safety      bridge-side per-tick velocity clamp at the manifest ceiling
              (<= the streamer's own 2.0), fail-closed stop on inference
              errors/stale observations, optional thermal watcher
              (policy_thermal), and stop() = hold + deactivate -- after
              which the streamer holds position forever by design.
  recording   /policy/obs_proprio, /policy/action, /policy/chunk_meta on
              the shared recorder (topics registered in
              recorder.STRUCTURED_TOPICS), plus the manifest embedded as
              an MCAP attachment into episodes started while attached (the
              attachments_provider is WRAPPED, not modified).

No file in the existing stack is imported-and-changed here; the bridge is
built ON the public surfaces of stream/state/cameras/hardware/recorder.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from collections import deque

import numpy as np

from .policy_manifest import (PolicyManifest, manifest_provenance,
                              validate_against_rig)
from .policy_map import (build_proprio, decode_action, gripper_motor_to_finger,
                         preprocess_frame, rig_to_sim)
from .policy_proto import PolicyClient

log = logging.getLogger("policy.bridge")

STREAM_SOURCE = "policy"

# /policy/* topics (contract C3). Registered in EpisodeRecorder.STRUCTURED_
# TOPICS -- the one existing-file insertion this feature makes (recorder.py).
TOPIC_OBS = "/policy/obs_proprio"
TOPIC_ACTION = "/policy/action"
TOPIC_CHUNK = "/policy/chunk_meta"


class PolicyBridge:
    def __init__(
        self,
        manifest: PolicyManifest,
        *,
        streamer,
        state_cache,
        hardware,
        camera_rig,
        kinematics,
        recorder=None,
        teleop_session=None,
        socket_path: str = "/tmp/rebot_policy.sock",
    ) -> None:
        self.m = manifest
        self.streamer = streamer
        self.states = state_cache
        self.hw = hardware
        self.rig = camera_rig
        self.kin = kinematics
        self.recorder = recorder
        self.teleop = teleop_session
        self.client = PolicyClient(socket_path)

        validate_against_rig(
            manifest,
            camera_names=list(camera_rig.cameras) if camera_rig else [],
            q_min=kinematics.lower,
            q_max=kinematics.upper,
        )
        self._provenance = manifest_provenance(manifest)

        # control-loop state (owned by the control thread)
        self._chunk: np.ndarray | None = None       # (chunk, action_dim)
        self._idx = 0
        self._q_prev_rig: np.ndarray | None = None  # last SUBMITTED target
        self._q_sim_prev: np.ndarray | None = None  # last decoded sim target
        self._last_action = self._initial_last_action()
        self._last_seg_time: float | None = None
        self._chunks_used = 0

        # inference thread plumbing
        self._req_q: queue.Queue = queue.Queue(maxsize=1)
        self._result: tuple[np.ndarray, dict] | None = None
        self._result_ready = threading.Event()
        self._infer_thread: threading.Thread | None = None

        # gripper event state (teleop conventions: busy flag + worker thread)
        self._gripper_closed = False
        self._gripper_busy = False
        self._gripper_done = threading.Event()

        # velocity finite-difference ring: (t_mono, q_rig(6,), motor_pos)
        self._fd_ring: deque = deque(maxlen=8)

        # low-pass state for target_lowpass_hz (None until first filtered tick)
        self._q_filt: np.ndarray | None = None

        self._halt = threading.Event()
        self._thread: threading.Thread | None = None
        self._orig_attachments = None
        self.thermal = None
        self.stopped_reason: str | None = None
        self.counters = {
            "ticks": 0, "submits": 0, "chunks": 0, "replan_stalls": 0,
            "vel_clamps": 0, "gripper_events": 0, "freeze_ticks": 0,
        }

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------

    def attach(self, ramp_speed: float = 0.25, min_ramp_s: float = 3.0) -> None:
        """Take the stream, ramp to the start pose, connect inference.
        Motors must already be on (runtime start_motors / ready ramp done).
        Safe to call again after stop() (multi-episode re-arm).
        min_ramp_s < 3 is for offline tests only -- hardware keeps the floor."""
        self.client.close()
        info = self.client.connect()
        log.info("inference server: %s", info.get("description", "?"))

        if self.teleop is not None:
            self.teleop.stop()          # holds, then deactivates "teleop"
        if not self.streamer.activate(STREAM_SOURCE):
            raise RuntimeError("could not acquire the setpoint stream")

        # Demos start with the gripper OPEN, but the previous session's
        # safe_home parks it CLOSED (grasp-aware close) — open it before the
        # episode or the policy's initial state is wrong and its first CLOSE
        # event is a visual no-op (seen on hardware, G1 2026-08-12).
        if self.hw is not None and not self._gripper_closed:
            self.hw.set_gripper_position(
                float(self.hw.gripper_open_position), timeout=4.0)

        st = self._fresh_state(timeout=3.0)
        q0 = st.q.copy()
        q1 = self.m.start_pose_rig()
        T = max(min_ramp_s, float(np.max(np.abs(q1 - q0))) / max(ramp_speed, 0.05))
        log.info("ramp to policy start pose over %.1f s: %s -> %s",
                 T, np.round(q0, 2).tolist(), np.round(q1, 2).tolist())
        zeros = np.zeros(6)
        self.streamer.submit(
            STREAM_SOURCE,
            [(0.0, q0, zeros), (T, q1, zeros), (T + 0.5, q1, zeros)],
        )
        time.sleep(T + 0.7)
        st = self._fresh_state(timeout=1.0)
        err = float(np.max(np.abs(st.q - q1)))
        if err > 0.15:
            self.streamer.deactivate(STREAM_SOURCE)
            raise RuntimeError(
                f"start pose not reached (max err {err:.3f} rad)")
        time.sleep(self.m.settle_s)

        self._q_prev_rig = q1.copy()
        self._q_sim_prev = rig_to_sim(self.m, q1)
        self._last_action = self._initial_last_action()

        if self.recorder is not None and self._orig_attachments is None:
            self._orig_attachments = self.recorder.attachments_provider
            self.recorder.attachments_provider = self._attachments_with_manifest

        if self.m.thermal_watch and self.thermal is None:
            from .policy_thermal import PolicyThermalWatcher

            self.thermal = PolicyThermalWatcher(
                self.states, self.hw,
                on_trip=lambda reason: self.stop(f"thermal: {reason}"),
            )
            self.thermal.start()
        log.info("policy bridge attached (%s)", self.m.name)

    def detach(self) -> None:
        """Undo attach(): stop loops, release the stream, restore hooks.
        Does NOT restart teleop -- that is the runtime's decision."""
        self.stop("detach")
        if self.thermal is not None:
            self.thermal.stop()
            self.thermal = None
        if self.recorder is not None and self._orig_attachments is not None:
            self.recorder.attachments_provider = self._orig_attachments
            self._orig_attachments = None
        self.client.close()

    def start_loop(self) -> None:
        self._halt.clear()
        self.stopped_reason = None
        self._infer_thread = threading.Thread(
            target=self._infer_loop, name="policy-infer", daemon=True)
        self._infer_thread.start()
        self._thread = threading.Thread(
            target=self._loop, name=f"policy-{int(self.m.control_hz)}hz",
            daemon=True)
        self._thread.start()

    def stop(self, reason: str = "stop") -> None:
        """Software policy stop: halt the loop, HOLD, release the stream.
        The streamer then holds the last target at zero velocity forever
        (stale->hold-never-coast); the power switch remains the true e-stop."""
        if self._halt.is_set():
            return
        if self.stopped_reason is None:   # keep the FIRST cause (a failure
            self.stopped_reason = reason  # must not be masked by episode_end)
        self._halt.set()
        for t in (self._thread, self._infer_thread):
            if t is not None and t is not threading.current_thread():
                t.join(timeout=2.0)
        self._thread = self._infer_thread = None
        if self._q_prev_rig is not None:
            self.streamer.hold(STREAM_SOURCE, self._q_prev_rig)
        self.streamer.deactivate(STREAM_SOURCE)
        log.info("policy stopped (%s)", reason)

    # ------------------------------------------------------------------
    # control loop
    # ------------------------------------------------------------------

    def _loop(self) -> None:
        period = self.m.tick_dt
        try:
            self._swap_in_chunk(first=True)
        except Exception as exc:
            log.error("first chunk failed: %s", exc)
            self._halt.set()
            self.stopped_reason = f"inference: {exc}"
            return
        # Absolute-deadline scheduling, unlike the stack's sleep-the-remainder
        # convention: this loop's wakeups land ~5 ms late every tick (GIL
        # pressure from the in-process JPEG encoders), and remainder-sleeping
        # turns that constant overshoot into a 40 Hz loop that time-dilates
        # the policy 1.26x (measured on the first rollout). Deadlines absorb
        # the overshoot; if we fall more than one period behind, re-anchor
        # instead of bursting to catch up.
        deadline = time.monotonic() + period
        while not self._halt.is_set():
            try:
                self._tick()
            except Exception as exc:
                log.error("policy tick failed -- stopping: %s", exc)
                self.stopped_reason = f"tick: {exc}"
                self._halt.set()
                break
            now = time.monotonic()
            if deadline - now > 0:
                time.sleep(deadline - now)
                deadline += period
            elif now - deadline > period:
                self.counters["overruns"] = self.counters.get("overruns", 0) + 1
                deadline = now + period
            else:
                deadline += period
        # loop exited inside the thread (error/thermal): leave the arm held
        if self.stopped_reason not in (None, "stop", "detach"):
            if self._q_prev_rig is not None:
                self.streamer.hold(STREAM_SOURCE, self._q_prev_rig)

    def _tick(self) -> None:
        self.counters["ticks"] += 1
        self._sample_fd()

        if self._gripper_busy:
            # FROZEN: no consumption, no submits; the streamer holds.
            self.counters["freeze_ticks"] += 1
            if self._gripper_done.is_set():
                self._finish_gripper_event()
            return

        if self._idx >= self.m.execute:
            self._swap_in_chunk()

        a = self._chunk[self._idx]
        q_rig, q_sim, grip = decode_action(self.m, a, self._q_sim_prev)

        if self._maybe_gripper_event(a, grip):
            return                       # froze this tick; arm holds

        # one-pole low-pass on the decoded target (manifest opt-in): raw
        # policy actions carry high-frequency noise that sim PD physics
        # filtered implicitly but the MIT loop executes faithfully (first
        # rollout: 37% per-step direction flips on joint 4 -> visible buzz)
        if self.m.target_lowpass_hz > 0.0:
            alpha = self.m.tick_dt / (
                self.m.tick_dt + 1.0 / (2.0 * np.pi * self.m.target_lowpass_hz))
            if self._q_filt is None:
                self._q_filt = self._q_prev_rig.copy()
            self._q_filt = self._q_filt + alpha * (q_rig - self._q_filt)
            q_rig = self._q_filt.copy()

        # bridge-side velocity clamp (the manifest ceiling; the streamer's
        # 2.0 rad/s clamp stays as the independent second line)
        now = time.monotonic()
        dt_nom = self.m.tick_dt
        dt = dt_nom if self._last_seg_time is None else min(
            max(now - self._last_seg_time, dt_nom), 0.25)
        step = q_rig - self._q_prev_rig
        max_step = self.m.max_joint_velocity * dt
        if np.any(np.abs(step) > max_step):
            self.counters["vel_clamps"] += 1
            step = np.clip(step, -max_step, +max_step)
            q_rig = self._q_prev_rig + step

        vff = step / dt
        tail = max(0.015, 0.75 * dt)
        self.streamer.submit(
            STREAM_SOURCE,
            [
                (0.0, self._q_prev_rig.copy(), vff),
                (dt, q_rig.copy(), vff),
                (dt + tail, q_rig.copy(), np.zeros(6)),
            ],
            t0=now,
        )
        self._last_seg_time = now
        self.counters["submits"] += 1
        self._record(TOPIC_ACTION, {
            "action": [float(v) for v in a],
            "q_rig_target": [float(v) for v in q_rig],
            "chunk_idx": self._idx,
        })

        # last_action is the RAW policy output (the training-time contract:
        # obs last_action == previous action, pre any rig-side clamping)
        self._last_action = np.asarray(a, np.float32).copy()
        self._q_prev_rig = q_rig
        self._q_sim_prev = q_sim
        self._idx += 1

        if self._idx == self.m.execute - self.m.replan_early:
            self._request_chunk_async()

    # ------------------------------------------------------------------
    # chunks / inference
    # ------------------------------------------------------------------

    def _swap_in_chunk(self, first: bool = False) -> None:
        if first:
            self._request_chunk_async()
        waited = 0.0
        while not self._result_ready.wait(timeout=0.5):
            if self._halt.is_set():
                raise RuntimeError("halted while waiting for a chunk")
            waited += 0.5
            self.counters["replan_stalls"] += 1
            log.warning("waiting %.1f s for the next chunk (inference slow?)",
                        waited)
            if waited >= 10.0:
                raise RuntimeError("no chunk after 10 s")
        chunk, info = self._result
        self._result = None
        self._result_ready.clear()
        if chunk.shape != (self.m.chunk, self.m.action_dim):
            raise RuntimeError(
                f"server returned chunk {chunk.shape}, manifest says "
                f"({self.m.chunk}, {self.m.action_dim})")
        self._chunk = chunk
        self._idx = 0
        self._chunks_used += 1
        self.counters["chunks"] += 1
        self._record(TOPIC_CHUNK, {
            "chunk_n": self._chunks_used,
            "latency_ms": info.get("latency_ms"),
            "server_info": {k: v for k, v in info.items()
                            if k not in ("latency_ms",)},
            **self._provenance,
        })

    def _request_chunk_async(self) -> None:
        try:
            self._req_q.put_nowait({
                "last_action": self._last_action.copy(),
                "info": {"execute": self.m.execute,
                         "policy": self.m.name},
            })
        except queue.Full:              # a request is already in flight
            pass

    def _infer_loop(self) -> None:
        while not self._halt.is_set():
            try:
                req = self._req_q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                obs = self._assemble_obs(req["last_action"])
                arrays, info = self.client.request("infer", obs, req["info"])
                self._result = (
                    np.asarray(arrays["chunk"], np.float32), info)
                self._result_ready.set()
            except Exception as exc:
                log.error("inference request failed: %s", exc)
                self.stopped_reason = f"inference: {exc}"
                self._halt.set()        # fail closed; control loop holds
                self._result_ready.set()   # unblock any waiter
                return

    # ------------------------------------------------------------------
    # observations
    # ------------------------------------------------------------------

    def _fresh_state(self, timeout: float):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            st = self.states.latest()
            if st is not None and time.monotonic() - st.t_mono < 0.6:
                return st
            time.sleep(0.02)
        raise RuntimeError("no fresh joint state (StateCache stale)")

    def _sample_fd(self) -> None:
        st = self.states.latest()
        if st is None:
            return
        if self._fd_ring and self._fd_ring[-1][0] == st.t_mono:
            return                      # same cache sample, skip
        self._fd_ring.append((st.t_mono, st.q.copy(), st.gripper_pos))

    def _fd_velocity(self) -> tuple[np.ndarray, float]:
        """(qd_rig(6,), motor_vel) from the two newest distinct samples.
        Hardware qd is NOT used: it reads +-0.1 rad/s on a stationary arm
        (docs/HANDOFF.md, unfixed)."""
        if len(self._fd_ring) < 2:
            return np.zeros(6), 0.0
        (t0, q0, g0), (t1, q1, g1) = self._fd_ring[-2], self._fd_ring[-1]
        dt = max(t1 - t0, 1e-3)
        return (q1 - q0) / dt, (g1 - g0) / dt

    def _assemble_obs(self, last_action: np.ndarray) -> dict[str, np.ndarray]:
        st = self._fresh_state(timeout=0.6)
        q_sim = rig_to_sim(self.m, st.q)
        qd_rig, motor_vel = self._fd_velocity()
        qd_sim = self.m.rig_sign * qd_rig
        finger = gripper_motor_to_finger(self.m, st.gripper_pos)
        # finger velocity through the same linear map's slope
        span_m = self.m.rig_gripper_motor_open - self.m.rig_gripper_motor_closed
        slope = 0.0 if abs(span_m) < 1e-9 else (
            (self.m.gripper_open - self.m.gripper_close) / span_m)
        proprio = build_proprio(
            self.m, q_sim, qd_sim, finger, motor_vel * slope, last_action)
        obs: dict[str, np.ndarray] = {"proprio": proprio}
        for cam_key, spec in self.m.cameras.items():
            cam = self.rig.cameras[spec.rig_camera]
            slot = cam.latest()
            if slot.color is None or time.monotonic() - slot.t_mono > 1.0:
                raise RuntimeError(f"camera {spec.rig_camera!r} has no fresh "
                                   "frame")
            obs[f"image.{cam_key}"] = preprocess_frame(
                slot.color, spec.crop, spec.resize)
        self._record(TOPIC_OBS, {
            "proprio": [float(v) for v in proprio],
            "gripper_closed": self._gripper_closed,
        })
        return obs

    # ------------------------------------------------------------------
    # gripper events (freeze-the-clock design, plan §4-R2)
    # ------------------------------------------------------------------

    def _maybe_gripper_event(self, a: np.ndarray, grip: float) -> bool:
        if self.m.gripper_mode != "event_threshold" or self.hw is None:
            return False
        want_close = (not self._gripper_closed
                      and grip < self.m.gripper_close_below)
        want_open = (self._gripper_closed
                     and grip > self.m.gripper_open_above)
        if not (want_close or want_open):
            return False
        self.counters["gripper_events"] += 1
        self._gripper_busy = True
        self._gripper_done.clear()
        # freeze bookkeeping: the action that fired the event becomes
        # last_action so the post-event replan sees the commanded gripper
        self._last_action = np.asarray(a, np.float32).copy()
        self.streamer.hold(STREAM_SOURCE, self._q_prev_rig)
        log.info("gripper event: %s (policy clock FROZEN)",
                 "close" if want_close else "open")
        threading.Thread(
            target=self._gripper_worker, args=(bool(want_close),),
            name="policy-gripper", daemon=True,
        ).start()
        return True

    def _gripper_worker(self, want_close: bool) -> None:
        """teleop._gripper_worker semantics: blocking primitives, a stall is
        a successful grasp, the state flips on ANY completed call."""
        completed = False
        try:
            if want_close:
                ok, grasped, pos = self.hw.close_gripper_grasp()
                log.info("gripper close: %s at %.3f rad",
                         "grasped object" if grasped
                         else ("complete" if ok else "timeout"), pos)
            else:
                reached, pos = self.hw.set_gripper_position(
                    float(self.hw.gripper_open_position), timeout=4.0)
                log.info("gripper open: %s at %.3f rad",
                         "reached" if reached else "timeout", pos)
            completed = True
        except Exception as exc:
            log.error("gripper command failed (state unchanged): %s", exc)
        finally:
            if completed:
                self._gripper_closed = want_close
                self._record("/robot/gripper", {"closed": want_close})
            self._gripper_done.set()

    def _finish_gripper_event(self) -> None:
        """Resume after a grasp: the frozen chunk is STALE (seconds passed);
        discard it and replan from the current observation."""
        self._gripper_busy = False
        self._gripper_done.clear()
        self._chunk = None
        self._idx = self.m.execute          # forces a swap
        # drain any stale in-flight result, then request fresh
        self._result = None
        self._result_ready.clear()
        try:
            while True:
                self._req_q.get_nowait()
        except queue.Empty:
            pass
        log.info("gripper event done -- replanning (clock resumes)")
        self._request_chunk_async()
        self._swap_in_chunk()

    # ------------------------------------------------------------------
    # recording / attachments
    # ------------------------------------------------------------------

    def _record(self, topic: str, payload: dict) -> None:
        if self.recorder is not None:
            self.recorder.write_json(topic, payload)

    def _attachments_with_manifest(self):
        out = list(self._orig_attachments() if self._orig_attachments else [])
        try:
            from pathlib import Path

            out.append((
                "policy/manifest.yaml", "application/yaml",
                Path(self.m.path).read_bytes(),
            ))
        except OSError as exc:
            log.error("manifest attachment failed: %s", exc)
        return out

    # ------------------------------------------------------------------

    def _initial_last_action(self) -> np.ndarray:
        """Training-time reset value: zeros, gripper channel at OPEN (+1) --
        verified against the recorded demos (obs41[0, 34:41] = [0..0, 1])."""
        a = np.zeros(self.m.action_dim, dtype=np.float32)
        if self.m.gripper_mode != "none" and self.m.action_dim > 6:
            a[6] = 1.0
        return a

    def snapshot(self) -> dict:
        out = {
            "policy": self.m.name,
            "stopped_reason": self.stopped_reason,
            "gripper_closed": self._gripper_closed,
            "gripper_busy": self._gripper_busy,
            **self.counters,
        }
        if self.thermal is not None:
            out["thermal"] = self.thermal.snapshot()
        return out
