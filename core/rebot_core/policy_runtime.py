"""Policy-rollout entry point -- composes the UNMODIFIED Runtime with a
PolicyBridge (or a trajectory replay). This is a separate __main__ so that
`rebot_core.runtime` (the teleop station) is never touched: the policy
feature exists only when a human explicitly launches THIS module.

    # full rollout (HARDWARE MOVES -- requires --arm):
    conda run -n teleop python -m rebot_core.policy_runtime \
        --manifest /path/to/policy.yaml --socket /tmp/rebot_policy.sock \
        --episodes 3 --episode-s 25 --arm

    # bring-up gate G1, sim-episode replay (no inference server needed):
    conda run -n teleop python -m rebot_core.policy_runtime \
        --manifest .../policy.yaml --replay .../ep_s21_0000/arrays.npz \
        --time-scale 0.25 --arm

Sequence (mirrors the GUI flow, then hands over):
  Runtime.start() (station tier: cameras, recorder, GUI) ->
  start_motors() (preflight scan, energize, READY ramp, live teleop) ->
  teleop released, stream re-owned as "policy", ramp to the manifest start
  pose -> rollout episodes (each one an MCAP episode) -> on exit the normal
  Runtime.shutdown() parks (safe home, disable).

Signals follow the runtime contract: first SIGINT stops the policy and
parks; further signals during shutdown are ignored; the power switch is
the real e-stop.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from pathlib import Path

import numpy as np

from .policy_bridge import PolicyBridge, STREAM_SOURCE
from .policy_manifest import load_manifest
from .policy_map import decode_action, sim_to_rig
from .runtime import Runtime, RuntimeConfig

log = logging.getLogger("policy.runtime")


def _wait_motors(rt: Runtime, timeout: float = 120.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if rt.motors_state == "on":
            return
        if rt.motors_state == "off" and rt.motors_error:
            raise RuntimeError(f"motor start failed: {rt.motors_error}")
        time.sleep(0.5)
    raise RuntimeError("motors did not come up in time")


def _replay(rt: Runtime, manifest, npz_path: str, time_scale: float) -> None:
    """Gate G1: drive a recorded SIM episode's decoded targets through the
    streamer. Validates the sign map, units, limits, default pose and the
    hold-tail cadence with zero learning in the loop. Slow first: 0.25x."""
    data = np.load(npz_path)
    actions = data["action"]
    dt = manifest.tick_dt / max(time_scale, 1e-3)
    log.warning("REPLAY %s: %d steps at %.0f ms/step (%.2fx)",
                npz_path, len(actions), dt * 1e3, time_scale)

    rt.teleop.stop()
    if not rt.streamer.activate(STREAM_SOURCE):
        raise RuntimeError("could not acquire the stream for replay")

    q_sim_prev = manifest.default_pose.copy()
    first_rig, _, _ = decode_action(manifest, actions[0], q_sim_prev)
    st = rt.states.latest()
    T = max(3.0, float(np.max(np.abs(first_rig - st.q))) / 0.25)
    zeros = np.zeros(6)
    rt.streamer.submit(STREAM_SOURCE, [
        (0.0, st.q.copy(), zeros), (T, first_rig, zeros),
        (T + 0.5, first_rig, zeros)])
    time.sleep(T + 0.7)

    q_prev = first_rig
    gripper_closed = False
    # previous session's safe_home parks the gripper CLOSED; demos start OPEN
    # (G1 2026-08-12: the demo's CLOSE event was a visual no-op)
    if rt.hw is not None:
        rt.hw.set_gripper_position(float(rt.hw.gripper_open_position), timeout=4.0)
    last_seg = None
    for i, a in enumerate(actions):
        t0 = time.monotonic()
        q_rig, q_sim_prev, grip = decode_action(manifest, a, q_sim_prev)
        seg_dt = dt if last_seg is None else min(max(t0 - last_seg, dt), 0.5)
        last_seg = t0
        vff = (q_rig - q_prev) / seg_dt
        rt.streamer.submit(STREAM_SOURCE, [
            (0.0, q_prev, vff), (seg_dt, q_rig, vff),
            (seg_dt + max(0.015, 0.75 * seg_dt), q_rig, zeros)], t0=t0)
        q_prev = q_rig
        if manifest.gripper_mode == "event_threshold" and rt.hw is not None:
            if not gripper_closed and grip < manifest.gripper_close_below:
                log.info("replay step %d: gripper CLOSE", i)
                rt.streamer.hold(STREAM_SOURCE, q_prev)
                rt.hw.close_gripper_grasp()
                gripper_closed = True
                last_seg = None
            elif gripper_closed and grip > manifest.gripper_open_above:
                log.info("replay step %d: gripper OPEN", i)
                rt.streamer.hold(STREAM_SOURCE, q_prev)
                rt.hw.set_gripper_position(
                    float(rt.hw.gripper_open_position), timeout=4.0)
                gripper_closed = False
                last_seg = None
        el = time.monotonic() - t0
        if el < dt:
            time.sleep(dt - el)
    rt.streamer.hold(STREAM_SOURCE, q_prev)
    log.warning("replay done: %s", rt.streamer.snapshot())
    rt.streamer.deactivate(STREAM_SOURCE)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--socket", default="/tmp/rebot_policy.sock")
    ap.add_argument("--episodes", type=int, default=1)
    ap.add_argument("--episode-s", type=float, default=30.0)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--replay", default=None,
                    help="arrays.npz of a recorded sim episode (gate G1); "
                         "no inference server involved")
    ap.add_argument("--time-scale", type=float, default=0.25)
    ap.add_argument("--arm", action="store_true",
                    help="REQUIRED: confirms the arm may energize and move")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--model", default="rs")
    ap.add_argument("--demos-root", default=str(Path.home() / "demos"))
    ap.add_argument("--gui-port", type=int, default=8800)
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )
    if not args.arm:
        print("refusing: pass --arm to confirm the arm may energize and move")
        return 2

    manifest = load_manifest(args.manifest)
    cfg = RuntimeConfig(
        model=args.model, channel=args.channel,
        demos_root=Path(args.demos_root), gui_port=args.gui_port,
    )
    rt = Runtime(cfg)

    def _on_signal(_sig, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    bridge = None
    try:
        rt.start()
        ok, msg = rt.start_motors()
        if not ok:
            raise RuntimeError(msg)
        log.warning("%s", msg)
        _wait_motors(rt)

        if args.replay:
            _replay(rt, manifest, args.replay, args.time_scale)
            return 0

        bridge = PolicyBridge(
            manifest,
            streamer=rt.streamer, state_cache=rt.states, hardware=rt.hw,
            camera_rig=rt.rig, kinematics=rt.kin, recorder=rt.recorder,
            teleop_session=rt.teleop, socket_path=args.socket,
        )
        bridge.attach()
        for ep in range(args.episodes):
            log.warning("=== policy episode %d/%d ===", ep + 1, args.episodes)
            if not args.no_record:
                rt.recorder.start(source="policy")
            bridge.start_loop()
            t_end = time.monotonic() + args.episode_s
            while time.monotonic() < t_end and bridge.stopped_reason is None:
                time.sleep(0.2)
            bridge.stop("episode_end")
            if not args.no_record:
                rt.recorder.stop(reason="policy_episode_end")
            log.warning("episode %d: %s", ep + 1, bridge.snapshot())
            if bridge.stopped_reason not in ("episode_end", None):
                log.error("aborting remaining episodes: %s",
                          bridge.stopped_reason)
                break
            if ep + 1 < args.episodes:
                bridge.attach()      # re-ramp to start pose for the next run
        return 0
    except KeyboardInterrupt:
        log.warning("interrupted -- stopping policy, then parking")
        return 0
    finally:
        if bridge is not None:
            bridge.detach()
        rt.shutdown()


if __name__ == "__main__":
    sys.exit(main())
