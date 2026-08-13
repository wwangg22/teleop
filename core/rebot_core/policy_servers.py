"""ML-free test servers for the policy-rollout infra (bring-up gate G0.5).

Both speak the C2 protocol (policy_proto) and assume ONLY the flow-family
proprio convention "first 6 entries = joint_pos_rel (policy frame)" plus a
joint_abs decode with offset=default_pose -- which lets them emit a HOLD
action for the arm's CURRENT pose without knowing anything else:

    a_hold[:6] = joint_pos_rel / scale     (decodes back to the current q)
    a_hold[6]  = +1.0                      (gripper open, demo convention)

NullServer  -- always the hold action: the full bridge path moves nothing.
SineServer  -- hold + a small sine on selected joints (POLICY frame), the
               rebotarm_ros2 mock_policy.py practice: prove manifest ->
               socket -> chunk -> streamer -> MIT on hardware with a
               harmless, obviously-recognizable motion. Amplitude defaults
               deliberately tiny.

Usage (either conda env; numpy only):
  python -m rebot_core.policy_servers --mode sine --socket /tmp/rebot_policy.sock \
      --chunk 50 --dt 0.02 --scale 0.5 --joints 4 5 --amplitude 0.08 --period 6
"""

from __future__ import annotations

import argparse
import logging

import numpy as np

from .policy_proto import PolicyServer

log = logging.getLogger("policy.servers")


class NullServer(PolicyServer):
    description = "null (hold current pose)"

    def __init__(self, socket_path: str, chunk: int, action_dim: int,
                 scale: float, dt: float) -> None:
        super().__init__(socket_path)
        self.chunk, self.action_dim = int(chunk), int(action_dim)
        self.scale, self.dt = float(scale), float(dt)
        self._anchor: np.ndarray | None = None

    def hold_action(self, proprio: np.ndarray) -> np.ndarray:
        # Anchor the hold to the FIRST pose seen, not the current one: an
        # echo-of-current hold re-anchors to the sagged pose every replan and
        # gravity ratchets the arm downward (measured on hardware, G0.5
        # 2026-08-12: joint3 0.33 -> 1.51 rad over 20 s at the extended
        # start pose). A fixed anchor makes the PD+FF fight for the original
        # pose instead of chasing the sag.
        if self._anchor is None:
            self._anchor = np.asarray(proprio[:6], np.float32).copy()
        a = np.zeros(self.action_dim, dtype=np.float32)
        a[:6] = self._anchor / self.scale
        if self.action_dim > 6:
            a[6] = 1.0
        return a

    def infer(self, arrays, info):
        proprio = arrays["proprio"]
        chunk = np.tile(self.hold_action(proprio), (self.chunk, 1))
        return {"chunk": chunk.astype(np.float32)}, {"server": "null"}


class SineServer(NullServer):
    description = "sine (hold + small sine on selected joints)"

    def __init__(self, socket_path: str, chunk: int, action_dim: int,
                 scale: float, dt: float, joints: list[int],
                 amplitude: float, period: float) -> None:
        super().__init__(socket_path, chunk, action_dim, scale, dt)
        self.joints = [int(j) for j in joints]
        self.amplitude, self.period = float(amplitude), float(period)
        self._t = 0.0            # continuous phase across requests

    def infer(self, arrays, info):
        base = self.hold_action(arrays["proprio"])
        chunk = np.tile(base, (self.chunk, 1)).astype(np.float32)
        # executed steps advance the phase; the tail beyond `execute` is
        # re-predicted next request, matching how chunked policies behave
        steps = self._t + self.dt * np.arange(self.chunk)
        wave = self.amplitude * np.sin(2 * np.pi * steps / self.period)
        for j in self.joints:
            # amplitude is a JOINT-space rad offset; convert to action units
            chunk[:, j] += wave.astype(np.float32) / self.scale
        executed = int(info.get("execute", self.chunk))
        self._t += self.dt * executed
        return {"chunk": chunk}, {"server": "sine", "phase_s": round(self._t, 3)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--mode", choices=("null", "sine"), default="null")
    ap.add_argument("--socket", default="/tmp/rebot_policy.sock")
    ap.add_argument("--chunk", type=int, default=50)
    ap.add_argument("--action-dim", type=int, default=7)
    ap.add_argument("--scale", type=float, default=0.5)
    ap.add_argument("--dt", type=float, default=0.02)
    ap.add_argument("--joints", type=int, nargs="+", default=[4, 5],
                    help="policy-frame joint indices for the sine (0-based)")
    ap.add_argument("--amplitude", type=float, default=0.08,
                    help="rad, joint space -- keep it small")
    ap.add_argument("--period", type=float, default=6.0)
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s",
                        datefmt="%H:%M:%S")
    if args.mode == "null":
        srv = NullServer(args.socket, args.chunk, args.action_dim,
                         args.scale, args.dt)
    else:
        srv = SineServer(args.socket, args.chunk, args.action_dim,
                         args.scale, args.dt, args.joints,
                         args.amplitude, args.period)
    srv.serve_forever()
    return 0


if __name__ == "__main__":
    import sys

    sys.exit(main())
