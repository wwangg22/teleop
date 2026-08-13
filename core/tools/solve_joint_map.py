"""Solve the sim->rig joint-space map between two URDFs of the same arm.

The Isaac training assets (eva_rl's RS-rebot-dev-arm URDF/USD) and this
stack's vendor RS URDF describe the SAME physical arm with DIFFERENT joint
conventions: every revolute <axis> is opposite and joint2/joint3's limit
windows are mirrored ([-3.14,0] vs [0,3.14]). A policy trained in the sim
convention must have its joint targets mapped before they reach the
SetpointStreamer, and the map must be MEASURED, not eyeballed.

Method: for each candidate per-joint sign vector s (offsets are zero by the
mirrored-limit evidence, but an optional offset solve is included), sample
random in-limit sim configurations q and compare the PAIRWISE DISTANCES of
the EE positions between models: || p_sim(q_i) - p_sim(q_j) || must equal
|| p_rig(s*q_i + c) - p_rig(s*q_j + c) || for the true map, regardless of
any rigid base/tool frame difference between the URDFs. Position distances
cannot see joint6 (the EE origin lies on the roll axis), so the metric also
compares RELATIVE ROTATION ANGLES, angle(R(q_i)^T R(q_j)) -- similar
matrices under constant base/tool offsets, hence frame-invariant too. The
unique sign vector that drives the worst-case mismatch to ~0 is the map.

Usage (conda env teleop -- needs pinocchio):
  python core/tools/solve_joint_map.py \
      [--sim-urdf PATH] [--rig-urdf PATH] [--samples 24] [--seed 0]

Prints the winning sign vector + residual, and the manifest snippet.
"""

from __future__ import annotations

import argparse
import itertools
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "core"))

from rebot_core.kinematics import Kinematics  # noqa: E402

DEFAULT_SIM_URDF = (
    "/home/asuka/Desktop/IsaacLab/eva_rl/source/reBot_RL/data/"
    "RS-rebot-dev-arm/00-arm-rs_asm-v3.urdf"
)


def _limits_from_urdf(path: str) -> tuple[np.ndarray, np.ndarray]:
    """Arm-joint limits straight from the URDF text (pinocchio reorders
    nothing for this simple chain, but read them by joint name anyway)."""
    import re

    txt = Path(path).read_text()
    lo, hi = np.zeros(6), np.zeros(6)
    for m in re.finditer(
        r'<joint\s+name="(joint[1-6])"[^>]*type="revolute"(.*?)</joint>', txt, re.S
    ):
        name, body = m.groups()
        lim = re.search(r'<limit[^>]*lower="([^"]+)"[^>]*upper="([^"]+)"', body)
        if lim is None:
            lim = re.search(
                r'<limit.*?lower="([^"]+)".*?upper="([^"]+)"', body, re.S
            )
        idx = int(name[len("joint"):]) - 1
        lo[idx], hi[idx] = float(lim.group(1)), float(lim.group(2))
    return lo, hi


def solve(sim_urdf: str, rig_urdf: str | None, samples: int, seed: int):
    sim = Kinematics(urdf_path=sim_urdf, lower=[-9] * 6, upper=[9] * 6)
    rig = Kinematics() if rig_urdf is None else Kinematics(urdf_path=rig_urdf)

    lo_s, hi_s = _limits_from_urdf(sim_urdf)
    rng = np.random.default_rng(seed)
    # Sample away from the exact limits so a sign flip cannot push a config
    # marginally out of the rig model's range and distort its FK clipping.
    qs = lo_s + (hi_s - lo_s) * (0.1 + 0.8 * rng.random((samples, 6)))

    def fk_all(kin, cfgs):
        import pinocchio as pin

        ps, rots = [], []
        for q in cfgs:
            pos, quat = kin.fk(q)
            ps.append(pos)
            rots.append(pin.Quaternion(quat[3], quat[0], quat[1], quat[2]).matrix())
        return np.stack(ps), rots

    def rel_angles(rots):
        n = len(rots)
        out = np.zeros((n, n))
        for i in range(n):
            for j in range(i + 1, n):
                rel = rots[i].T @ rots[j]
                c = np.clip((np.trace(rel) - 1.0) / 2.0, -1.0, 1.0)
                out[i, j] = out[j, i] = float(np.arccos(c))
        return out

    p_sim, r_sim = fk_all(sim, qs)
    d_sim = np.linalg.norm(p_sim[:, None] - p_sim[None, :], axis=-1)
    a_sim = rel_angles(r_sim)

    results = []
    for signs in itertools.product((1.0, -1.0), repeat=6):
        s = np.array(signs)
        p_rig, r_rig = fk_all(rig, [s * q for q in qs])
        d_rig = np.linalg.norm(p_rig[:, None] - p_rig[None, :], axis=-1)
        err = float(np.max(np.abs(d_rig - d_sim)))
        err += float(np.max(np.abs(rel_angles(r_rig) - a_sim)))
        results.append((err, signs))
    results.sort()
    return results


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim-urdf", default=DEFAULT_SIM_URDF)
    ap.add_argument("--rig-urdf", default=None,
                    help="default: the stack's vendor RS URDF")
    ap.add_argument("--samples", type=int, default=24)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    results = solve(args.sim_urdf, args.rig_urdf, args.samples, args.seed)
    best_err, best_signs = results[0]
    second_err, second_signs = results[1]
    print(f"best   signs {list(best_signs)}  worst pairwise mismatch {best_err:.2e} m")
    print(f"second signs {list(second_signs)}  mismatch {second_err:.2e} m")
    ambiguous = second_err < 10 * max(best_err, 1e-12)
    if ambiguous:
        print("AMBIGUOUS -- two maps fit; add samples or check the URDFs.")
    else:
        print("manifest snippet:")
        print(f"  rig_map:\n    sign: {[int(v) for v in best_signs]}\n"
              f"    offset: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]")
    return 1 if ambiguous else 0


if __name__ == "__main__":
    sys.exit(main())
