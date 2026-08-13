"""Policy manifest (contract C1 of the policy-rollout infra).

A trained policy deploys onto this rig by shipping a `policy.yaml` next to
its checkpoint. The manifest declares EVERYTHING the (policy-agnostic)
PolicyBridge needs: observation layout, camera preprocessing, action space
+ decode constants, the sim->rig joint map, timing, and safety knobs. The
bridge refuses to start on anything it cannot satisfy -- validation is
fail-closed and cross-checked against the live rig (camera names, joint
limits, streamer velocity ceiling).

Design rationale (sim2real/notes/irl_rollout_plan.md §0b): genericity lives
in this schema + the wire protocol (policy_proto) + the recording schema.
Porting a NEW policy family = a new manifest + (at most) a new inference
server; the runtime-side code never changes.

The sim->rig joint map deserves emphasis: the Isaac training URDF and the
vendor RS URDF describe the same arm with every revolute axis OPPOSITE
(joint2/3 limit windows mirrored). The map was SOLVED numerically, not
eyeballed -- core/tools/solve_joint_map.py, frame-invariant pairwise
distance + relative-rotation-angle signature over all 64 sign vectors:
sign = [-1]*6, offset = 0, residual 1.7e-5 m (runner-up 200x worse).
"""

from __future__ import annotations

import hashlib
import logging
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

log = logging.getLogger("policy.manifest")

MANIFEST_VERSION = 1

# proprio blocks the bridge knows how to assemble (policy_map.build_proprio).
# A new observation layout = a new block builder there + a name here.
KNOWN_PROPRIO_BLOCKS = {
    "joint_pos_rel8": 8,   # arm(6)+fingers(2), policy frame, minus default
    "joint_vel8_fd": 8,    # finite-difference (hardware qd is untrustworthy)
    "joint_pos_rel6": 6,
    "joint_vel6_fd": 6,
    "last_action": None,   # action.dim, maintained by the bridge
}
KNOWN_ACTION_SPACES = ("joint_abs", "joint_delta")
KNOWN_GRIPPER_MODES = ("event_threshold", "continuous_ratelimited", "none")
KNOWN_CROPS = ("none", "center_16x9")

# The runtime constructs its SetpointStreamer with this ceiling (stream.py
# default). A manifest may ask for LESS (the bridge enforces its own clamp);
# asking for more is refused -- raising the rig ceiling is a human/runtime
# decision, never a manifest one.
RUNTIME_STREAMER_MAX_VEL = 2.0


@dataclass
class CameraSpec:
    rig_camera: str                 # name in station.yaml
    resize: tuple[int, int]         # (width, height) fed to the policy
    crop: str = "none"
    device: str = ""                # informational (d405/d435i)


@dataclass
class PolicyManifest:
    name: str
    proprio_blocks: list[str]
    proprio_dim: int
    cameras: dict[str, CameraSpec]
    action_dim: int
    action_space: str
    action_scale: float
    gripper_mode: str
    gripper_close_below: float
    gripper_open_above: float
    default_pose: np.ndarray        # (6,) policy(sim) frame
    gripper_default: float          # finger m at default pose
    gripper_open: float             # finger m fully open (policy frame)
    gripper_close: float
    rig_sign: np.ndarray            # (6,) sim->rig: q_rig = sign*q_sim + offset
    rig_offset: np.ndarray
    rig_gripper_motor_open: float   # rig gripper motor rad at open
    rig_gripper_motor_closed: float
    control_hz: float
    chunk: int
    execute: int
    replan_early: int
    max_joint_velocity: float
    start_pose: np.ndarray          # (6,) policy frame
    thermal_watch: bool
    settle_s: float
    checkpoint: str = ""
    ckpt_sha256: str = ""
    server: dict = field(default_factory=dict)
    raw: dict = field(default_factory=dict)
    path: str = ""

    # -- derived ---------------------------------------------------------

    @property
    def tick_dt(self) -> float:
        return 1.0 / self.control_hz

    def start_pose_rig(self) -> np.ndarray:
        return self.rig_sign * self.start_pose + self.rig_offset


def _fail(msg: str) -> None:
    raise ValueError(f"policy manifest rejected: {msg}")


def load_manifest(path: str | Path) -> PolicyManifest:
    """Parse + self-validate. Rig cross-checks live in validate_against_rig()
    because they need the station config / kinematics, which the caller owns."""
    path = Path(path)
    raw = yaml.safe_load(path.read_text())
    if not isinstance(raw, dict):
        _fail("not a mapping")
    if int(raw.get("manifest_version", -1)) != MANIFEST_VERSION:
        _fail(f"manifest_version must be {MANIFEST_VERSION}")
    for key in ("name", "obs", "action", "policy_frame", "rig_map", "timing",
                "safety"):
        if key not in raw:
            _fail(f"missing section {key!r}")

    obs, act = raw["obs"], raw["action"]
    pf, rm = raw["policy_frame"], raw["rig_map"]
    tm, sf = raw["timing"], raw["safety"]

    action_dim = int(act.get("dim", 0))
    if action_dim < 1:
        _fail("action.dim must be >= 1")
    blocks = list(obs.get("proprio", []))
    if not blocks:
        _fail("obs.proprio must list at least one block")
    dim = 0
    for b in blocks:
        if b not in KNOWN_PROPRIO_BLOCKS:
            _fail(f"unknown proprio block {b!r} (known: "
                  f"{sorted(KNOWN_PROPRIO_BLOCKS)})")
        dim += KNOWN_PROPRIO_BLOCKS[b] or action_dim
    declared = int(obs.get("proprio_dim", dim))
    if declared != dim:
        _fail(f"obs.proprio_dim={declared} but blocks sum to {dim}")

    cameras: dict[str, CameraSpec] = {}
    for cam_name, spec in (obs.get("cameras") or {}).items():
        crop = str(spec.get("crop", "none"))
        if crop not in KNOWN_CROPS:
            _fail(f"camera {cam_name!r}: unknown crop {crop!r}")
        resize = spec.get("resize")
        if (not isinstance(resize, (list, tuple)) or len(resize) != 2
                or int(resize[0]) < 1 or int(resize[1]) < 1):
            _fail(f"camera {cam_name!r}: resize must be [width, height]")
        cameras[str(cam_name)] = CameraSpec(
            rig_camera=str(spec.get("rig_camera", cam_name)),
            resize=(int(resize[0]), int(resize[1])),
            crop=crop,
            device=str(spec.get("device", "")),
        )

    space = str(act.get("space", ""))
    if space not in KNOWN_ACTION_SPACES:
        _fail(f"action.space {space!r} not in {KNOWN_ACTION_SPACES} "
              "(ee_pose is planned, not implemented)")
    gr = act.get("gripper") or {"mode": "none"}
    mode = str(gr.get("mode", "none"))
    if mode not in KNOWN_GRIPPER_MODES:
        _fail(f"gripper.mode {mode!r} not in {KNOWN_GRIPPER_MODES}")

    def vec6(section: dict, key: str) -> np.ndarray:
        v = section.get(key)
        if not isinstance(v, (list, tuple)) or len(v) != 6:
            _fail(f"{key} must be a 6-vector")
        return np.asarray(v, dtype=float)

    sign = vec6(rm, "sign")
    if not np.all(np.isin(sign, (-1.0, 1.0))):
        _fail("rig_map.sign entries must be +1 or -1")
    default_pose = vec6(pf, "default_pose")

    chunk = int(tm.get("chunk", 0))
    execute = int(tm.get("execute", 0))
    replan_early = int(tm.get("replan_early", 0))
    control_hz = float(tm.get("control_hz", 0.0))
    if not (1 <= execute <= chunk):
        _fail(f"timing: need 1 <= execute({execute}) <= chunk({chunk})")
    if not (0 <= replan_early < execute):
        _fail(f"timing: need 0 <= replan_early({replan_early}) < execute")
    if not (1.0 <= control_hz <= 200.0):
        _fail(f"timing.control_hz {control_hz} outside [1, 200]")

    max_vel = float(sf.get("max_joint_velocity", 1.0))
    if not (0.05 <= max_vel <= RUNTIME_STREAMER_MAX_VEL):
        _fail(
            f"safety.max_joint_velocity {max_vel} outside "
            f"[0.05, {RUNTIME_STREAMER_MAX_VEL}] -- the runtime streamer "
            "clamps at "
            f"{RUNTIME_STREAMER_MAX_VEL} rad/s; raising THAT is a runtime "
            "decision, not a manifest one"
        )

    start = sf.get("start_pose", "default_pose")
    start_pose = default_pose.copy() if start == "default_pose" else np.asarray(
        start, dtype=float
    )
    if start_pose.shape != (6,):
        _fail("safety.start_pose must be 'default_pose' or a 6-vector")

    m = PolicyManifest(
        name=str(raw["name"]),
        proprio_blocks=blocks,
        proprio_dim=dim,
        cameras=cameras,
        action_dim=action_dim,
        action_space=space,
        action_scale=float(act.get("scale", 1.0)),
        gripper_mode=mode,
        gripper_close_below=float(gr.get("close_below", 0.0)),
        gripper_open_above=float(gr.get("open_above", 0.0)),
        default_pose=default_pose,
        gripper_default=float(pf.get("gripper_default", 0.0)),
        gripper_open=float(pf.get("gripper_open", 0.0)),
        gripper_close=float(pf.get("gripper_close", 0.0)),
        rig_sign=sign,
        rig_offset=vec6(rm, "offset"),
        rig_gripper_motor_open=float(rm.get("gripper_motor_open", 5.0)),
        rig_gripper_motor_closed=float(rm.get("gripper_motor_closed", 0.0)),
        control_hz=control_hz,
        chunk=chunk,
        execute=execute,
        replan_early=replan_early,
        max_joint_velocity=max_vel,
        start_pose=start_pose,
        thermal_watch=bool(sf.get("thermal_watch", False)),
        settle_s=float(sf.get("settle_s", 1.0)),
        checkpoint=str(raw.get("checkpoint", "")),
        ckpt_sha256=str(raw.get("ckpt_sha256", "")),
        server=dict(raw.get("server") or {}),
        raw=raw,
        path=str(path),
    )
    return m


def validate_against_rig(
    m: PolicyManifest,
    camera_names: list[str],
    q_min: np.ndarray,
    q_max: np.ndarray,
) -> None:
    """Cross-checks that need the live rig. Fail-closed."""
    for cam_name, spec in m.cameras.items():
        if spec.rig_camera not in camera_names:
            _fail(
                f"camera {cam_name!r} wants rig camera {spec.rig_camera!r}; "
                f"rig has {camera_names}"
            )
    start_rig = m.start_pose_rig()
    if np.any(start_rig < np.asarray(q_min) - 1e-9) or np.any(
        start_rig > np.asarray(q_max) + 1e-9
    ):
        _fail(
            f"start pose (rig frame) {np.round(start_rig, 3).tolist()} "
            f"violates rig joint limits"
        )


def manifest_provenance(m: PolicyManifest) -> dict:
    """What gets recorded with every rollout episode (contract C3)."""
    out = {
        "policy_name": m.name,
        "manifest_path": m.path,
        "checkpoint": m.checkpoint,
        "ckpt_sha256": m.ckpt_sha256,
    }
    if m.checkpoint:
        ckpt = Path(m.checkpoint)
        if not ckpt.is_absolute() and m.path:
            ckpt = Path(m.path).parent / ckpt
        if ckpt.exists() and not m.ckpt_sha256:
            h = hashlib.sha256()
            with open(ckpt, "rb") as f:
                for block in iter(lambda: f.read(1 << 20), b""):
                    h.update(block)
            out["ckpt_sha256"] = h.hexdigest()
    return out
