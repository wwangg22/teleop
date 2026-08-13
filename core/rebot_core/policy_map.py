"""Pure math for the policy bridge: frame maps, obs assembly, action decode,
camera preprocessing. No threads, no hardware -- everything here is unit-
tested offline (core/tests/test_policy_infra.py) against a real recorded
sim episode, because every constant in this file is a place where a silent
sign/unit error becomes a wrong motion on a real arm.

Frames:
    policy frame  -- the convention the checkpoint was trained in (Isaac
                     URDF: joint2/3 limits [-3.14, 0], elbow angles negative).
    rig frame     -- this stack's vendor RS URDF (joint2/3 limits [0, 3.14]).
    q_rig = sign * q_sim + offset;  sign/offset come from the manifest and
    were solved numerically (core/tools/solve_joint_map.py: sign=[-1]*6).

Gripper:
    The rig gripper is ONE motor (open=5.0 rad .. closed=0.0); the policy
    frame has TWO prismatic fingers sharing a command (data shows both
    finger columns identical). Linear map through "fraction open".
"""

from __future__ import annotations

import numpy as np
from PIL import Image

from .policy_manifest import PolicyManifest


# ---------------------------------------------------------------------------
# joint-space maps
# ---------------------------------------------------------------------------


def sim_to_rig(m: PolicyManifest, q_sim: np.ndarray) -> np.ndarray:
    return m.rig_sign * np.asarray(q_sim, float) + m.rig_offset


def rig_to_sim(m: PolicyManifest, q_rig: np.ndarray) -> np.ndarray:
    # sign is its own inverse (entries are +-1)
    return m.rig_sign * (np.asarray(q_rig, float) - m.rig_offset)


def gripper_motor_to_finger(m: PolicyManifest, motor_pos: float) -> float:
    """Rig gripper motor rad -> policy-frame finger position (m)."""
    span = m.rig_gripper_motor_open - m.rig_gripper_motor_closed
    frac = 0.0 if abs(span) < 1e-9 else (
        (float(motor_pos) - m.rig_gripper_motor_closed) / span
    )
    frac = float(np.clip(frac, 0.0, 1.0))
    return m.gripper_close + frac * (m.gripper_open - m.gripper_close)


# ---------------------------------------------------------------------------
# proprio assembly (contract: the manifest's obs.proprio block list)
# ---------------------------------------------------------------------------


def build_proprio(
    m: PolicyManifest,
    q_sim: np.ndarray,          # (6,) policy frame
    qd_sim: np.ndarray,         # (6,) policy frame, finite-differenced
    finger_pos: float,          # policy-frame finger m (shared by both)
    finger_vel: float,          # policy-frame finger m/s
    last_action: np.ndarray,    # (action_dim,) policy frame
) -> np.ndarray:
    blocks: list[np.ndarray] = []
    for name in m.proprio_blocks:
        if name == "joint_pos_rel8":
            rel = q_sim - m.default_pose
            f = finger_pos - m.gripper_default
            blocks.append(np.concatenate([rel, [f, f]]))
        elif name == "joint_pos_rel6":
            blocks.append(q_sim - m.default_pose)
        elif name == "joint_vel8_fd":
            blocks.append(np.concatenate([qd_sim, [finger_vel, finger_vel]]))
        elif name == "joint_vel6_fd":
            blocks.append(qd_sim)
        elif name == "last_action":
            blocks.append(np.asarray(last_action, float))
        else:                       # load_manifest already refused unknowns
            raise ValueError(f"unknown proprio block {name!r}")
    out = np.concatenate(blocks).astype(np.float32)
    if out.shape != (m.proprio_dim,):
        raise ValueError(
            f"proprio came out {out.shape}, manifest says ({m.proprio_dim},)"
        )
    return out


# ---------------------------------------------------------------------------
# action decode
# ---------------------------------------------------------------------------


def decode_action(
    m: PolicyManifest,
    action: np.ndarray,             # (action_dim,) raw policy output
    q_sim_prev_target: np.ndarray,  # (6,) previous target, policy frame
) -> tuple[np.ndarray, np.ndarray, float]:
    """-> (q_rig_target(6,), q_sim_target(6,), gripper_cmd).

    joint_abs:   q_sim = scale * a[:6] + default_pose   (the eva_bc contract:
                 a = (q_target - q_default) / scale, scale 0.5)
    joint_delta: q_sim = q_sim_prev_target + scale * a[:6]
    gripper_cmd is the RAW last channel; its meaning belongs to the gripper
    mode (event thresholds / continuous), not to this function.
    """
    a = np.asarray(action, float)
    if m.action_space == "joint_abs":
        q_sim = m.action_scale * a[:6] + m.default_pose
    elif m.action_space == "joint_delta":
        q_sim = np.asarray(q_sim_prev_target, float) + m.action_scale * a[:6]
    else:
        raise NotImplementedError(
            f"action space {m.action_space!r} (ee_pose is planned: decode to "
            "pose, kinematics.ik with the previous target as seed, reject on "
            "non-convergence -- not implemented until a policy needs it)"
        )
    grip = float(a[6]) if m.action_dim > 6 else 0.0
    return sim_to_rig(m, q_sim), q_sim, grip


# ---------------------------------------------------------------------------
# camera preprocessing
# ---------------------------------------------------------------------------


def preprocess_frame(rgb: np.ndarray, crop: str, resize: tuple[int, int]
                     ) -> np.ndarray:
    """RGB HxWx3 uint8 -> (resize_h, resize_w, 3) uint8.

    center_16x9: crop height to width*9/16, centered -- the D405's only 4:3
    mode vs the 16:9 training render (plan §2). Downscale uses PIL BOX
    (area averaging), the closest match to the cv2.INTER_AREA used to build
    the training frames (cv2 is not in the teleop env).
    """
    h, w = rgb.shape[:2]
    if crop == "center_16x9":
        want_h = int(round(w * 9 / 16))
        if want_h < h:
            top = (h - want_h) // 2
            rgb = rgb[top:top + want_h]
    elif crop != "none":
        raise ValueError(f"unknown crop {crop!r}")
    out_w, out_h = resize
    img = Image.fromarray(rgb, mode="RGB").resize(
        (out_w, out_h), Image.Resampling.BOX
    )
    return np.asarray(img, dtype=np.uint8)
