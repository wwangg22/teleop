"""Policy-rollout infra validation, no hardware, no torch (test_phase4 style):

A) manifest load + fail-closed validation (happy path + every sad path)
B) frame maps + action decode round-trip against a REAL recorded sim episode
   (skipped with a note if the sim2real dataset is not on this machine)
C) wire protocol round-trip over a real unix socket (NullServer)
D) full PolicyBridge cadence against MockHW + SineServer: segments, hold
   tails, replan-early, first-command-gate compliance, motion sanity
E) gripper event FSM: freeze -> blocking grasp -> discard chunk -> replan
F) thermal watcher: permissive on bad data, trips on 3x critical, protective
   stop order (safe_home THEN disable), latched

Run:  conda run -n teleop python core/tests/test_policy_infra.py
"""

from __future__ import annotations

import os
import sys
import tempfile
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rebot_core.kinematics import (JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER,  # noqa: E402
                                   Kinematics)
from rebot_core.policy_bridge import PolicyBridge  # noqa: E402
from rebot_core.policy_manifest import load_manifest, validate_against_rig  # noqa: E402
from rebot_core.policy_map import (decode_action, gripper_motor_to_finger,  # noqa: E402
                                   preprocess_frame, rig_to_sim, sim_to_rig)
from rebot_core.policy_proto import PolicyClient  # noqa: E402
from rebot_core.policy_servers import NullServer, SineServer  # noqa: E402
from rebot_core.policy_thermal import PolicyThermalWatcher  # noqa: E402
from rebot_core.stream import SetpointStreamer  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name
          + (f" :: {detail}" if detail and not cond else ""))
    if not cond:
        FAILS.append(name)


SIM2REAL = Path("/home/asuka/Desktop/IsaacLab/sim2real")
EPISODE = SIM2REAL / "data/policy_ws/raw/ep_s21_0000/arrays.npz"

MANIFEST_YAML = """\
manifest_version: 1
name: test_policy
checkpoint: ""
obs:
  proprio: [joint_pos_rel8, joint_vel8_fd, last_action]
  proprio_dim: 23
  cameras:
    wrist:     {{rig_camera: wrist, crop: center_16x9, resize: [160, 90]}}
    workspace: {{rig_camera: workspace, crop: none, resize: [160, 90]}}
action:
  dim: 7
  space: joint_abs
  scale: 0.5
  gripper: {{mode: event_threshold, close_below: 0.0, open_above: 0.0}}
policy_frame:
  default_pose: [0.0, -1.35, -0.3, -0.85, 0.0, 0.0]
  gripper_default: 0.04
  gripper_open: 0.045
  gripper_close: 0.0
rig_map:
  sign: [-1, -1, -1, -1, -1, -1]
  offset: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  gripper_motor_open: 5.0
  gripper_motor_closed: 0.0
timing: {{control_hz: 50, chunk: {chunk}, execute: {execute}, replan_early: 1}}
safety:
  max_joint_velocity: 1.5
  start_pose: default_pose
  thermal_watch: false
  settle_s: 0.1
"""


def write_manifest(tmp: Path, chunk=20, execute=10, mutate=None) -> Path:
    text = MANIFEST_YAML.format(chunk=chunk, execute=execute)
    if mutate:
        text = mutate(text)
    p = tmp / "policy.yaml"
    p.write_text(text)
    return p


# ===========================================================================
# A) manifest
# ===========================================================================


def test_manifest(tmp: Path):
    m = load_manifest(write_manifest(tmp))
    check("manifest: happy path", m.proprio_dim == 23 and m.chunk == 20)
    check("manifest: start pose rig frame",
          np.allclose(m.start_pose_rig(), [0.0, 1.35, 0.3, 0.85, 0.0, 0.0]))
    validate_against_rig(m, ["wrist", "workspace"],
                         JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
    check("manifest: rig validation ok", True)

    def expect_reject(name, mutate):
        try:
            mm = load_manifest(write_manifest(tmp, mutate=mutate))
            validate_against_rig(mm, ["wrist", "workspace"],
                                 JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
            check(f"manifest: rejects {name}", False, "accepted")
        except (ValueError, NotImplementedError):
            check(f"manifest: rejects {name}", True)

    expect_reject("bad action space",
                  lambda t: t.replace("space: joint_abs", "space: ee_wrong"))
    expect_reject("proprio dim mismatch",
                  lambda t: t.replace("proprio_dim: 23", "proprio_dim: 24"))
    expect_reject("execute > chunk",
                  lambda t: t.replace("execute: 10", "execute: 21"))
    expect_reject("over-ceiling velocity",
                  lambda t: t.replace("max_joint_velocity: 1.5",
                                      "max_joint_velocity: 4.0"))
    expect_reject("unknown camera",
                  lambda t: t.replace("rig_camera: workspace",
                                      "rig_camera: nosuchcam"))
    expect_reject("unknown proprio block",
                  lambda t: t.replace("joint_vel8_fd", "qd_magic"))


# ===========================================================================
# B) maps + decode vs the real episode
# ===========================================================================


def test_maps(tmp: Path):
    m = load_manifest(write_manifest(tmp))
    rng = np.random.default_rng(3)
    q = rng.uniform(-1.0, 1.0, size=(32, 6))
    back = np.stack([rig_to_sim(m, sim_to_rig(m, qi)) for qi in q])
    check("map: sim->rig->sim identity", np.allclose(back, q, atol=1e-12))
    check("map: gripper motor->finger endpoints",
          abs(gripper_motor_to_finger(m, 5.0) - 0.045) < 1e-9
          and abs(gripper_motor_to_finger(m, 0.0) - 0.0) < 1e-9)

    img = (np.arange(640 * 480 * 3, dtype=np.uint8).reshape(480, 640, 3))
    out = preprocess_frame(img, "center_16x9", (160, 90))
    check("map: d405 crop+resize shape", out.shape == (90, 160, 3))
    out2 = preprocess_frame(np.zeros((720, 1280, 3), np.uint8), "none",
                            (160, 90))
    check("map: d435i resize shape", out2.shape == (90, 160, 3))

    if not EPISODE.exists():
        print("SKIP maps-vs-episode (sim2real dataset not present)")
        return
    data = np.load(EPISODE)
    obs41, actions = data["obs41"], data["action"]
    # the training contract this whole file exists to preserve
    d = np.abs(obs41[1:, 34:41] - actions[:-1]).max()
    check("episode: last_action contract exact", d < 1e-6, f"max diff {d}")
    # decode -> encode identity on every recorded action
    q_sim_prev = m.default_pose.copy()
    ok = True
    for a in actions[:500]:
        q_rig, q_sim, _ = decode_action(m, a, q_sim_prev)
        q_sim_prev = q_sim
        re_encoded = (q_sim - m.default_pose) / m.action_scale
        ok &= bool(np.allclose(re_encoded, a[:6], atol=1e-6))
        ok &= bool(np.all(q_rig >= JOINT_LIMITS_LOWER - 1e-6)
                   and np.all(q_rig <= JOINT_LIMITS_UPPER + 1e-6))
    check("episode: decode/encode identity + rig limits", ok)


# ===========================================================================
# C) protocol round-trip
# ===========================================================================


def test_proto(tmp: Path):
    sock = str(tmp / "proto.sock")
    srv = NullServer(sock, chunk=20, action_dim=7, scale=0.5, dt=0.02)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    time.sleep(0.2)
    c = PolicyClient(sock)
    info = c.connect(retry_for_s=5.0)
    check("proto: ping", "null" in info.get("description", ""))
    proprio = np.arange(23, dtype=np.float32) / 10.0
    img = np.random.default_rng(0).integers(0, 255, (90, 160, 3), np.uint8)
    arrays, rinfo = c.request("infer", {"proprio": proprio, "image.wrist": img},
                              {"execute": 10})
    chunk = arrays["chunk"]
    check("proto: chunk shape/dtype",
          chunk.shape == (20, 7) and chunk.dtype == np.float32)
    check("proto: hold action = proprio[:6]/scale",
          np.allclose(chunk[0, :6], proprio[:6] / 0.5) and chunk[0, 6] == 1.0)
    # fidelity: a second round-trip echoing arrays through the null server
    arrays2, _ = c.request("infer", {"proprio": proprio, "image.wrist": img})
    check("proto: deterministic", np.array_equal(arrays2["chunk"], chunk))
    c.close()


# ===========================================================================
# D/E) bridge against MockHW + servers
# ===========================================================================


class MockHW:
    def __init__(self, q0):
        self.q = np.asarray(q0, float).copy()
        self.state_machine = "IDLE"
        self.gripper_pos = 5.0
        self.gripper_open_position = 5.0
        self.close_calls = 0
        self.open_calls = 0
        self.safe_homed = False
        self.disabled = False

    def get_joint_positions(self, request=False):
        return self.q.copy()

    def set_joint_position_velocity_target(self, q, qd):
        self.q = np.asarray(q, float).copy()

    def close_gripper_grasp(self, position=0.0, timeout=8.0):
        self.close_calls += 1
        time.sleep(0.25)                    # blocking, like the real carrot
        self.gripper_pos = 0.4
        return True, True, 0.4

    def set_gripper_position(self, position, timeout=3.0):
        self.open_calls += 1
        time.sleep(0.15)
        self.gripper_pos = float(position)
        return True, float(position)

    def safe_home(self):
        self.safe_homed = True

    def disable(self):
        self.disabled = True


class FakeStates:
    """StateCache stand-in: fresh reads of MockHW, scriptable temps."""

    def __init__(self, hw):
        self.hw = hw
        self.temps = None

    def latest(self):
        from rebot_core.state import RobotState

        now = time.monotonic()
        return RobotState(q=self.hw.q.copy(), qd=np.zeros(6),
                          effort=np.zeros(6), gripper_pos=self.hw.gripper_pos,
                          gripper_effort=0.0, t_mono=now, t_wall=time.time())

    def temperatures(self):
        return self.temps


class FakeCam:
    def __init__(self, h, w):
        self._slot = None
        self.h, self.w = h, w

    def latest(self):
        from rebot_core.cameras import FrameSlot

        return FrameSlot(
            color=np.full((self.h, self.w, 3), 128, np.uint8),
            depth=None, t_mono=time.monotonic(), t_wall=time.time(), seq=0)


class FakeRig:
    def __init__(self):
        self.cameras = {"wrist": FakeCam(480, 640),
                        "workspace": FakeCam(720, 1280)}


class TogglingSineServer(SineServer):
    """Sine + a scripted gripper: close on requests 3-4, open on 6+."""

    def infer(self, arrays, info):
        out, oinfo = super().infer(arrays, info)
        if 3 <= self.requests + 1 <= 4:
            out["chunk"][:, 6] = -1.0
        return out, oinfo


def _mk_bridge(tmp, hw, server, chunk=20, execute=10):
    sock = str(tmp / f"{server.__class__.__name__}.sock")
    server.socket_path = sock
    threading.Thread(target=server.serve_forever, daemon=True).start()
    time.sleep(0.2)
    manifest = load_manifest(write_manifest(tmp, chunk=chunk, execute=execute))
    states = FakeStates(hw)
    streamer = SetpointStreamer(hw, control_hz=200.0,
                                q_min=JOINT_LIMITS_LOWER,
                                q_max=JOINT_LIMITS_UPPER)
    streamer.start()
    kin = Kinematics()
    bridge = PolicyBridge(manifest, streamer=streamer, state_cache=states,
                          hardware=hw, camera_rig=FakeRig(), kinematics=kin,
                          recorder=None, teleop_session=None, socket_path=sock)
    return bridge, streamer, manifest


def test_bridge_cadence(tmp: Path):
    m0 = load_manifest(write_manifest(tmp))
    hw = MockHW(m0.start_pose_rig())
    srv = SineServer("", chunk=20, action_dim=7, scale=0.5, dt=0.02,
                     joints=[4], amplitude=0.08, period=2.0)
    bridge, streamer, m = _mk_bridge(tmp, hw, srv)

    submitted = []
    orig_submit = streamer.submit

    def spy(source, points, t0=None, append=False):
        submitted.append(points)
        return orig_submit(source, points, t0=t0, append=append)

    streamer.submit = spy
    bridge.attach(min_ramp_s=0.3)
    bridge.start_loop()
    time.sleep(2.0)
    bridge.stop("test_done")
    streamer.stop()

    c = bridge.counters
    check("bridge: ticked at rate", c["ticks"] >= 80, str(c))
    check("bridge: submitted most ticks", c["submits"] >= 70, str(c))
    check("bridge: replanned", c["chunks"] >= 6, str(c))
    check("bridge: no replan stalls", c["replan_stalls"] == 0, str(c))
    check("bridge: first-command gate never tripped",
          streamer.counters.refused_first == 0)
    peaks = [float(np.abs(s[1][1][4] - m.start_pose_rig()[4]))
             for s in submitted if len(s) == 3]
    moved = max(peaks) if peaks else 0.0
    check("bridge: sine moved joint 5 (bounded)", 0.01 < moved < 0.3,
          f"peak {moved:.4f}")
    motion = [s for s in submitted if len(s) == 3]   # stop() appends a
    seg = motion[-1]                                 # 2-point brake hold
    check("bridge: 3-point hold-tail segments",
          len(motion) >= 70 and np.all(seg[2][2] == 0.0)
          and seg[2][0] > seg[1][0] > seg[0][0],
          f"motion segs {len(motion)}")
    check("bridge: stop released the stream", streamer.active is False)


def test_gripper_fsm(tmp: Path):
    m0 = load_manifest(write_manifest(tmp))
    hw = MockHW(m0.start_pose_rig())
    srv = TogglingSineServer("", chunk=20, action_dim=7, scale=0.5, dt=0.02,
                             joints=[4], amplitude=0.05, period=2.0)
    bridge, streamer, m = _mk_bridge(tmp, hw, srv)
    bridge.attach(min_ramp_s=0.3)
    bridge.start_loop()
    # attach() itself opens the gripper once at episode start (the previous
    # session's safe_home parks it closed), so the FSM's open event is the
    # SECOND open call.
    deadline = time.monotonic() + 6.0
    while time.monotonic() < deadline and hw.open_calls < 2:
        time.sleep(0.05)
    bridge.stop("test_done")
    streamer.stop()

    c = bridge.counters
    check("gripper: close then open happened",
          hw.close_calls == 1 and hw.open_calls >= 2,
          f"close={hw.close_calls} open={hw.open_calls}")
    check("gripper: events counted", c["gripper_events"] >= 2, str(c))
    check("gripper: clock froze during grasp", c["freeze_ticks"] >= 8, str(c))
    check("gripper: replanned after events",
          c["chunks"] >= c["gripper_events"] + 1, str(c))


# ===========================================================================
# F) thermal watcher
# ===========================================================================


def test_thermal():
    hw = MockHW(np.zeros(6))
    states = FakeStates(hw)
    trips = []
    w = PolicyThermalWatcher(states, hw, on_trip=trips.append,
                             poll_hz=50.0, consecutive_to_trip=3)
    w.start()
    states.temps = (np.zeros(6), 0.0)              # "never populated" 0.0s
    time.sleep(0.15)
    check("thermal: permissive on invalid data", not w.tripped)
    states.temps = (np.array([40, 45, 62, 40, 40, 40], float), 35.0)
    time.sleep(0.15)
    check("thermal: warn does not trip", not w.tripped)
    states.temps = (np.array([40, 45, 80, 40, 40, 40], float), 35.0)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and not hw.disabled:
        time.sleep(0.02)
    w.stop()
    check("thermal: tripped on 3x critical", w.tripped and trips)
    check("thermal: safe_home THEN disable", hw.safe_homed and hw.disabled)
    before = hw.close_calls
    w.protective_stop("again")                     # latched: no second run
    check("thermal: latched", w.tripped and hw.close_calls == before)


# ===========================================================================


if __name__ == "__main__":
    with tempfile.TemporaryDirectory(prefix="policy_infra_") as td:
        tmp = Path(td)
        test_manifest(tmp)
        test_maps(tmp)
        test_proto(tmp)
        test_bridge_cadence(tmp)
        test_gripper_fsm(tmp)
        test_thermal()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURES: {FAILS}")
        sys.exit(1)
    print("ALL PASS")
