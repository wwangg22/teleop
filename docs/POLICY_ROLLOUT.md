# Policy rollout — generic deployment of trained policies on the reBot arm

*Added 2026-08-12. Design plan (with the sim-side context):
`/home/asuka/Desktop/IsaacLab/sim2real/notes/irl_rollout_plan.md`. This doc
is the operator/developer manual for the implementation.*

The infra deploys ANY trained policy through three stable contracts. A new
policy = a `policy.yaml` manifest next to its checkpoint + (at most) a new
inference-server adapter for its model family. Nothing in the teleop
runtime changes per policy, and the ordinary teleop stack behaves
identically when no policy is running.

```
 [inference process]                    [rebot_core process]
 serve_policy_flow.py  <-- unix --  PolicyBridge (policy_bridge.py)
 (env_isaaclab6, torch)   socket       |  obs assembly (StateCache, cameras)
 NullServer / SineServer  (C2:         |  action decode (manifest, C1)
 (ML-free checkout)   policy_proto)    v
                                    SetpointStreamer.submit("policy", ...)
                                       100 Hz Hermite upsample + clamps
                                       -> HardwareManager -> 500 Hz MIT -> CAN
```

## The three contracts

**C1 — manifest** (`core/rebot_core/policy_manifest.py`): observation
blocks, camera crop/resize, action space + decode constants, the sim->rig
joint map, timing (chunk/execute/replan), safety knobs. Fail-closed: load
+ `validate_against_rig` refuse anything unsatisfiable. Template:
`sim2real/scripts/policy/manifests/policy_template.yaml`; a filled example:
`sim2real/data/policy_ws/runs/E_base124/policy.yaml`.

**C2 — wire protocol** (`core/rebot_core/policy_proto.py`): stdlib-only
length-prefixed JSON+raw-ndarray frames over a unix socket, versioned.
`PolicyClient` (bridge side) / `PolicyServer` (subclass, implement
`infer`). Torch stays in ITS OWN process: the 500 Hz MIT loop lives on the
rebot_core GIL, and the sentinel's gil_starvation check exists because
in-process compute has starved it before.

**C3 — recording**: every rollout writes `/policy/obs_proprio`,
`/policy/action`, `/policy/chunk_meta` (registered in
`recorder.STRUCTURED_TOPICS`) and embeds the manifest as an MCAP
attachment (`policy/manifest.yaml`) with the checkpoint sha in every
chunk_meta row — episodes self-describe which policy produced them.

## The sim->rig joint map (READ THIS BEFORE HARDWARE)

The Isaac training URDF and the vendor RS URDF describe the same arm with
every revolute axis OPPOSITE (joint2/3 limit windows mirrored:
sim [-3.14, 0] vs rig [0, 3.14]). The map is **q_rig = -q_sim, offset 0,
on all six joints** — solved numerically by
`core/tools/solve_joint_map.py` (frame-invariant pairwise-distance +
relative-rotation-angle signature over all 64 sign vectors; residual
1.7e-5 m, runner-up 200x worse). It ships in each manifest's `rig_map`.
Gate G1 (replay at 0.25x) is the hardware confirmation — run it before
any policy rollout on a new manifest.

Gripper: rig = ONE motor (open 5.0 rad -> closed 0.0); policy frame = two
prismatic fingers sharing a command. Linear map through fraction-open
(`policy_map.gripper_motor_to_finger`), constants in the manifest.

## What the bridge does (policy_bridge.py)

- **Ownership**: `teleop.stop()` -> `streamer.activate("policy")` — the
  same exclusive-source rule as everything else; then a slow explicit ramp
  to the manifest start pose (respects the 0.35 rad first-command gate).
- **Control**: `control_hz` loop, one action per tick, submitted as the
  proven 3-point hold-tail segment with measured-dt velocity feedforward
  (`teleop._publish_segment` shape — extrapolating tails ratchet).
- **Replanning**: chunk covers `execute` ticks; the next chunk is requested
  `replan_early` ticks before exhaustion on a dedicated inference thread —
  the control loop never blocks on the socket.
- **Gripper (event mode)**: on a[6] crossing the threshold the POLICY CLOCK
  FREEZES (streamer holds position natively), the force-capped
  `close_gripper_grasp()` / `set_gripper_position()` runs on a worker
  thread (stall == grasp; state flips on any completed call), then the
  stale chunk is DISCARDED and a fresh one requested. Rationale: real
  close takes 1–6 s vs ~0.4 s in the demos; the scene barely changes
  during a grasp. `continuous_ratelimited` is declared but NOT implemented
  — raw `set_gripper_target` streaming bypasses the carrot force cap
  (14 Nm frame ceiling ≈ 500 N at the fingers, the thing that broke
  gripper hardware); implement the rate limit before ever enabling it.
- **Safety**: bridge-side per-tick velocity clamp at the manifest ceiling
  (validated <= the streamer's 2.0, which stays as the independent second
  line); fail-closed stop on inference errors or stale observations
  (streamer then holds forever); `stop()` = hold + deactivate (the
  software policy stop — power switch remains the true e-stop);
  optional thermal watcher.
- **Thermal** (`policy_thermal.py`): port of the ROS thermal_guard.
  t_mos only via StateCache's existing ~2 s poll (zero extra CAN), warn
  60 C / critical 75 C x3 consecutive, fail-permissive on invalid
  readings, protective stop = safe_home THEN disable (never a mid-pose
  drop; if homing fails it HOLDS energized). Started only by the bridge
  and only when the manifest sets `safety.thermal_watch: true`.

## Running it

```bash
# 0. inference server (its own process/env; GPU when free):
/home/asuka/miniconda3/envs/env_isaaclab6/bin/python \
    sim2real/scripts/policy/serve_policy_flow.py \
    --ckpt <run>/ckpt_final.pt --socket /tmp/rebot_policy.sock

# rollout (HARDWARE MOVES; --arm is the explicit consent):
conda run -n teleop python -m rebot_core.policy_runtime \
    --manifest <run>/policy.yaml --episodes 3 --episode-s 25 --arm
```

## Bring-up gates (from the plan; each passes before the next)

| gate | what | how | status |
|---|---|---|---|
| G0 | offline validation | `core/tests/test_policy_infra.py` (36 checks) + `sim2real/scripts/policy/replay_sim_traj.py` (decode/limits/mock-stream) + `test_serve_policy_cpu.py` (real ckpt over the wire, L1 0.050 vs demo) | **PASS 2026-08-12** |
| G0.5 | ML-free hardware checkout | `python -m rebot_core.policy_servers --mode sine --amplitude 0.05` + policy_runtime with the same manifest | needs a human at the rig |
| G1 | sim-episode replay | `policy_runtime --replay <ep>/arrays.npz --time-scale 0.25 --arm`, then 0.5x/1x; PASS = motion matches the sim video, no vel_clamps/gate trips, grasp lands | needs a human |
| G2 | shadow mode | serve the ckpt, log predicted vs demo actions on REAL observations (arm holding) | needs a human |
| G3 | camera match | overlay real frames vs sim renders; iterate mounts (wrist hand-eye is UNCALIBRATED) | needs a human |
| G4 | tethered rollout | manifest `max_joint_velocity: 1.0`, hand on power switch, <=5 episodes, recording on | needs a human |
| G5 | nominal rollouts | raise the ceiling only after G4 is clean; N>=20 episodes, MCAP everything | needs a human |

## Hazards (each one is a measured lesson, not caution boilerplate)

- **First-command gate**: the first target after `activate()` must be
  within 0.35 rad of the measured pose or the streamer self-deactivates.
  The bridge's ramp starts from the measured pose for exactly this reason.
- **Velocity clamps are silent lag**: demos contain wrist-flick peaks up
  to ~6 rad/s; anything over the ceilings is ramped, not executed — watch
  `vel_clamps` in both the streamer and bridge snapshots; a persistent
  count means the executed trajectory is NOT the inferred one.
- **Gripper force**: never stream raw gripper targets; the carrot
  primitives are the only force-capped path.
- **Motors latch on unclean shutdown**: the arm holds the last MIT
  setpoint at full kp with no host. One SIGINT; let safe-home finish.
- **One Isaac/FLUX/torn-GPU job at a time** on this machine (31 GB RAM,
  16 GB VRAM); the inference server + rebot_core coexist fine (the server
  is idle between requests).

## Insertion points into pre-existing files (complete list)

- `core/rebot_core/recorder.py` — three `/policy/*` topic strings added to
  `STRUCTURED_TOPICS` (channels exist on every episode, carry messages
  only when a bridge writes; count 0 during ordinary teleop, same as the
  unused-hand quest topics). That is the ONLY edit to existing code.
- `EpisodeRecorder.attachments_provider` is WRAPPED (instance attribute,
  restored on detach) to add the manifest attachment — no file change.

## Adding a new policy family

1. Write a server: subclass `PolicyServer`, implement `infer(arrays, info)
   -> ({"chunk": (N, action_dim) f32}, info)`. Run it in whatever env the
   model needs.
2. Write a manifest. If the observation layout needs a block the bridge
   does not know, add a builder to `policy_map.build_proprio` and its name
   to `KNOWN_PROPRIO_BLOCKS` — that is the designed extension seam.
3. `ee_pose` action space: decode to pose + `kinematics.ik` (seed =
   previous target, reject on non-convergence) — planned seam in
   `policy_map.decode_action`, unimplemented until a policy needs it.
4. Run the gates. G1 (replay) only applies to policies with recorded
   sim demos; G0.5 (sine) always applies.
