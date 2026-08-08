# Plan: migrating this stack off ROS

Written 2026-08-08 by the agent that debugged and rebuilt this stack on
hardware, for the agent that will refine and execute it. Everything below is
grounded in the code as pushed — verify against the code, not against
vendor docs or old comments.

## Why this is unusually feasible

The layer that actually controls the arm is **already ROS-free**:
`rebotarm_ros2/third_party/reBotArm_control_py` is plain Python —
`motorbridge` (Rust CAN, pip) + `pinocchio` + numpy. The 500 Hz MIT loop,
gravity feedforward, and the grasp-aware gripper live there and in
`rebotarmcontroller/hardware_manager.py` (which imports nothing from rclpy —
check: its ROS surface is only that the *node* wraps it). ROS provides four
separable things today:

1. **IK/FK** — MoveIt `/compute_ik` + `/compute_fk` (teleop command path, 20 Hz)
2. **Quest input** — the quest2ros headset app → `ros_tcp_endpoint` → topics
3. **Inter-process transport** — driver ↔ teleop ↔ recorder ↔ GUI over DDS
4. **Recording/tooling** — rosbag2, RViz, robot_state_publisher

Target end state: **one Python process** (control + teleop + recorder core)
+ the FastAPI GUI, talking to CAN and cameras directly, writing MCAP/LeRobot
episodes. No rclpy anywhere.

## Behavioral invariants — port these, do not rediscover them

These encode hardware lessons. Losing any of them re-breaks the robot:

- **Safe-home on every exit path**; motors LATCH torque on unclean shutdown
  (no process needed — motor-internal). One SIGINT only; further signals
  ignored once shutdown starts. If a start later fails with "gripper did
  not enter mit mode" → wedged motor → power-cycle the arm (support it).
- **Never two control loops on the bus.** MIT mode has no firmware velocity
  ceiling; two writers = violent thrash.
- **Serialize ALL CAN access.** motorbridge has no internal locking; a
  concurrent request/response read can eat the control loop's reply frame
  (1 s stall). In one process this becomes a simple mutex — this is a major
  reason the merge is worth it.
- Gravity FF evaluated at the *target* (cached, 0.02 rad threshold), not
  via per-tick position reads (that bug capped the "500 Hz" loop at 50–160 Hz).
- Teleop guards: deadman (trigger), watchdog (0.3 s), per-step Cartesian
  clamp, `max_ik_jump` reject, measured-lag brake-instead-of-silence,
  frame-jump auto-disengage (2 m/s), joint-limit check. Velocity budgets:
  teleop 1.5 rad/s, executor clamp 2.0 rad/s with **measured**-dt budgeting.
- Gripper: ramped close (3 rad/s), stall detect (0.01 rad / 0.15 s, ramp
  ≥0.1 rad ahead), re-anchor to `hold_torque/kp` (~2 Nm). A stall on an
  object is a SUCCESS. kp override 20 (yaml ships 50).
- Recording: nothing is ever auto-deleted; stop reasons are recorded, not
  acted on. Disk budget (Jetson eMMC): 60 MB/s sustained — one 640×480×30
  RGB-D camera uncompressed (~46 MB/s) fits; 848×480×30 (~61 MB/s) thrashes
  the whole machine (DDS delivered inputs minutes late).

## Phases — each independently shippable, ROS path kept until validated

### Phase 1: replace MoveIt with pinocchio IK/FK  (small, do first)

- MoveIt is used as a numerical IK/FK service only: no planning, no
  execution, collision checking already OFF (`avoid_collisions: False` —
  false self-collision positives from the gripper value; the real guards
  are joint-limit + jump checks in teleop code).
- Write `q2r2_bringup/kinematics.py`: damped least squares on pinocchio
  Jacobians, URDF = the RS one the SDK already loads, joint-limit clamp,
  convergence + residual report. Seed from current measured q (teleop
  already passes it).
- Wire behind the existing `_ik()` / FK call sites with a param
  `ik_backend: moveit|pinocchio` for **live A/B on hardware**. Compare:
  solution agreement near workspace center, behavior near singularities
  (reject, don't explode — `max_ik_jump` stays), rate (MoveIt round trip
  was 2.5 ms; expect <0.5 ms in-process).
- Exit: teleop runs a full session on `pinocchio` backend; MoveIt launch no
  longer started. RViz ghost: keep `robot_state_publisher` (standalone)
  until Phase 5's viz replacement.

### Phase 2: Quest input without ROS

Today: quest2ros Unity app → `ros_tcp_endpoint` (TCP, CDR-serialized ROS
msgs) → `/q2r_*` topics at ~60 Hz. Two viable routes; prototype both, pick one:

- **(a) oculus_reader-style** (rail-berkeley): read controller poses +
  buttons over ADB directly. Kills the app, the endpoint, AND the
  head-relative-frame problem at the source (today the app's frame follows
  the headset until A+B is pressed — a whole class of operator error).
  Verify: pose rate ≥50 Hz stable over USB/WiFi ADB, grip analog available
  (today's record toggle is `press_middle` > 0.8 with 0.3 hysteresis).
- **(b) speak the app's protocol without ROS**: reimplement the TCP
  endpoint as ~300 lines (length-prefixed messages, CDR payload — parse
  with `pycdr2` or hand-rolled; struct layouts are trivial). Known traps
  already fixed once in `ros_tcp_communication` (§8 of teleop README): the
  app NUL-terminates its syscommand JSON; CDR has a 4-byte encapsulation
  header and field alignment — a naive parser yields 1e152 garbage.
  Sanity check: received quaternions must be unit-norm.

### Phase 3: single control process (the big one)

Merge teleop math into the driver process; ROS leaves the control path:

- One process: 500 Hz MIT thread (SDK loop as-is) + 20 Hz teleop tick +
  recorder hooks. The chunk executor's Hermite upsampling/ensembling exists
  ONLY because commands crossed a process boundary — in-process, the 20 Hz
  tick writes targets directly under the (now trivial) mutex; keep the
  velocity clamp + measured-dt logic, delete the ensembling machinery.
- The `_cmd_lock` saga (bounded lock windows, skipped publisher cycles,
  callback groups) collapses into one mutex with one reader thread for
  state at ~50 Hz.
- Feature-flag it: keep the ROS node entry point working (thin wrapper over
  the same core lib) until the new path has run a full demo-collection
  session. Structure suggestion: extract `rebotarm_core/` (SDK wrapper +
  guards + gripper + teleop math, zero rclpy imports) consumed by both.

### Phase 4: recording + cameras without ROS

- Cameras: `pyrealsense2` directly (device serials in station.yaml carry
  over; enable global time). Frames → in-process queues.
- Episodes: write **MCAP directly** (pip `mcap`; it is not a ROS format) or
  LeRobot dataset format if training is already LeRobot-shaped. Keep: one
  dir per episode, info.json with stop reason, param snapshot, never-delete.
  Keep raw stamps per stream; align offline only.
- Respect the disk budget above; the bytes-on-disk status counter (proof of
  writing) is worth porting as-is.

### Phase 5: GUI + viz

- The GUI is already FastAPI + vanilla JS; its rclpy node becomes direct
  calls into the core process (or a small IPC if kept separate). procman
  (start/stop/adopt, SIGINT-parent-only, no-kill-on-GUI-restart) ports
  unchanged — it manages plain processes, nothing ROS about it.
- Viz: rerun.io covers RGB-D + 3D robot pose + plots (feed it FK from
  pinocchio). This retires RViz and the last reason for
  robot_state_publisher.

## Suggested order & effort (refine, don't trust blindly)

| Phase | Effort | Hardware time | Risk |
|---|---|---|---|
| 1 IK | ~1 day | half day A/B | low |
| 2 Quest | ~1 week (route a) | 1 day | medium — input device quirks |
| 4 Recording | 2–3 days | half day | low |
| 3 Merge | 1–2 weeks | 2–3 days careful | highest — do LAST of the code phases, after 1/2/4 shrank the surface |
| 5 GUI/viz | 2–3 days | none | low |

(4 before 3 is deliberate: a ROS-free recorder can tap ROS topics via a thin
bridge meanwhile, and the merge then lands into an already-proven sink.)

## Traps that will waste your days if forgotten

- Vendor docs and old comments lie; the code and `teleop/README.md` §6–§7 +
  `docs/ARM.md` are the record. §7 lists REFUTED theories — do not re-chase
  the DM-URDF gravity theory, "frozen joint_states", or MoveIt latency.
- `model:=rs` everywhere while ROS still runs; `can1` not can0.
- rosbag2 Humble: topics + `--regex` together silently records nothing
  (matters until Phase 4 ships).
- The driver finds the SDK by walking up to a dir containing
  `third_party/reBotArm_control_py` — keep that layout, or replace the
  mechanism in `hardware_config.py` when restructuring.
- Jetson: 2 CPU cores parked at 729 MHz by default; Python threads are
  SCHED_OTHER. After Phase 3, consider pinning the control thread and
  measuring jitter before blaming the code.
- Watch `motorbridge` wheels per-arch (0.5.0 validated on arm64; check
  x86_64 before promising desktop-driven control).

## Validation protocol (every phase)

1. Dry-run equivalent first (no hardware writes) — teleop has `dry_run`;
   preserve that capability in the new core.
2. Bench: arm powered, holding still — state rates steady, no stalls, CAN
   frame rate sane (~3500 fps steady-state).
3. Slow teleop session: translation-only, small scale, then normal. Watch
   the diagnostics counters (port them): `lag` recovering without
   re-squeeze, `vel_clamp` reasonable, `overrun≈0`, `frame_jump` rare.
4. Full demo-collection session including gripper grasp cycles and episode
   recording at budgeted disk rate.
5. Only then delete the replaced path.
