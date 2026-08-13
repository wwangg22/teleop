# Port worklog — ROS removal, desktop-hosted

Companion to `plan.md`. Newest entries at the bottom. Every phase gets: what
was done, what broke, how it was fixed, deviations from plan.

## 2026-08-08 — environment (pre-port)

- Miniconda + `teleop` env (py3.12): motorbridge==0.5.0 (x86_64 wheel exists —
  arch risk retired), pin 4.1.0, numpy 2.5.1, pyrealsense2 2.58.3, mcap,
  mcap-ros2-support, pycdr2, rerun-sdk 0.35, fastapi/uvicorn. NVIDIA driver
  595.84 (open) on the RTX 5080; no CUDA toolkit by design. can-utils + adb.
- Verified with no ROS: pinocchio loads the RS URDF (nq=8); vendor SDK imports.

## 2026-08-08 — Phase 0: rebotarm_core extraction — DONE

- `core/rebot_core/config.py`: port of `hardware_config.py`. ament_index
  removed; explicit `sdk_root` (defaults into `rebotarm_ros2/third_party/`),
  default model now **rs** (vendor default dm is the wrong arm). Kept: layered
  deep-merge precedence, `_runtime` gain resolution, tempfile yaml shim (Phase
  4 will remove), SDK model-cache seeding (the thing that makes gravity FF use
  the RS model). Added `resolved_urdf_path()` helper for Phase 1.
- `core/rebot_core/hardware.py`: `HardwareManager` ported byte-for-byte except
  the two ROS leaks: config import, and `current_pose()` now returns numpy
  `(pos3, quat_xyzw)` via a local Shepperd rotmat→quat (no tf_transformations).
- Verification (no hardware): constructs in the teleop env with zero ROS;
  `kinematics_model=00-arm-rs_asm-v3.urdf`, `kinematics_warning=None`,
  mode=mit, 6 joints + gripper. First try, no issues.
- Hardware note: PCAN-USB enumerates as **can0** on this desktop (Jetson's
  "always can1" is dead — channel is config now). Interface present, DOWN
  until `sudo ip link set can0 up type can bitrate 1000000`.

## 2026-08-08 — Phase 1: kinematics.py — DONE

- `core/rebot_core/kinematics.py`: reduced pinocchio model from the SDK RS
  URDF (prismatic gripper joints locked at neutral), EE frame `gripper_end`
  (MoveIt's `gripper_tcp` was a zero-offset alias of it on RS — verified in
  `rebotarm_rs.urdf.xacro`). DLS IK with Levenberg-style damping growth:
  near-singular targets converge slowly then FAIL tolerance instead of
  exploding, matching the reject-don't-explode contract. Limits = the QTR
  constants (RS one-sided joint2/3).
- `core/tests/test_kinematics.py`: 7 property tests, all pass first run —
  fk∘ik round trip, teleop step pattern (300 warm-seeded ~mm steps, <10%
  reject), unreachable reject, singular-home behavior, limit respect,
  determinism.
- `core/tools/kinematics_bench.py`: warm-seed (production pattern) mean
  0.038 ms, p99 0.047 ms, 2.6 iters — ~65× faster than MoveIt's 2.5 ms.
  Cold seeds solve 158/500 (expected for DLS; production always seeds warm,
  rejects hold). Episode-replay A/B deferred: no episodes on this machine
  yet (they live on the Jetson) — property tests cover the contract.

## 2026-08-08 — Phase 2: quest_link.py — code DONE, headset validation pending

- `core/rebot_core/quest_link.py`: ros_tcp_endpoint reimplemented (~450
  lines incl. docs). v0.7.0 framing, immediate `__handshake`, syscommands
  (`__publish`/`__subscribe`/`__topic_list` handled; service ones log+ignore),
  CDR via pycdr2 dataclasses, NUL-strip on syscommand JSON AND destinations,
  quaternion-norm reject, latest-value pose slots + 256-deep inputs ring,
  3 s sliding-window rate meters, newest-connection-wins (loud), binds
  0.0.0.0, haptics send stub.
- **Broke**: `from __future__ import annotations` silently breaks pycdr2 type
  resolution for builtin field types (str/bool) — errors like "Type str
  cannot be resolved". All-float64 messages still decoded, which made it look
  intermittent. Fix: no future-annotations import in that file (noted in its
  docstring).
- `core/tests/test_quest_link.py`: 12 checks, all pass — CDR bytes are built
  BY HAND in the test (independent of pycdr2) so encoder and decoder
  cross-check; covers app quirks, corruption reject, keepalives,
  reconnect/newest-wins, unknown syscommands.
- `core/tools/quest_link_dump.py`: live monitor for the real-headset bars
  (pose ≥50 Hz sustained, unit norms, clean edges, 30 min soak). NEEDS THE
  HEADSET — run when it's on this LAN, point the app at this machine's IP.

## 2026-08-08 — Phase 3: cameras.py + recorder.py — DONE (live-camera tested)

- The D405 wrist camera is attached to THIS desktop (serial 260522275150 =
  station.yaml's) — enumerated over USB 3.2 and captured live.
- `core/rebot_core/cameras.py`: pyrealsense2 pipeline per camera, global
  time enabled, latest-frame slot + drop-oldest recorder queue (drops
  counted), JPEG/depth-preview encode ON DEMAND only (Pillow — no OpenCV
  dependency needed). Live test: 30.1 Hz sustained, both streams, clean stop.
- `core/rebot_core/recorder.py`: in-process MCAP writer (ZSTD chunks).
  Ported intact: episode dir layout + collision suffixes, atomic info.json,
  stop reasons, bytes-ON-DISK counter + rate, squeeze-toggle hysteresis
  (RecordToggle, 0.8/0.3/1 s), params snapshot (config dump — the
  `ros2 param dump` timeout/pgrep hacks die), never-delete. New:
  `clean_close` flag in info.json replaces rosbag2's metadata.yaml
  convention. Encodings recorded in info.json: structured=json, color=jpeg,
  depth=raw z16 (~18 MB/s/camera raw; ZSTD in mcap crushed a 6.5 s
  110 MB depth stream into a 43 MB file).
- End-to-end test WITH the real camera: 6.5 s episode, 180/180 frames at
  30.0 Hz (max gap = one frame period), 296 joint-state msgs at 49.3 Hz,
  zero queue drops, clean close. `core/tools/episode_report.py` (counts/
  rates/gap detection/problem flags) reports OK.
- Deferred: old-Jetson-bag read check via mcap-ros2-support (no old bags on
  this machine yet — run when episodes are copied over).

## 2026-08-08 — Phase 4: single-process merge — code DONE, hardware pending

New modules (all `core/rebot_core/`):
- `stream.py` — SetpointStreamer: the chunk executor distilled. KEPT: Hermite
  sampling w/ analytic-derivative vff (ported verbatim), measured-dt velocity
  clamp capped 4× nominal, first-command 0.35 rad gate, stale→hold-never-
  coast, stand-down during SAFE_HOMING/TRAJ_RUNNING/GRAVITY_COMP, inactive-
  until-activate. DELETED: temporal ensembling (newest-covering-segment wins;
  buffer+append survives for future policies), ROS time (monotonic
  everywhere), all topic machinery. NEW: exclusive source ownership
  (teleop vs policy — one owner, ever).
- `state.py` — StateCache: replaces /joint_states. One reader thread, 50 Hz,
  bounded cmd_lock_window (2 ms) with skip-on-miss + skip counter; ~1 Hz
  temps piggybacked. The old ros_actions unlocked reads are NOT ported.
- `teleop.py` — QuestTeleopReal's control math ported whole: filter window +
  sign-aligned quat average, frame-jump two-strike, deadman/gripper edges,
  anchor resync, lag brake THAT KEEPS COMMANDING, cart clamp from commanded
  pose, orientation conjugation mapping + slerp limits, IK guard seeded from
  cmd_q, one-ratio velocity budget scaling, 3-point hold-tail segments with
  measured dt, gripper toggle flips on any COMPLETED call (worker thread —
  the old one blocked a subscription callback for up to 6 s), dry-run sim.
  tf_transformations → local numpy quat helpers. Dropped: two-driver graph
  check (structurally impossible now), service mode, ros2quest/legacy nodes.
- `runtime.py` — the process: assembly, ARM stdin confirmation, ready-pose
  ramp through the streamer, record-toggle mux on press_middle, 1 Hz
  diagnostics→recorder, SIGINT/SIGTERM→park then SIG_IGN (one-signal rule),
  kinematics mismatch now FATAL at startup (was a warning), CLI
  (`python -m rebot_core.runtime [--dry-run|--channel can0]`).
- Tests `core/tests/test_phase4.py` (25 checks, ALL PASS): streamer safety
  properties against a mock hardware + a full dry-run operator session
  driven by a fake Quest app over a real socket (engage/anchor/track ±drift/
  gripper/release-hold/re-anchor/frame-jump/watchdog).
  - Test-authoring note: with the trigger still held, the deadman re-engages
    immediately after a guard disengage — faithful to the ROS node; assert
    on counters, not transient engaged state.
- Runtime smoke test: assembles with the REAL camera + quest link + dry-run
  teleop, snapshots, clean shutdown.
- Deviations from plan.md, with reasons:
  1. The 20 Hz→direct-write idea is refined: targets go through the 100 Hz
     SetpointStreamer (Hermite), because direct 20 Hz writes into MIT are
     exactly the "20 Hz staircase" the executor was built to kill. The
     ensembling machinery is still deleted as planned.
  2. Dropped-CAN-write counter deferred: counting CallError swallows needs
     an SDK edit, which plan.md rule 1 forbids today. Observability instead:
     streamer write/clamp/overrun counters, StateCache skip counter, and
     external `ip -s link`/candump during the hardware bench.
- HARDWARE VALIDATION PENDING: can0 still DOWN (needs
  `sudo ip link set can0 up type can bitrate 1000000`). Bench ladder §7 of
  plan.md not yet run; motors not yet scanned from this machine.

## 2026-08-08 — Phase 5: GUI + launcher + viz — DONE (browser-tested)

- `core/gui/procman.py`: ported with ONLY the ROS sourcing removed; the
  SIGINT-parent-only / survive-restart / never-SIGKILL behaviors intact.
  Logs at ~/.rebot_station/logs/.
- `core/gui/server.py` + `web/index.html`: the dashboard now runs INSIDE the
  runtime process (uvicorn daemon thread, `--gui`, port 8800/8801) — the old
  GuiNode's subs/clients are direct method calls. Routes: status, recorder
  start/stop, MJPEG color+depth (encode-only-when-watched preserved),
  episode list, joint traces, bag playback (from MCAP). Path-traversal guard
  ported.
- `core/gui/episodes.py`: mcap-based episode browser (replaces bag_utils).
- `core/gui/launcher.py` + `units.yaml`: browser start/arm/stop. Units:
  can-check, motor-scan check (with a NEVER-while-runtime-up warning),
  runtime (confirm + stdin ARM box, singleton on the module path — same
  pgrep lesson as the Jetson), runtime-dry. Proxies the runtime dashboard.
- All of it exercised over HTTP against the live dry-run runtime + real
  camera: status, record start→stop through the API (real 28 MB episode),
  MJPEG bytes on the wire, joints trace, playback stream, 404 on
  path traversal, launcher start→adopt→stop cycle.
- `core/rebot_core/viz.py`: rerun.io live viz (`--rerun`): camera JPEG
  passthrough, FK EE trail, joints + lag scalars. API surface verified
  against rerun 0.35; the interactive viewer itself needs a desktop session
  — first spawn is on the operator.
- **Debugging detour worth remembering**: SIGINT appeared broken (runtime
  survived kill -INT). Root cause: my own test harness — `cd X && python &`
  backgrounds a bash SUBSHELL; the saved PID was bash, python was orphaned
  on SIGTERM, and the V4L device was left wedged (camera I/O errors until
  the orphan died). The runtime's actual signal path is correct: one SIGINT
  → "shutting down (further signals ignored)" → clean exit, ports released.
  Verified explicitly on the real PID.

## Remaining — blocked on operator/hardware, not code

1. `sudo ip link set can0 up type can bitrate 1000000` (needs password) —
   then: motor scan 7/7 → plan.md §7 bench ladder (state rates, CAN fps,
   gripper on air/block, dry-run→small→normal teleop, MockPolicy-equivalent,
   full recorded session).
2. ~~Quest headset validation~~ **DONE 2026-08-08 with the real headset**:
   app pointed at 10.0.0.210:10000, connected on first try through the
   launcher-started dry-run runtime. All six streams at **72.2 Hz**
   (README's 72 was right, ROS_MIGRATION's ~60 was wrong), **0 decode
   errors / 0 bad quaternions in 10k+ frames**, payload sizes match the
   hand-computed CDR layouts exactly (pose 76 B, twist 52 B, inputs 24 B).
   - Real-world protocol fix: the app sends topic names WITHOUT the leading
     slash (`q2r_right_hand_pose`); pre-registered defaults were
     slash-prefixed, so 6 pre-registration frames were dropped as unknown.
     quest_link now normalizes topic names both ways; after restart:
     unknown_topic=0. All test suites re-pass.
   - The app also `__subscribe`s to q2r_twist, dice_twist, and both haptic
     feedback topics — matches ros2quest.py's documentation of the return
     path; our stub covers it if haptics are ever wanted.
   - Operator-note: "already running elsewhere" from the launcher =
     singleton guard doing its job (a terminal-started runtime was still
     alive; two runtimes = two CAN masters on a live arm).
   - 30 min soak still outstanding; run during the first long dry session.
3. Old-Jetson episode read check when data is copied over.

## 2026-08-08 — FIRST LIVE ARM SESSIONS from this desktop + lag root cause

- can0 came UP; the operator drove the REAL ARM with the new stack twice
  (13:26 and 13:34): energize → ready ramp ("ready pose reached") → teleop
  engage/release cycles → clean park on stop ("safe home, then disable",
  9 s). No safety incidents.
- **Lag investigation** (the reported "extremely laggy, something hanging"):
  per-stage timers added to the teleop tick (`timing_ms` in diagnostics).
  Measured while engaged: total p50 0.63 ms (FK 0.07, IK 0.30) — nothing in
  the compute path hangs. Root causes found in the live logs instead:
  1. **Quest delivery gaps**: `quest_age` p95 62 ms / max 88 ms in dry run;
     on live runs 5× ">300 ms" bursts tripped the motion WATCHDOG →
     auto-disengage → operator experiences constant stalling. Desktop AND
     headset are both on WiFi; NetworkManager wifi powersave is at distro
     default (usually ON). FIX PENDING (operator): wire ethernet to the
     desktop, or disable wifi powersave.
  2. **StateCache lock starvation** (live only): "joint states stale -
     skipping" ×3 — 50 Hz reads with a 2 ms window lost against the 500 Hz
     MIT loop + 100 Hz streamer on the same RLock for >1 s. FIXED: 25 Hz,
     8 ms window, gripper reads decimated to ~5 Hz (they feed only
     GUI/recorder).
- **Health sentinel** (`sentinel.py`, per operator request "alert where it
  can actually hang"): 5 Hz checks with trip/clear logging (no spam), per-
  session incident summary at shutdown, alerts into the episode recorder
  (`/health/alerts`) and the status snapshot. Watchpoints chosen from
  hardware history: 500 Hz MIT loop liveness (>20 ms silent) + lock-skip
  ratio (>10%), streamer tick gap (>50 ms), state age (>300 ms), Quest pose
  inter-arrival gap (>150 ms — precursor of the 300 ms watchdog), teleop
  tick gap, gripper command in flight >6 s (wedged-motor tell), recorder
  bytes-flat-while-recording, and a GIL-starvation canary (10 ms sleeper
  measuring oversleep >20 ms). Instrumentation on the 500 Hz path is plain
  attribute writes. All suites re-pass; synthetic trip/clear test + dry-run
  smoke ("clean session, no incidents") green.
- Misc: dashboard poll 1 Hz → 4 Hz. `tools/collect.py` (episode labeling
  helper written on a misunderstanding) deleted at operator request — the
  recorder itself was never involved. Stray additive `task.json` left in
  test episode ep_133058 per the never-touch-episodes rule.

## 2026-08-08 — camera to 60 fps + streaming stats on the dashboard

- D405 mode enumeration: 60 AND 90 fps supported at 640x480 and 848x480
  (both z16 depth + rgb8 color); 1280x720 caps at 30.
- `station.yaml` profile 640x480x30 → **640x480x60** (the Jetson 60 MB/s
  disk budget that forced 30 is gone; measured ~14 MB/s to NVMe with JPEG
  color + ZSTD-chunked depth). Live capture verified: 59.9 Hz sustained,
  0 restarts. 848x480x60 or 90 fps are available if ever wanted.
- Dashboard: per-camera Hz/frames/drops/age panel + live "N Hz · N frames"
  captions on both stream views + active HEALTH alerts surfaced. Takes
  effect on next runtime start.

## 2026-08-08 — measured-Hz meters across the whole joint-data path

- Every hop now reports live measured Hz (3 s sliding window):
  teleop `send_hz` (accepted commands), streamer `write_hz` (hardware
  target writes, nominal 100), StateCache `state_hz` (successful reads,
  nominal 25), MIT `loop_hz` (nominal 500, from the cycle counter), and
  recorder `channel_hz` per stored topic. All in `/api/status` and rendered
  on the dashboard (stream/state + recorder panels).
- **Data-quality fix found while adding this**: joint states were recorded
  at only 1 Hz (piggybacked on the diagnostics loop). Now a StateCache
  `on_state` hook stores EVERY successful state sample (~25 Hz on live
  hardware) with both monotonic and wall stamps. Commanded targets were
  already stored per-send (20 Hz).
- Known minor: `recorder_stall` can trip transiently on very-low-traffic
  recordings (mcap buffers its first 1 MB chunk before anything hits disk);
  with cameras recording it flushes constantly and the check is meaningful.
- All suites re-pass. Needs a runtime restart to take effect (a dry-run
  runtime from before these changes is running as of this entry).

## 2026-08-08 — station/motor lifecycle split + data-management GUI

Per operator request ("8801 should work regardless; starting teleop should
only mean starting MOTOR CONTROL"):
- `runtime.py` restructured into two tiers. STATION (immediately on
  launch): QuestLink, cameras, recorder, GUI, sentinel, diagnostics, and a
  DRY teleop session (input math always live = axis calibration for free).
  MOTORS (on demand): `POST /api/motors/start {"confirm":"ARM"}` constructs
  the hardware layer on a worker thread, energizes, ready-ramps, and swaps
  the teleop session to live; `/api/motors/stop` parks and swaps back to
  dry. `--dry-run` now means motors are BLOCKED entirely. The terminal ARM
  stdin prompt is gone — arming happens in the GUI. States: off/starting/
  on/stopping, surfaced in the header with error reporting.
- New GUI (single page on the runtime port): motor start/stop with ARM
  input; live-data panels at 5 Hz (EE xyz + quaternion from FK, per-joint
  q/qd/effort/temps table, raw quest pose/trigger); depth probe (center
  distance mm + valid %); episode manager — list with per-episode joint-
  position plot (canvas), embedded video playback, info.json view, storage
  totals, and TRASH (move to demos/.trash — restorable rename; the
  never-delete rule holds: no code path destroys episodes).
- units.yaml: `runtime`/`runtime_dry` units replaced by `station` /
  `station_sim`; stdin ARM box no longer needed.
- New endpoints: /api/live, /api/motors/*, /api/storage,
  /api/episodes/{d}/{n}/detail, .../trash, /api/depth/{camera}.
- Validated: suites ALL PASS; station smoke exercised live/motors(blocked
  in dry-run)/storage/detail/trash end-to-end.
- `docs/DATA.md` written: data-longevity plan (dual-stamp semantics,
  self-describing episodes incl. URDF + intrinsics embedding, rate-change
  robustness via raw-stamps-only, checksums, migration policy, backup gap)
  with a 7-item implementation checklist AWAITING OPERATOR SIGN-OFF.

## 2026-08-08 — DATA.md checklist IMPLEMENTED (operator approved)

Episodes are now schema v1, self-describing, and integrity-checked. Detail:

1. **Schema + conventions in-band** (`recorder.py`): `SCHEMA_VERSION = 1`
   and a `CONVENTIONS` block (units rad/rad-s/Nm, z16→meters via
   depth_scale, quat order xyzw, frames base_link/gripper_end, joint order,
   clock semantics for log_time/t_wall/t_mono/t_device, and the alignment
   rule: "match on stamps, NEVER assume fixed dt") written into every
   info.json. This is the answer to "what if we change control freq":
   rates are metadata, per-sample stamps are truth.
2. **Embedded artifacts**: every episode.mcap now carries two MCAP
   attachments — the RS URDF (`urdf/00-arm-rs_asm-v3.urdf`) and the
   resolved hardware config yaml — so FK/IK and the hardware setup are
   reconstructible from the episode file alone. Recorder gained
   `attachments_provider` / `metadata_provider` hooks; runtime supplies
   them (`_episode_attachments` / `_episode_metadata`).
3. **Camera calibration captured live** (`cameras.py`): at pipeline start,
   per-stream intrinsics (fx/fy/ppx/ppy/model/coeffs), the device
   depth_scale (z16→meters), serial, and an explicit
   `extrinsics_cam_to_ee: "UNCALIBRATED"` marker → info.json
   `camera_calibration`. Hand-eye calibration remains TODO and is honestly
   labeled as absent.
4. **Provenance**: hostname, operator, stack id, git rev + dirty flag
   (repo verified to be a git checkout; rev recorded per episode).
5. **Integrity**: sha256 of episode.mcap computed at finalize into
   info.json; `episode_report.py` re-hashes and fails loudly on mismatch.
6. **Per-frame stamps sidecar**: new `/camera/<name>/stamps` JSON channel
   (seq, t_wall, t_mono, RealSense t_device per frame) — sub-frame-accurate
   video↔joints matching forever, independent of log_time policy.
7. **`tools/resample.py`** — THE one sanctioned resampler: uniform-grid
   export at any --hz from raw stamps (linear interp for q/qd/effort/
   targets with NO extrapolation, nearest camera frame by INDEX within a
   tolerance, NaN/-1 for uncovered points), written ADDITIVELY to
   `episode/derived/aligned_<hz>hz.npz` + manifest carrying the source
   checksum. Frames referenced by index, decoded from the mcap on demand.
8. **`episode_report.py` gate extended**: checksum verify, per-channel
   log_time monotonicity, wall-vs-mono skew drift (>0.5 s flags a clock
   step mid-episode), schema-version known, attachments present for v1.

## 2026-08-08 — "struggling to keep up" session analyzed → CAN-read overhaul

Session 14:11–14:14 (station.log), sentinel verdict, three distinct causes:
1. **mit_loop_skips 11–31% repeatedly** — every state/gripper/temps poll did
   sequential CAN round trips while holding the command lock; each ms held
   = a skipped 500 Hz cycle = chatter/sluggishness while driving.
2. **One ~1 s whole-stack freeze** (14:13:26: mit_loop 1001 ms silent +
   stream 996 ms + state 1038 ms, simultaneously) — the documented
   motorbridge eaten-reply stall: a request/response read lost its reply
   and blocked for the SDK's 1 s timeout while holding the lock.
3. **quest_gaps**: many 150–520 ms (WiFi; several watchdog disengages) and
   two ~5 s outages during the session tail — pattern consistent with the
   headset being taken off (app pauses streaming on the proximity sensor),
   not diagnosable further from our side. Ethernet/powersave advice stands.

FIX (structural, addresses 1 AND 2): the 500 Hz MIT stream's feedback
frames already refresh the SDK MotorState cache continuously — so
StateCache now reads CACHED state (`get_joint_state(request=False)`,
`get_gripper_state(request=False)` via the motor map): **zero CAN traffic,
microsecond lock holds** at 25 Hz. The historical "cached position
freezes" trap (observed on the Jetson with NO active MIT stream) is
guarded by a requested-read cross-check every ~4 s: divergence >0.05 rad
logs loudly, counts (`verify_failures` in snapshot), and substitutes the
requested values. Temps stay requested (params aren't in MIT feedback)
but decimated to ~2 s. Gripper CONTROL paths (grasp ramp, reached-target)
still use authoritative param reads — only telemetry went cached.
Also: recorder_stall false positive at recording start fixed (5 s grace
for mcap's first-chunk buffering). Suites re-pass.
VALIDATE NEXT LIVE SESSION: expect mit_loop_skips ≈0 and no 1 s stalls;
watch `verify_diff_rad` (should sit <0.01) and `verify_failures` (must
stay 0 — nonzero means the cached-read assumption is wrong on this
firmware and we revert to requested reads at low rate).

## 2026-08-08 — "still freezes" follow-up: STALE PROCESS + replay-burst bug

Operator reported freezes persisting. Two findings:

1. **The CAN-read overhaul never ran.** The station process (started
   14:10:44) predates it and was never restarted — GUI motor start/stop
   does NOT reload code. All mit_loop_skips/state_stale trips in the
   "new" session are the old build. OPERATOR PROCEDURE: restart the
   station unit after code changes (launcher stop → start).
2. **New bug caught by the logs — TCP backlog replay.** The two ~5 s
   quest gaps were WiFi stalls during which the app's messages queued in
   the socket; on recovery the kernel flushed SECONDS of stale input in a
   burst: replayed squeezes fired the record toggle twice in one second
   (stop ep_141245 + start ep_141339), a third replayed toggle raced a
   stop transition and crashed mcap finalize ("'NoneType' has no attribute
   'finish'"), and replayed poses tripped the frame-jump guard at 8.6 m/s
   (the guard doing its job on garbage input). A 106 s gap at 14:19 =
   headset off.

Fixes (all tested):
- **Recorder transitions race-safe**: a non-blocking `_transition_lock`
  serializes whole start/stop transitions (the inner state lock must be
  released to join writer threads — that window was the race). Concurrent
  callers get "start/stop already in progress".
- **Replay-burst suppression in quest_link**: >30 pose arrivals of one
  hand inside 200 ms (=150 Hz sustained vs the real 72) marks a backlog
  flush; consumer hooks (teleop + record toggle) are suppressed for its
  duration while latest-value slots keep updating for the GUI. Counted as
  `replay_suppressed`, logged throttled. Test: 300-message flush → 270
  suppressed, slots fresh, only a normal-window's worth leaked.
- **Edge-detector resync after gaps**: first inputs sample after a
  >watchdog gap resyncs the gripper-button edge detector (no phantom
  toggle from a button that changed during the outage); the record toggle
  gets an explicit `resync()` on >0.3 s gaps (a squeeze held across the
  gap must not fire on reconnect).
All suites + two new targeted tests PASS. RESTART THE STATION to load
everything since 14:10 (CAN overhaul + these).

## 2026-08-08 — periodic disengage every ~4 s: WiFi roam-scan diagnosis

Operator: "robot disengages every 3-4 s while holding the controller."
Log forensics (15:37:31–15:38:08): WATCHDOG disengages at metronomic 4 s
intervals, each a genuine ~300–350 ms transport outage (pose AND inputs
stale together). 123 quest_gaps trips in the session; inter-trip spacing
histogram: 29 of 38 gaps are EXACTLY 4 s. Signature of periodic
off-channel WiFi scanning on the desktop adapter. The NM connection
(Blueground308) has no pinned BSSID and no band lock → roam scans free to
run. Watchdog threshold (0.3 s) deliberately NOT raised — moving the arm
on half-second-old hand data is not a fix, it's a hazard.
Remedies (operator action, in order of certainty): 1) ethernet;
2) pin BSSID + disable powersave via nmcli (commands given in chat);
3) both devices on 5 GHz. NOTE: station STILL not restarted since 14:10 —
none of today's fixes (CAN overhaul, replay suppression) have run yet.

## 2026-08-08 — 4 s disengage FINAL diagnosis: Quest app reconnect loop

Chased across three layers; each ruled out with data:
- WiFi radio: BSSID pinned + powersave off, VERIFIED APPLIED; parallel
  40 s ping test (5 Hz) during the symptom: gateway 200/200, Quest
  199/200 — the network is CLEAN. Not radio.
- Our endpoint: reference ros_tcp_endpoint sends nothing on registration
  either — no missing server response.
- THE CAUSE: the quest2ros app is in a client-side reconnect loop —
  connections counter 73→201 within the session, ~30 connects/min
  (one per ~2 s, machine-like), each new socket opened WHILE the old one
  lives (our newest-wins churns them; EBADF on the replaced reader is
  the expected teardown). Onset timestamped at 15:38, MID-SESSION with
  zero server-side change (old build had run since 14:10), immediately
  after a ~100 s headset-off gap — the app resumed from headset doze in
  a broken state and its reconnect timer never recovered. Earlier same-
  day sessions (13:25, 14:12) held ONE connection for many minutes.
- FIX: restart the quest2ros app on the headset (fully quit + relaunch)
  or reboot the headset. No server-side change can stop a client-side
  reconnect loop. Live check after: `connections` counter in
  /api/status must stop increasing.

## 2026-08-08 — Quest reconnect loop CONFIRMED FIXED by app restart;
## motors-not-starting diagnosed + preflight scan added

- Operator restarted the quest2ros app: connections counter stable at 1.
  The 4 s disengage saga is closed (root causes over the day: WiFi
  roam-scan → pinned; app reconnect loop → app restart).
- Motors failing to start, log shows the stack: (1) can0 flapped
  ("Network is down" during 16:05 attempt); (2) failed attempts left a
  motor wedged ("gripper did not enter mit mode" ×3 — the documented
  only-power-cycle-clears state); (3) the 16:12 attempt HUNG in connect
  on the partial bus, motors_state stuck "starting" (no retry possible
  without a station restart).
- Code fix: `_preflight_scan()` — motorbridge scan (15 s timeout) runs
  BEFORE HardwareManager construction; != 7/7 motors → fail fast with the
  power-cycle instruction instead of hanging. The Jetson runbook's
  "always scan first" rule is now enforced mechanically.
- Operator procedure for the current wedged state: 1) support the arm +
  POWER-CYCLE it, wait ~15 s; 2) restart the station (clears the hung
  start worker AND loads the preflight scan); 3) start motors from the
  GUI — the scan now gates it.

## 2026-08-08 — ARM typing requirement removed (operator request)

- `require_confirmation` now defaults False; `--require-arm` flag exists
  for anyone who wants the old behavior back. GUI: the typed-ARM input is
  gone; "start motors" shows a single one-click browser confirm dialog
  (misclick guard only). API accepts an empty body. Verified: empty-body
  /api/motors/start passes the confirm layer (blocked only by --dry-run in
  the test runtime, as expected).

Validated end-to-end on a real recording (ep_140850, camera at 60 fps):
report OK with checksum + attachments; resample --hz 20 → 160 grid
points, 160/160 frames matched (worst frame-to-grid offset 10.6 ms,
tol 25 ms), no NaN inside the interpolation range. All three test suites
re-pass. Remaining from DATA.md, deliberately open: off-machine backup
target (BIGGEST real risk — operator decision needed), hand-eye
calibration, LeRobot export (consumes resampler output).

## 2026-08-12 — policy-rollout infra (generic, manifest-driven) — offline-DONE

- New: `core/rebot_core/policy_{manifest,proto,map,bridge,thermal,servers,
  runtime}.py`, `core/tools/solve_joint_map.py`, tests
  `core/tests/test_policy_infra.py`, docs `docs/POLICY_ROLLOUT.md`. One
  edit to existing code: three `/policy/*` topics in
  `recorder.STRUCTURED_TOPICS` (inert-by-default channels). Design plan:
  `sim2real/notes/irl_rollout_plan.md` (IsaacLab side).
- Deploys ANY trained policy via three contracts: policy.yaml manifest
  (fail-closed validation), stdlib unix-socket inference protocol (torch
  in its OWN process — GIL), fixed MCAP channels + manifest attachment.
  Teleop released / stream re-owned as "policy"; 3-point hold-tail
  segments; replan-early chunking; gripper events FREEZE the policy clock
  through the blocking carrot grasp then replan (real close is 1-6 s vs
  0.4 s in the demos). Thermal protective stop ported from the ROS
  thermal_guard (60/75 C x3, fail-permissive, safe_home-then-disable),
  bridge-opt-in only.
- ⭐ The Isaac training URDF vs our RS URDF: every revolute axis OPPOSITE.
  Map solved numerically (solve_joint_map.py, frame-invariant signature):
  q_rig = -q_sim on all six joints, residual 1.7e-5 m. Manifest carries
  it; G1 replay at 0.25x is the hardware confirmation.
- Verified offline (no hardware, no GPU): 36/36 checks — manifest sad
  paths, decode/encode identity over a real 667-step demo, wire fidelity,
  50 Hz bridge cadence vs MockHW+SineServer (no gate trips, hold tails),
  gripper freeze/replan FSM, thermal trip order + latch. Real checkpoint
  served on CPU: (50,7) chunk, L1 0.050 vs the demo's ground truth ==
  the sim-side open-loop figure. Demo replay decode: 0 limit violations,
  mock-stream tracking err 4e-4 rad.
- Hardware gates G0.5-G5 (sine on rig, 0.25x replay, shadow, camera
  overlay, tethered, nominal) are documented in POLICY_ROLLOUT.md and
  need a human at the rig. Wrist hand-eye is still UNCALIBRATED; demo
  wrist flicks reach ~6 rad/s and will clamp at the ceilings (visible in
  vel_clamps, not dangerous).
