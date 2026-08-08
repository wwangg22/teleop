# Session Handoff — reBot Arm B601-RS on Jetson Orin Nano

**Read `~/rebot-setup/README.md` first** — it holds the durable knowledge
(hardware facts, safety behaviours, control-loop internals, local repo
modifications, debugging lessons, and how the policy executor works).
**Caveat:** parts of that README predate the Aug 7 findings below; where they
disagree, this file and `~/Desktop/teleop/README.md` §6–§8 are current.

This file covers only **live session state** and **what to do next**.
Last updated: **2026-08-07** (after the teleop jitter root-cause investigation
and fixes).

---

## Live state (2026-08-07)

Nothing arm-related runs from systemd/autostart — verify with `pgrep`, don't
trust this table blindly:

| Process | State | Note |
|---|---|---|
| `reBotArmController` | **STOPPED** | last run ended ~01:40 after parking |
| `move_group`, `robot_state_publisher` | running | MoveIt for the teleop IK guard |
| `ros_tcp_endpoint` | running | Quest link |
| `rviz2`, `TemperatureDisplay`, `HomeButton`, `MockPolicy` | stopped | |
| `ws_gateway` / motorbridge-studio | stopped | binary still installed in `~/.local/bin` — **check `pgrep -af motorbridge` before every driver launch**; a second CAN master is a real hazard |

- Arm parked at zero and disabled by the teleop's park-on-exit.
- `can1` (PCAN-USB) UP, 1 Mbit/s, ERROR-ACTIVE, zero error frames.
  `can0` is DOWN and unused — **always launch with `channel:=can1`**; the YAML
  default is can0.

---

## Aug 7: teleop jitter root causes — found and fixed (unverified on hardware)

Full analysis in `~/Desktop/teleop/README.md` §7. Summary of what was actually
wrong (all verified in code + logs) and what changed:

1. **The "500 Hz" MIT loop ran at 50–160 Hz.** Its gravity feedforward called
   `get_positions()` every tick = six synchronous 0x7019 CAN round-trips inside
   the loop body, holding `_cmd_lock` throughout.
   *Fixed:* gravity is now evaluated at `_q_target` with a cache
   (`third_party/.../controllers/rebotarm_endpose_controller.py`). Removes
   ~3000 frames/s of CAN traffic (bus load ~80% → ~45%).
2. **Chunk executor velocity ceiling collapsed under contention** — its
   per-tick step clamp budgeted *nominal* dt on ticks that ran late.
   *Fixed:* clamp budgets measured elapsed time, capped at 4× nominal
   (`chunk_executor.py`).
3. **Ensembling blended live chunks 50/50 against stale hold tails**
   (`m=0.01 favor=oldest` with only 2 overlapping chunks), snapping at chunk
   expiry — a 20 Hz staircase into kp=150.
   *Fixed:* defaults now `m=5.0 favor=newest` (`rebotarm_controller.py`).
4. **Joint-state publisher raced the control loop on the CAN bus** (reads
   outside `_cmd_lock`, reentrant timer) — could stall the loop up to the SDK's
   1 s call timeout (observed as 1 s `joint_states` gaps).
   *Fixed:* all its CAN reads run inside a bounded-wait `cmd_lock_window()`,
   skipping the cycle on timeout; both 100 Hz timers moved to
   mutually-exclusive callback groups (`ros_publishers.py`,
   `hardware_manager.py`, `chunk_executor.py`).
5. **Teleop/executor standoff**: at 0.25 rad lag the teleop went silent, the
   executor fell back to holding its own short target, lag froze until
   re-squeeze. *Fixed:* teleop now publishes a hold chunk at `cmd_q` while
   lag-holding, plus measured-dt velocity feedforward, loop-overrun logging,
   and 1 Hz `~/diagnostics` (`QuestTeleopReal.py`).
6. **Gripper crushed whatever it grasped** (found later on Aug 7): close was a
   pure MIT spring to the mechanical limit — kp × (remaining stroke), saturated
   at the rs-00's **14 Nm** MIT frame ceiling, held indefinitely; nothing writes
   `limit_cur`/`limit_torque`; and a successful grasp reported as `timeout`,
   desyncing the teleop's toggle so the button could never send OPEN while
   holding an object.
   *Fixed:* `close_gripper_grasp()` — ramped target (3 rad/s), stall detection
   (0.01 rad / 0.15 s while ramp ≥0.1 rad ahead), re-anchor to
   `hold_torque/kp` past the stall point → steady squeeze ~2 Nm; grasp returns
   success ("grasped object at X rad"); gripper kp/kd overridden 50/4 → 20/2;
   six `gripper_*` params live-tunable via `ros2 param set`; `safe_home()`
   inherits the bounded close; teleop flips its toggle on any completed service
   call (`hardware_manager.py`, `ros_services.py`, `rebotarm_controller.py`,
   `QuestTeleopReal.py`). Tuning guide: README_ARM §6b / teleop README §3.

**REFUTED — do not re-chase:** the "DM vs RS gravity URDF mix-up" theory.
`resolve_hardware_config()` seeds the SDK model cache with the RS urdf_path on
every driver start (verify in `/tmp/rebotarm_ros2_*/rs_hardware.yaml`). The
`rebotarm.yaml` edit was a no-op for the ROS driver, and the
`_verify_kinematics_model` startup check compares the cache to itself (always
green, proves nothing).

---

## Commit state

The original bring-up work **is committed**: `~/rebotarm_ros2` at `60641f1`
(branch `jetson-setup-and-policy-executor`), `third_party/reBotArm_control_py`
at `1e9d0bf` (branch `expose-motor-temperature`) — matching the patches in
`~/rebot-arm-private/patches/`.

**Uncommitted working-tree changes** (protect these — includes all Aug 7 fixes):

```
~/rebotarm_ros2:
  src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py
  src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py
  src/rebotarmcontroller/rebotarmcontroller/ros_actions.py
  src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py
  src/rebotarmcontroller/rebotarmcontroller/ros_services.py
  src/rebotarmcontroller/rebotarmcontroller/chunk_executor.py
~/rebotarm_ros2/third_party/reBotArm_control_py:
  config/rebotarm.yaml                      (hardware_yaml dm→rs; harmless no-op for ROS)
  reBotArm_control_py/controllers/rebotarm_endpose_controller.py
```

Both workspaces are **symlink installs** — edits are live without rebuild.

---

## Next steps

1. **Hardware-verify the Aug 7 fixes** (none have run on the arm yet):
   - launch driver (`model:=rs channel:=can1`), confirm
     `ros2 topic hz /rebotarm/joint_states` is steady with no 1 s gaps;
   - CAN frame rate should drop to ~3500 fps (was ~4900);
   - `MockPolicy` small-amplitude on joint 0 → smooth, no 20 Hz stepping;
   - teleop: dry-run → translation-only small-scale → normal. Watch
     `/quest_teleop_real/diagnostics`: `lag` must recover without re-squeezing,
     `lag_hold`/`overrun` near zero;
   - gripper: `ros2 service call /rebotarm/gripper/close ...` on air →
     "complete"; on a wooden block → "grasped object at X rad", gripper effort
     in `joint_states` settles ≈ `gripper_hold_torque` (2 Nm), temp flat over a
     2-min hold, open releases; teleop button cycles
     OPEN→CLOSE(grasp)→OPEN→CLOSE(air) without desyncing.
2. **Commit both repos and regenerate the patches** in
   `~/rebot-arm-private/patches/` (procedure in its README).
3. **Fix `driver_params.yaml` not being loaded** — still true; every knob in it
   is inert. Pass params as launch args or wire the YAML into
   `driver.launch.py`.
4. **Fix velocity feedback** — `get_joint_velocities()` still returns the
   unreliable cached `MotorState.vel` (±0.1 rad/s on a stationary arm). Fix
   before collecting policy training data.
5. **Calibrate gravity FF for RS** — the hardcoded `1.55×` on joints 2/3 is an
   uncalibrated torque bias; make it configurable (wire `tau_scale` into the
   endpos path) and tune against measured droop.
6. **Verify thermal thresholds** against the RobStride rs-06/rs-00 datasheet —
   140/167 °F are placeholders and joint3 already hit 129 °F just holding.
7. **Settle Quest orientation handedness** — ±90°-per-axis OrientationViz test;
   see teleop README §3 "KNOWN LIMIT". `hand_axes_rpy` cannot fix a reflection.

Lower priority: persist CAN setup at boot (systemd), raise `txqueuelen` on can1
(currently 10) and use `restart-ms 100` (only `reset-can.sh` sets it),
performance CPU governor / `jetson_clocks` while driving, observation side for
policy training, software e-stop, upstream the `hardware.launch.py` import fix.

---

## The one thing to remember

**If the arm goes limp but still responds on CAN: POWER-CYCLE THE ARM before
touching software.**

This exact state — 7/7 motors scanning, live position, `fault 0x0`, 48.3 V bus,
`run_mode 0`, `enable()` succeeding, and **zero torque** — once defeated four
independent software stacks (motorbridge-studio, stock upstream ROS driver,
modified driver, raw motorbridge). It was a latched motor state that only power
removal clears, and it advertised nothing in any readable register. Power off,
wait ~15 s, power on. It worked immediately.

Hours were lost suspecting the code, then the power supply. Neither was at fault.
