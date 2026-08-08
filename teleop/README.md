# Quest → reBot B601-RS Teleoperation

Operating notes for **this machine** (Jetson `william`, ROS 2 Humble). Covers running
Meta Quest teleoperation against the reBot B601-**RS** arm, plus the local fixes that
make it work.

Two workspaces are involved:

| Path | Contains |
|---|---|
| `~/Desktop/teleop` | this workspace — `q2r2_bringup` (teleop nodes), `quest2ros` (msgs), `ros_tcp_endpoint` |
| `~/rebotarm_ros2` | the robot — `rebotarmcontroller` driver, MoveIt config, URDFs |

---

## 1. Source your shell

Every terminal needs all three, in this order:

```bash
source /opt/ros/humble/setup.bash
source ~/rebotarm_ros2/install/setup.bash
source ~/Desktop/teleop/install/setup.bash
```

Put them in `~/.bashrc` to stop thinking about it. After a `colcon build` that adds a
**new** package, re-source rather than reusing an old shell.

---

## 2. Run it

> **`model:=rs` on every launch.** The repo defaults to `dm`, which is the wrong
> kinematics *and* has sign-flipped joint limits for this arm. See §5.

### Terminal 0 — CAN (once per boot)

```bash
sudo ip link set can1 up type can bitrate 1000000
ip -br link show can1          # expect UP
```

`can1` is the PCAN USB adapter the arm is wired to. `can0` is the Jetson's built-in
mttcan controller and is **not** used. RS talks SocketCAN — there is no `/dev/ttyACM*`.

### Terminal 1 — robot driver

```bash
ros2 launch rebotarm_bringup driver.launch.py model:=rs channel:=can1
```

**Launch this exactly once, and only when you are about to use the arm.**

- It **energises the motors immediately.** `HardwareManager.connect()` calls
  `_start_endpos_loop()`, which puts the joints in MIT mode, calls `enable()` on the group
  and sets `_enabled = True` — before any `/rebotarm/enable` call. The arm holds whatever
  pose it read at connect time, drawing current the whole time (see §4).
- **Never run two instances.** Each opens its own 500 Hz MIT loop on the same CAN bus with
  its own `_q_target`. Two of them fight over the same motor IDs — one drives to a new
  target while the other keeps commanding the pose it last latched, at `kp=150` with no
  velocity ceiling. The arm thrashes violently with nobody touching the controller. See §7.

Before launching, confirm nothing is already up:

```bash
pgrep -af reBotArmController      # expect no output
```

### Terminal 2 — MoveIt + RViz

```bash
ros2 launch rebotarm_moveit_config hardware.launch.py model:=rs
```

Required: teleop uses `/compute_ik` from `move_group` as its safety guard. This launch
starts no `ros2_control` — the driver owns the hardware.

### Terminal 3 — Quest link

```bash
ros2 launch ros_tcp_endpoint endpoint.py
```

Then in the headset's Quest2ROS app: **IP `10.0.0.184`**, **port `10000`**, press
**Apply**. (Re-check the IP after a network change — DHCP can move it. It is hard-coded
as `ROS_IP` in `src/ros_tcp_communication/launch/endpoint.py`.)

Verify:

```bash
ros2 run q2r2_bringup CheckTCPconnection      # prints all six topics at 1 Hz
ros2 run q2r2_bringup OrientationViz          # then open http://localhost:8080
```

### Terminal 4 — teleop

```bash
# ALWAYS dry-run first. Sends nothing to the hardware, runs every check.
ros2 run q2r2_bringup QuestTeleopReal --ros-args -p dry_run:=true

# for real:
ros2 run q2r2_bringup QuestTeleopReal
```

Type `ARM` at the prompt to proceed.

**Dry run with RViz — no robot, no CAN, no motors.** The dry run publishes its *simulated*
joint state, so RViz shows a ghost arm following your controller. This is the cheapest way
to check the axis mapping (§10) and it needs neither the driver nor the arm:

```bash
# Terminal A
ros2 launch rebotarm_moveit_config hardware.launch.py model:=rs
# Terminal B — the Quest link (§ Terminal 3)
ros2 launch ros_tcp_endpoint endpoint.py
# Terminal C
ros2 run q2r2_bringup QuestTeleopReal --ros-args -p dry_run:=true
```

Do **not** start `driver.launch.py` for this. If the driver is running it also publishes
`/rebotarm/joint_states`, and two publishers make RViz flicker between the real arm and the
simulation; the node detects that, warns, and suppresses its own publishing (so RViz then
shows the real arm, which will not move). Disable with `-p publish_sim_joint_states:=false`.

### Or: the demo-station GUI (all of the above from a browser)

```bash
ros2 run demo_station demo_gui        # -> http://<jetson-ip>:8800
```

Starts/stops every process above from the **Processes** tab (driver behind a
confirmation dialog; teleop's `ARM` prompt via the card's input box), shows
live camera feeds and recording state, and browses recorded demos. See
`src/demo_station/README.md` — including the **disk throughput budget** that
dictates camera profiles while recording. Recording demos: side-squeeze the
controller (grip button) to start/stop an episode; bags land in
`~/teleop_demos/`, and nothing is ever auto-deleted.

### Terminal 5 — temperatures (recommended)

```bash
ros2 run rebotarmcontroller TemperatureDisplay
# or one-shot:
ros2 topic echo /rebotarm/thermal_status --once
```

---

## 3. Controls

| Input | Action |
|---|---|
| **Trigger** (`press_index`) | **Deadman.** Arm moves only while held. Release → stop and hold. Re-squeeze re-anchors where your hand is, so there is no jump. |
| **Upper button** | Toggle gripper open/closed |
| Side button, thumbstick | unused |

On exit — Ctrl-C, SIGTERM, exception, or a crash — the arm returns to **all joints zero**
via the driver's `safe_home` (velocity-ramped 0.5 rad/s, closes the gripper first), then
disables. `kill -9` cannot be intercepted; always use Ctrl-C.

Useful parameters:

```bash
-p track_orientation:=false     # translation only — fall back here if the wrist misbehaves
-p position_scale:=0.2          # hand travel → robot travel
-p max_cart_step:=0.008         # m per command; lower = slower, safer
-p side:=left
-p dry_run:=true
```

### Gripper — grasp-aware close (Aug 7)

The upper button toggles open/closed, but "closed" no longer means "fingers at the
mechanical limit". The driver's close (`/rebotarm/gripper/close`) now:

1. **Ramps** the MIT target toward closed at `gripper_close_speed` (default 3 rad/s over
   the 0→5 rad stroke, ~1.7 s from fully open) instead of stepping it to the limit.
2. **Detects the object**: fingers moved < `gripper_stall_dpos` (0.01 rad) over
   `gripper_stall_window` (0.15 s) while the ramp ran ≥0.1 rad ahead → grasp.
3. **Re-anchors** the target just past the object by `gripper_hold_torque / gripper_kp`
   rad, so the steady squeeze is **~`gripper_hold_torque` (default 2 Nm at the motor
   shaft)** — not `kp × remaining-stroke`, which used to saturate the rs-00's MIT torque
   range (**14 Nm**, extracted from the compiled motorbridge model table) for as long as
   you held the object.

A grasp reports **success** ("grasped object at X rad"), so the teleop's toggle stays in
sync — previously closing on any real object reported "timeout", the teleop kept thinking
the gripper was open, and the button could only ever send CLOSE again. The teleop now also
trusts the *commanded* state whenever the service call completes, so it can't wedge even
against an old driver.

**Tuning the closing gains** (all live, no restart — the driver applies them immediately;
kp/kd/hold_torque even affect a hold already in progress):

```bash
ros2 param set /reBotArmController gripper_hold_torque 3.0   # firmer grip
ros2 param set /reBotArmController gripper_kp 30.0           # stiffer spring
```

| parameter | default | what it does | tune it when |
|---|---|---|---|
| `gripper_hold_torque` | 2.0 Nm | Steady squeeze on a grasped object. **This is the one knob for "how firm".** | Objects slip → raise in 0.5 steps. Delicate objects crush → lower. |
| `gripper_kp` | 20.0 | Spring stiffness (SDK yaml shipped 50). Sets the transient pinch while the stall detector converges (~`kp × close_speed × stall_window`) and how rigidly an empty-closed gripper holds position. | Fingers oscillate → lower. Empty gripper feels floppy → raise (hold force stays `hold_torque` either way — the re-anchor distance auto-compensates). |
| `gripper_kd` | 2.0 | Damping (yaml shipped 4). | Oscillation that lowering kp doesn't fix → raise slightly. |
| `gripper_close_speed` | 3.0 rad/s | Target ramp rate; the speed the fingers meet the object. | Impacts feel harsh → lower (close takes longer). |
| `gripper_stall_window` | 0.15 s | How long the fingers must be stationary to declare a grasp. | Soft/squishy objects trigger "grasp" too early while still compressing → raise to 0.25–0.3. |
| `gripper_stall_dpos` | 0.01 rad | Motion threshold defining "stationary". | Rarely. Raise slightly if creaky/vibrating grasps never latch. |

Sanity numbers while tuning: watch the gripper effort in `ros2 topic echo
/rebotarm/joint_states` during a hold — it should settle near `gripper_hold_torque`. If a
long hold trips the thermal guard (75 °C protective stop), your hold torque is too high.
Torque here is at the **motor shaft**; fingertip force depends on the gripper transmission,
so tune by feel/scale, not by Nm alone.

### Orientation

On by default. The hand's rotation delta is anchored exactly like its position: at the
instant you squeeze, the delta is identity, so re-anchoring never jumps.

| parameter | default | meaning |
|---|---|---|
| `orientation_frame` | `tool` | `tool`: delta taken in the **hand's own** frame and applied in the **tool's own** frame — a twist about your hand's long axis twists the tool about its long axis. `base`: delta taken in the Quest **world** frame, applied in `base_link`. |
| `orientation_scale` | `1.0` | fraction of your hand's rotation the tool follows; `0.5` halves every angle |
| `max_ang_step` | `0.05` rad | slerp clamp per command (~2.9°, so ~57°/s at 20 Hz) |
| `hand_axes_rpy` | `[0,0,0]` | fixed axis remap, applied by conjugation |

**Why `tool` is the default.** Nothing in this stack converts between the Quest world frame
and `base_link` — Unity is left-handed Y-up, ROS is right-handed Z-up, and the headset's
origin depends on where you were standing when it initialised. `base` mode therefore isn't
intuitive until that alignment is measured and written into `hand_axes_rpy`. `tool` mode is
*less* sensitive: the anchor cancels the constant offset between Quest-world and base_link.
But it does **not** cancel the mapping between the controller's body axes and the tool's
body axes, and it does not cancel handedness — an earlier version of this section claimed
it "sidesteps" the Unity/ROS frame problem, which is **wrong**.

**If rotating about one axis turns the tool about a different one**, the controller's local
axes don't line up with the gripper's. Calibrate in a dry run against the RViz ghost, one
axis at a time, adjusting `hand_axes_rpy` until they agree — no hardware needed.

**MEASURED 2026-08-07 — the app's frame FOLLOWS YOUR HEAD until you align it.**
With the right controller lying untouched on a table, moving only the headset swung the
published hand pose by **35 cm and 92°** (and `/q2r_*_hand_twist` is the derivative of the
same head-relative pose — 6.8 rad/s "angular velocity" on a stationary controller — so the
twist topic is equally coupled). Consequences while unaligned: looking around drags the
arm; moving your hand sideways barely moves the robot because your head naturally follows
the hand and cancels the relative motion; and the effective axes depend on where you're
looking, so no calibration converges. **Fix it at the source: hold the controller in your
normal driving grip and press A+B on the right controller (X+Y for the left)** — quest2ros
aligns its relative frame to the controller's current pose. Verify: put the controller
down, move your head — the pose stream (RViz ghost / OrientationViz) must stay still.
Two teleop-side defenses exist as of the same date: `pos_axes` (signed axis remap for
translation, calibrate AFTER aligning) and `max_hand_speed` (default 2.0 m/s — two
consecutive faster-than-hand samples disengage and hold, so head sweeps can't move the arm).

**KNOWN LIMIT — `hand_axes_rpy` cannot fix a handedness flip.** The remap is applied by
conjugation (`R·δ·R⁻¹`), which can only re-orient axes (det = +1). A left→right-handed
conversion is a reflection (det = −1), so if the Quest app streams raw Unity quaternions,
every rotation arrives **mirrored** (right angle-magnitude, inverted sense on all axes at
once) and no `hand_axes_rpy` value will ever converge — each axis can be aligned
individually, but the sign stays wrong on all of them together. Settle it empirically
before calibrating: run `OrientationViz`, hold the controller still, rotate it exactly +90°
about one physical axis at a time, and read the raw quaternion. If the vector part points
along the *negative* of the expected axis, the stream is unconverted — the fix is a
quaternion **component remap** in `_on_pose` (or in the Unity app), not `hand_axes_rpy`.
Nothing in this workspace currently performs that conversion.

**Verified properties** (unit-tested, not just reasoned about): the anchor identity holds
exactly in both frames with remap and scaling active; a hand rotation of N° about a local
axis produces exactly N° about the corresponding tool axis; `orientation_scale` halves
angles as advertised; every output is unit-norm.

Rotation is the least forgiving input this arm has — near the ready pose 15° of pitch needs
a ~0.5 rad joint jump, and past ~30° the solver starts preferring wrist-flipped branches
(`joint6 → ±π`). The slerp clamp keeps each request small enough that IK stays on the seed's
branch, and `max_ik_jump` catches anything that slips through. If the wrist misbehaves, drop
`orientation_scale` to 0.5 before changing anything else.

---

## 4. Safety

- **RS runs `arm_control_mode: mit`** — impedance control with **no firmware velocity
  ceiling**. Torque is proportional to target error. All protection comes from the small
  bounded steps in `QuestTeleopReal`; do not raise `max_cart_step` casually.
- **Motors heat up while merely enabled.** MIT mode holds position with continuous
  torque, and `joint2`/`joint3` fight gravity the whole time. `joint3.mos` reached
  53 °C / 127 °F just sitting idle. The thermal guard warns at 60 °C and trips at 75 °C.
  - **Do not leave the driver running when you are not using the arm** — launching it
    energises the motors (§2), so an idle driver is an idle arm burning holding current.
    Shut the driver down, or `disable` it, between sessions.
  - Park **folded** (`joint2`/`joint3` positive) rather than extended — far less
    holding current, because gravity torque scales with horizontal reach.
  - `/rebotarm/gravity_compensation/start` is the low-effort hold. It puts the state
    machine in `GRAVITY_COMP`, where motion commands are **rejected** — stop it before
    teleop.

**Kill torque immediately, without motion:**

```bash
ros2 service call /rebotarm/disable std_srvs/srv/Trigger "{}"
```

**Do not** Ctrl-C the driver to stop the motors — its shutdown calls `safe_home()`, which
*drives the arm to zero first*. Also note a disabled arm goes limp: support it before
disabling if it is holding a loaded pose.

Re-enable:

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger "{}"
```

---

## 5. RS vs DM — why `model:=rs` matters

`rebotarm_hardware.yaml` has `default_model: dm`. This arm is **RS** (RobStride motors,
CAN). Differences that bite:

| | DM | **RS (this arm)** |
|---|---|---|
| transport | `/dev/ttyACM0` serial | **SocketCAN** (`can1`) |
| control mode | `posvel` (firmware velocity limit) | **`mit`** (no velocity limit) |
| `joint2`, `joint3` | `[-3.14, 0]` | **`[0, 3.14]`** — sign-flipped |
| `joint4` | `[-1.87, 1.57]` | `[-1.57, 1.57]` |
| URDF | `reBot_B601_DM_with_gripper.urdf` | `00-arm-rs_asm-v3.urdf` |

**All-zeros is a dead end for teleop.** `joint2`/`joint3` sit exactly on their lower limit
there, and the arm is straight out, so almost any Cartesian delta needs them to go
**negative** — outside their range. Measured with `/compute_ik`:

| seed / target | result |
|---|---|
| zeros, its own FK pose `[0.3017, 0, 0.2177]` | **OK** (even at a 0.03 s timeout) |
| zeros, that pose **+1 cm in x** | **FAIL −31**, even at a 1 s timeout |
| ready `[0, 1, 1, 0, 0.5, 0]`, its own FK pose `[0.3854, 0.0972, 0.4163]` | OK |
| ready, that pose **+1 cm in x** | **OK** → `[0, 1.042, 1.019, 0.023, 0.5, 0]` |

Note joint2/joint3 going *up* to absorb the +1 cm — at zeros that is the direction they
cannot go. So the pose itself is solvable and it is not a timeout or a collision problem;
it is the one-sided limit at the boundary. `QuestTeleopReal` therefore moves to
`ready_positions` before teleop and parks back at zero on exit.

Consequence for dry runs: a dry run never executes that move, so the arm stays at zeros
and every target is rejected. `QuestTeleopReal` handles it by simulating perfect tracking
from `FK(ready_positions)` instead of reading the unmoved hardware — see §3.

---

## 6. How control actually works

### The one thing to internalise

**ROS never talks to the motors.** No ROS call sends a CAN frame. Every ROS command does
exactly one thing: **write two numpy arrays.** A separate 500 Hz thread is the only code
that ever transmits.

```
  _q_target[6]      <-- all ROS commands write here, and nowhere else
  _qd_target[6]
        |
        |  read 500x/sec by ONE thread
        v
   CAN frames -> motors
```

ROS sets *intent*; the loop *executes* it. Slow or bursty ROS traffic doesn't slow the
motors — it only changes what the loop finds in those arrays. But if the loop stalls,
nothing upstream can save you.

### The stack

```
  L0  Quest headset --TCP--> ros_tcp_endpoint --> /q2r_right_hand_pose   (72 Hz)
                                                          |
  L1  QuestTeleopReal (this package, YOUR process)         |
      - anchored delta: target = anchor_ee + (hand - anchor_hand) * scale
      - clamp to max_cart_step from the last COMMANDED pose
      - /compute_ik --> move_group --> joint solution, used as a veto
      - publish JointTrajectory chunk                      20 Hz
                            |
  ==================== ROS 2 / DDS boundary =====================
                            |
  L2  reBotArmController (the driver: ONE process, ~21 threads)
      entry points, all of which only WRITE THE ARRAYS:
        /policy/action_chunk       (topic)   -> ChunkExecutor, 100 Hz, Hermite + vel
        /follow_joint_trajectory   (action)  -> Hermite interp, 100 Hz, + vel
        /move_to_pose_ik           (service) -> one-shot; leaves qd_target STALE
                                                (whatever was last latched), and
                                                solves IK while holding _cmd_lock
        /joints/<j>/cmd/mit|pos_vel (topic)  -> per-joint; clobbers the others, never use
        /enable /disable /safe_home (service)-> lifecycle
                            |
  L3  HardwareManager  -- arbitration, not control
      - state machine: IDLE | TRAJ_RUNNING | GRAVITY_COMP | SAFE_HOMING |
                       LOWLEVEL_STREAMING
      - _cmd_lock (RLock) serialises every writer
      - set_joint_position_velocity_target(q, qd) -> the arrays
                            |
  L4  RebotArmEndPose  -- owns _q_target / _qd_target. Storage plus IK/traj helpers.
                            |
  L5  RebotArm._control_loop_impl   <== THE control loop, 500 Hz, own thread
        while running:
            t0 = perf_counter()
            _ctrl_fn(self, dt)            # -> _endpos_loop_cb -> _loop_cb
            sleep(0.002 - elapsed)        # fixed period, NOT deadline-corrected
                            |
  L6  group.send_mit(pos, vel, kp, kd, tau)
        for each of 6 joints: motor.send_mit(...)    # 6 separate CAN frames
        except CallError: pass                       # <-- failures are SILENT
                            |
  L7  SocketCAN can1 @ 1 Mbit --> RobStride firmware
        the REAL servo loop, in the motor, in silicon:
            tau = kp*(q_target - q) + kd*(qd_target - qd) + tau_ff
```

### One 500 Hz cycle

`HardwareManager._endpos_loop_cb` -> `RebotArmEndPose._loop_cb`:

1. Take `_cmd_lock`, waiting up to **1.5 ms** (`_CMD_LOCK_WAIT`). It used to give up
   instantly and skip the cycle — see §7.
2. Bail if `_control_output_enabled` is false. That is the flag `disable` clears.
3. If gravity feedforward is on, compute the Pinocchio gravity torque **at the target
   posture `_q_target`** (cached, recomputed when the target moves >0.02 rad), joints 2
   and 3 scaled x1.55, into `tau_ff`. It used to read the measured position here instead —
   six blocking 0x7019 CAN round-trips per tick that capped the loop at 50–160 Hz (§7).
4. `send_mit(_q_target, vel=_qd_target, kp, kd, tau=tau_ff)` -> 6 CAN frames.
5. Gripper gets its own MIT frame — a pure spring toward `_gripper_target` (kp=20/kd=2
   after the Aug 7 override; the yaml ships 50/4). The close service ramps that target
   and re-anchors it on grasp so the squeeze is bounded (§3 Gripper).
6. Sleep the remainder of 2 ms.

### Why MIT mode explains almost everything

`kp = [50, 150, 150, 50, 50, 50]`, `kd = [3, 10, 10, 5, 4, 4]`, and the motor computes
`tau = kp*(q_target - q) + kd*(qd_target - qd) + tau_ff`.

- **No velocity ceiling.** Torque is proportional to position error, so a large jump in
  `_q_target` is a large torque, immediately. Every safety property comes from the step
  clamps upstream. (`posvel` mode, used by the DM variant, has a firmware `vlim`. RS
  does not.)
- **`qd_target` matters enormously.** Leave it at 0 while the arm should be moving and the
  damping term becomes `-kd*qd` — a brake fighting your own motion at every setpoint, with
  10.0 of it on joints 2/3. This is why streaming `move_to_pose_ik` is jerky and why the
  trajectory and chunk paths feed the interpolating spline's analytic derivative forward
  instead, so damping acts on velocity *error*.
- **Holding costs current.** The arm holds position by continuously generating torque
  against gravity. An idle-but-enabled arm gets hot (§4).

### Rates

| stage | rate | notes |
|---|---|---|
| Quest -> ROS | 72 Hz | filtered over 8 samples ~ 110 ms lag |
| chunk publishing (this package) | 20 Hz | one MoveIt IK solve per tick |
| trajectory / chunk setpoints | **100 Hz** | lowered from 200; each tick takes `_cmd_lock` |
| **MIT control loop** | **500 Hz** | the only CAN writer. Before the Aug 7 fix its gravity FF did 6 blocking CAN reads per tick — real rate was 50–160 Hz (§7) |
| firmware servo loop | kHz, inside the motor | never visible from ROS |
| `joint_states` feedback | 100 Hz | BEST_EFFORT — the QoS trap (§8) |

The gap between 20 Hz and 500 Hz is why interpolation exists: **25 control cycles happen
between two teleop commands.** Feed raw 20 Hz steps and each is held for 25 cycles then
jumps — a staircase. The chunk executor's job is to make those 25 cycles walk a smooth
curve with a matching velocity.

### Two quiet hazards

- **`except CallError: pass` in `send_mit`.** A CAN transmit failure on any joint is
  silently swallowed: that joint simply gets no command that cycle and nothing logs it. A
  degrading bus therefore fails quietly. Watch the counters in §7 (Troubleshooting).
- **The loop sleeps a fixed period, not to a deadline.** `sleep(dt - elapsed)` with no
  accumulator, so an overrun permanently shifts the phase instead of being caught up. The
  true rate under load is somewhat below 500 Hz.

---

## 7. Troubleshooting

**Teleop rejects every target — `sent=0`, and the arm only twitches on trigger release**
Symptoms: a wall of `IK guard: solution outside limits -> joint6=+3.12` and
`IK guard rejected target (code=-31)`, `joint6` near ±π from a seed of `0.0`, and exit stats
showing `sent=0`. The arm appearing to move on *release* is a red herring — with nothing ever
sent, the only command it receives all session is the brake chunk published when you let go.

Cause: **the reference pose and the IK seed came from different sources.** The anchor pose was
read from **TF** while IK was seeded from **joint states**, and on this machine those
disagreed by **50 mm**. Every request then paired a position with an orientation that no
configuration near the seed actually has, and IK could satisfy both only by flipping the
wrist — which the guard correctly rejected, every tick, forever.

`QuestTeleopReal` now takes its reference from `/compute_fk` on the measured joints — the same
model the IK solver uses — so the two are consistent by construction. Preflight prints the
disagreement so it can never be silent again:

```
[ok]   FK vs TF agreement: 0.3 mm
[WARN] FK vs TF agreement: 50.7 mm -> TF disagrees with the kinematic model
```

If that warning appears, TF is broken. Look for a **second `robot_state_publisher`** — that
is what caused it here, two of them publishing transforms for the same links so consumers
take whichever message landed last:

```bash
pgrep -af robot_state_publisher        # expect ONE
ros2 topic info /tf | grep Publisher   # expect 1-2 (rsp + static_transform_publisher)
```

Control no longer depends on TF, but RViz does, so clear it anyway.

**"The DM/RS gravity model mix-up" — investigated Aug 7 and REFUTED. Do not re-chase it.**
An earlier version of this section blamed jitter + droop on the SDK loading the DM URDF
(`rebotarm.yaml` shipped pointing at `rebotarm_dm.yaml`, and `load_robot_model()` takes no
argument). The mechanism is real *for standalone SDK use*, but it **cannot occur under the
ROS driver**: `resolve_hardware_config()` (`hardware_config.py`) pre-seeds the SDK's model
cache with the ROS-resolved config — including the RS `urdf_path` — before the controller is
constructed. Every run's resolved snapshot in `/tmp/rebotarm_ros2_*/rs_hardware.yaml` shows
`urdf_path: .../00-arm-rs_asm-v3.urdf`. The gravity model was RS all along. Two corollaries:

- The `_verify_kinematics_model` startup check compares that cache **to itself** — it is
  tautological and its green log line proves nothing.
- Editing `third_party/.../config/rebotarm.yaml` changes nothing for the driver. (Still
  worth keeping correct for standalone SDK scripts.)

**What the droop/lag actually was (fixed Aug 7):** the "500 Hz" MIT loop's gravity
feedforward called `get_positions()` every tick, and on RobStride that is **six synchronous
0x7019 CAN round-trips inside the loop body** — capping the real rate at an irregular
50–160 Hz, holding `_cmd_lock` throughout, and starving the chunk executor. Compounded by:
the executor's per-tick velocity clamp being budgeted with **nominal** dt (its effective
ceiling collapsed below the teleop's 0.8 rad/s under contention, so the arm structurally
fell behind), ensembling that 50/50-blended each live chunk against the previous chunk's
zero-velocity hold tail (a position + velocity-FF staircase at exactly 20 Hz), and a teleop
standoff: at 0.25 rad of lag the teleop went silent, the executor fell back to holding *its
own* short target, and the gap froze until re-squeeze. All four are fixed — gravity is now
evaluated at the target posture with no CAN reads, the clamp uses measured elapsed time,
ensembling defaults to `m=5.0 favor=newest`, and the teleop keeps publishing a hold chunk
at `cmd_q` while lag-holding. Gravity feedforward applies to **all** joints (Pinocchio
generalized gravity), with the hardcoded ×1.55 extra on joints 2/3 only — that factor is
still uncalibrated for RS and is the remaining known torque bias.

**Everything is extremely jittery, including the driver's own ready-pose move**
If even `follow_joint_trajectory` (the driver's smooth path) is jittery, the problem is not
the command path — it is **CPU starvation of the 500 Hz control loop.** The driver is a
single Python process with ~21 threads sharing one GIL, and its MIT loop is `SCHED_OTHER`
at nice 0. Measured on this Jetson while RViz was up:

```bash
# 1. is the loop keeping time? nominal is 100 Hz, rock steady
ros2 topic hz /rebotarm/joint_states
#    bad: average 78-92, std dev 0.028s, max gap 0.342s   <-- 34x the 10ms period

# 2. is it being preempted?
P=$(pgrep -f lib/rebotarmcontroller/reBotArmController)
grep nonvoluntary_ctxt_switches /proc/$P/status
#    bad: 1660565

# 3. who is eating the CPU?
ps -eo pcpu,pid,comm --sort=-pcpu | head -6
#    rviz2 was at 101% of one core; load average 4.0 on 6 cores
```

A 500 Hz impedance loop that stalls for tens of milliseconds delivers torque at irregular
intervals, and with `kp=150` and no velocity ceiling each late command lands as a jolt.
Fixes, in order of effect:

1. **Close RViz while driving real hardware.** It costs a full core. Run
   `driver.launch.py` alone; teleop only needs `move_group` for IK, not `rviz2`.
2. **Check for duplicate launches** — `pgrep -af endpoint.py`, `pgrep -af hardware.launch`.
   Two endpoints means two publishers on `/q2r_*_hand_pose`, so the pose filter averages
   two interleaved streams and invents motion.
3. **Raise the driver's priority:** `sudo renice -n -10 -p $(pgrep -f lib/rebotarmcontroller/reBotArmController)`
4. Drop `joint_state_rate` from 100 to 50 Hz, and close Chrome.

**The arm thrashes violently on its own, with nobody holding the trigger**
Two driver instances. Check first, before anything else:

```bash
pgrep -af reBotArmController          # more than one line = this is your bug
ros2 topic info /rebotarm/joint_states | grep Publisher   # must be 1
```

The giveaway in the teleop log is:

```
Ignoring unexpected goal response. There may be more than one action
server for the action '/rebotarm/follow_joint_trajectory'
```

Both instances run a 500 Hz MIT loop on the same CAN IDs with independent targets, so a
commanded move (even the ready-pose ramp) is fought by the other instance still holding
its stale target. Fix:

```bash
pkill -f reBotArmController
ros2 launch rebotarm_bringup driver.launch.py model:=rs channel:=can1   # once
```

`QuestTeleopReal` preflight now fails on this (`[FAIL] exactly one driver instance`).

**RViz freezes — the arm moves but the model does not follow**
Usually the arm has been **disabled**. `disable()` calls `stop_control_loop()`, and that
loop is what polls motor feedback over CAN, so `/rebotarm/joint_states` keeps publishing at
100 Hz but with unchanging positions. Nothing is broken; re-`enable` (or relaunch the
driver) and feedback resumes. Distinguish it from the QoS fault below by whether the
numbers *change*:

```bash
ros2 topic echo /rebotarm/joint_states --field position   # frozen values vs no values
```

**RViz shows the arm collapsed, all links inside each other**
Nothing is publishing joint states *that RViz can receive*. The driver publishes
`/rebotarm/joint_states` as **BEST_EFFORT**; `robot_state_publisher` and RViz subscribe
**RELIABLE**, which DDS treats as incompatible and delivers nothing — so no TF for
movable joints and every link renders at the origin. Look for
`offering incompatible QoS ... No messages will be received`.

The clean fix is one line in the **driver**: in
`~/rebotarm_ros2/src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py`, publish
`/{namespace}/joint_states` with a RELIABLE profile instead of `node.sensor_qos`. A
reliable publisher satisfies both subscriber kinds, so RViz, `robot_state_publisher`,
MoveIt and everything else start working at once. At 100 Hz on localhost the cost is
nil. Rebuild with `colcon build --packages-select rebotarmcontroller`.

Per-consumer overrides are the alternative, but note **`ros2 launch` does not accept
`--ros-args`** (only `name:=value` launch arguments) — a `qos_overrides.…` parameter has
to be added to the `robot_state_publisher` / `rviz2` `Node(...)` entries inside
`hardware.launch.py` itself.

`q2r2_bringup` nodes already subscribe BEST_EFFORT and are unaffected either way.

**`IK rejected target` / `NO_IK_SOLUTION` for everything** — arm is at all-zeros. See §5.

**Preflight: `Quest pose stream (no data)`** — endpoint not running, or the app is not
connected. Restart Terminal 3 and press **Apply** in the headset again; the app gives up
if the endpoint disappears.

**Preflight: `joint states … (none seen)`** — driver not running, or QoS mismatch (above).

**Headset won't connect** — confirm it is on the `10.0.0.x` WiFi. The headset cannot be
identified by a network scan (randomised MAC, no mDNS, no open ports); read its IP from
the headset's own WiFi settings. A successful connect logs `Connection from <ip>` in the
endpoint terminal.

**Motors respond on CAN but no joint states** — check `ip -s link show can1`. TX climbing
with RX at 0 means nothing is answering; both climbing means the bus is healthy and the
problem is above CAN (usually QoS).

---

## 8. Local patches — lost on a re-clone

`src/ros_tcp_communication` and `src/quest2ros` are **not** git repos here. Two bugs in
the recommended `guguroro/ros_tcp_communication` fork were fixed by hand:

1. **`server.py`, `handle_syscommand`** — the fork commented out upstream's
   `[:-1]`. The Quest app NUL-terminates its syscommand JSON, so `json.loads` threw
   `Extra data: line 1 column 92` and the endpoint dropped every connection within ~3 ms.
   Fixed with `.rstrip("\x00")`.
2. **`publisher.py`, `RosPublisher.send`** — the fork replaced
   `deserialize_message` with a hand-rolled parser (`ros_msg_converter.py`) that ignores
   the CDR encapsulation header and field alignment. Symptom: topics publish at the right
   rate but values are denormals and ~1e152. Fixed by restoring `deserialize_message`;
   `ros_msg_converter.py` is now dead code.

Sanity check that decoding is right: the received quaternion must be **unit-norm**
(`OrientationViz` shows the norm in green/red). A byte-shifted buffer cannot produce that.

Also local: `src/Quest2ROS2/package.xml` depends on `quest2ros` (upstream said
`quest2ros2_msg`, which nothing provides). `ros2quest.py` still has the stale import and
would crash if run — it is not part of the workflow.

**Driver-side local edits (Aug 7, in `~/rebotarm_ros2` — uncommitted, lost on re-clone
until committed and the patches in `~/rebot-arm-private/patches/` are regenerated):**

1. `third_party/.../controllers/rebotarm_endpose_controller.py` — gravity feedforward now
   evaluated at `_q_target` with a cache, instead of six blocking 0x7019 CAN reads per
   500 Hz tick.
2. `src/.../chunk_executor.py` — per-tick step clamp budgets **measured** elapsed time
   (capped at 4× nominal); control-tick timer moved to a mutually-exclusive callback group.
3. `src/.../rebotarm_controller.py` — `policy_ensemble_m` 0.01→5.0,
   `policy_ensemble_favor` oldest→newest; two dedicated mutually-exclusive callback groups.
4. `src/.../ros_publishers.py` + `hardware_manager.py` — all of the joint-state
   publisher's CAN reads now happen inside a bounded-wait `_cmd_lock` window
   (`cmd_lock_window`); on timeout the cycle is skipped instead of racing the control loop.
5. `src/.../hardware_manager.py` + `ros_services.py` + `rebotarm_controller.py` —
   **grasp-aware gripper close** (`close_gripper_grasp`): ramped target, stall detection,
   re-anchor to a bounded ~2 Nm hold instead of the 14 Nm MIT saturation; stalling on an
   object is reported as a successful grasp; `gripper_*` params live-tunable (§3 Gripper).
   `safe_home()` inherits the same bounded close. The wait/poll paths now also take
   `cmd_lock_window` instead of reading CAN unlocked.

---

## 9. Node reference (`q2r2_bringup`)

| Node | Purpose |
|---|---|
| `QuestTeleopReal` | **Real hardware.** Publishes action chunks to `/rebotarm/policy/action_chunk` (default; `command_mode:=service` streams `move_to_pose_ik` instead). Deadman, watchdog, step clamps, IK guard, 1 Hz `~/diagnostics`, park-to-zero on exit. Has `dry_run`. |
| `QuestIKBridge` | Same idea for the **mock/RViz** stack (`demo.launch.py`) — MoveIt IK → `JointTrajectory`. No hardware. |
| `OrientationViz` | Browser 3D view of controller orientation at `http://localhost:8080`. Stdlib only. |
| `CheckTCPconnection` | Prints all six Quest topics at 1 Hz; reports timeouts. |
| `SimulationInput` | Fake Quest publisher — test the stack with no headset. |

Mock-only stack, no motors, no CAN:

```bash
ros2 launch rebotarm_moveit_config demo.launch.py model:=rs
ros2 run q2r2_bringup QuestIKBridge
```

---

## 10. Known-unverified

- `ready_positions` **is** IK-solvable under the RS model — confirmed: `/compute_ik` solves
  at `FK(ready)` and at ±1 cm and ±3 cm in every axis from it, with `joint6 = 0` and no
  joint jump.
- Translation teleop on powered hardware **works** as of the FK-reference fix (§7, "teleop
  rejects every target"). Before that fix nothing was ever sent: `sent=0` with
  `guard_reject=44, limit_reject=97` in a full session.
- Still unmeasured on hardware: how much of the residual jitter is CPU starvation versus the
  command path. The CAN-frame-rate check in §7 is the way to tell.
- **Axis mapping is unconverted, for BOTH position and rotation.** Unity is left-handed
  Y-up, ROS right-handed Z-up, and nothing in this workspace transforms between them. For
  position: watch the RViz ghost in a dry run while moving your hand along one axis at a
  time; if an axis is mirrored or swapped, that is the reason. For rotation: `tool` mode
  does **not** sidestep this (a previous revision claimed it did — see §3, "KNOWN LIMIT");
  run the ±90°-per-axis `OrientationViz` test before trusting orientation tracking.
- **Orientation tracking has been exercised on hardware only after the FK/TF fix.** The
  mapping's mathematical properties are unit-tested, but how it *feels* to drive, and
  whether `hand_axes_rpy` needs a non-zero value, is unconfirmed.
