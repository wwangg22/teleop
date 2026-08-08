# reBot Arm B601-RS on Jetson Orin Nano — Setup, Operation, and Notes

Everything learned bringing up a Seeed reBot Arm B601-RS (RobStride motors) on an
NVIDIA Jetson Orin Nano Super, under ROS 2 Humble + MoveIt 2, including a
learned-policy action-chunk executor.

---

## 1. Hardware

| | |
|---|---|
| Compute | NVIDIA Jetson Orin Nano Super Dev Kit, 6× Cortex-A78AE, 7.4 GB RAM |
| OS | Ubuntu 22.04.5, JetPack 6.2 / L4T R36.4.7, CUDA 12.6 |
| Arm | reBot Arm B601-RS, 6 DOF + gripper, RobStride motors |
| CAN adapter | PEAK PCAN-USB (`0c72:000c`) |

### CAN — the single most important detail

**The PEAK adapter is `can1`, NOT `can0`.**

`can0` is the Orin's *onboard* `mttcan` controller. It has no transceiver wired
and is unused — scanning it returns nothing. Every launch command must pass
`channel:=can1`, and the Seeed wiki's `can0` examples are wrong for this machine.

Bitrate is **1 Mbit/s**. Nothing persists it across reboots, so bring it up each
session:

```bash
~/rebot-setup/can-up.sh
```

A sudoers drop-in (`/etc/sudoers.d/99-can-willy`) allows `ip link set can*`
without a password. Note it does **not** cover the `restart-ms` argument.

### Motors

| Joint | ID | Model | Feedback ID |
|---|---|---|---|
| joint1–3 | `0x01`–`0x03` | `rs-06` | `0xFD` |
| joint4–6 | `0x04`–`0x06` | `rs-00` | `0xFD` |
| gripper | `0x07` | `rs-00` | `0xFD` |

Verify at any time:

```bash
python3 -m motorbridge scan --vendor robstride --channel can1 \
    --transport socketcan --start-id 1 --end-id 7
```

Expect `scan done: 7 motor(s) found`. **Always run this before starting the
driver** — it takes two seconds and prevents the driver hanging on a partial bus.

---

## 2. Software stack and its landmines

```
MoveIt 2  ──►  reBotArmController (ROS 2 driver)
                   │
                   ├── reBotArm_control_py   (kinematics/dynamics, pinocchio)
                   └── motorbridge           (CAN motor driver)
                            │
                         SocketCAN can1 @ 1 Mbit
                            │
                       RobStride motors
```

### Python environment — read this before debugging anything

**Conda must NOT be active.** Miniforge's base is Python 3.13; ROS 2 Humble is
built against system Python **3.10**, and the reBot SDK declares
`requires-python = ">=3.10,<3.12"`. Under conda's Python, `import pinocchio`
fails and the driver dies at startup.

Auto-activation is disabled (`conda config --set auto_activate_base false`).
If your prompt shows `(base)`, run `conda deactivate`. Use `conda activate rebot`
only for non-ROS motorbridge work.

### Version pins that must not be "upgraded"

| Package | Pin | Why |
|---|---|---|
| `numpy` | **exactly 1.23.5** | ≥1.24 removed `np.float`, which Ubuntu's `transforms3d` still uses → `tf_transformations` import fails → driver won't start. But the SDK needs ≥1.23.4. 1.23.5 is the only value satisfying both. |
| `setuptools` | **<80** | `colcon-core` requires it. Upgrading to 83 breaks the build. |
| `pin` (pip pinocchio) | **do not install** | ROS ships `ros-humble-pinocchio` 4.0.0; a second copy risks shadowing it. |

### The SDK is not pip-installable — by design

`third_party/reBotArm_control_py` has no `[build-system]` in its
`pyproject.toml` and a flat layout setuptools can't auto-discover. It does not
need installing: `hardware_config.py` locates the directory and
`sys.path.insert()`s it at runtime. **Ignore the wiki's `pip install -e` step.**

### Rebuilding

```bash
~/rebot-setup/build-ws.sh
```

Python-only changes need no rebuild (`--symlink-install`). Changes to
`.msg` files **do** require `colcon build`.

---

## 3. Running the arm

### Normal startup

```bash
# 1. Verify motors (2 seconds, prevents a hung driver)
python3 -m motorbridge scan --vendor robstride --channel can1 \
    --transport socketcan --start-id 1 --end-id 7

# 2. CAN up
~/rebot-setup/can-up.sh

# 3. Driver  — ENERGIZES THE MOTORS
source /opt/ros/humble/setup.bash
source ~/rebotarm_ros2/install/setup.bash
ros2 launch rebotarm_bringup driver.launch.py model:=rs channel:=can1

# 4. MoveIt + RViz (separate terminal)
ros2 launch rebotarm_moveit_config hardware.launch.py model:=rs
```

In RViz: **Displays → MotionPlanning → Planning Request → tick Query Goal
State** (off by default in the shipped config), then drag the marker →
**Plan** → inspect → **Execute**.

### Simulation — no hardware, zero risk

```bash
ros2 launch rebotarm_moveit_config demo.launch.py model:=rs
```

Uses `mock_components/GenericSystem` — never touches CAN. Ctrl-C is safe here.
Verified to match real-hardware behaviour closely.

### GUI helpers (built for this setup)

```bash
ros2 run rebotarmcontroller TemperatureDisplay   # live motor temps, °F
ros2 run rebotarmcontroller HomeButton           # one button -> zero pose
ros2 run rebotarmcontroller MockPolicy           # fake policy chunks
```

### Useful commands

```bash
ros2 service call /rebotarm/enable    std_srvs/srv/Trigger
ros2 service call /rebotarm/disable   std_srvs/srv/Trigger   # ARM GOES LIMP
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger   # drive to zero

ros2 run rebotarmcontroller MoveTo -- --joint joint1 --position 0.15 --duration 6.0
ros2 run rebotarmcontroller MoveTo -- 0 0 0 0 0 0 --duration 10.0
ros2 run rebotarmcontroller GravityCompensation                # hand-guiding

ros2 topic echo /rebotarm/joint_states --once
ros2 topic echo /rebotarm/arm_status --once
ros2 topic echo /rebotarm/thermal_status --once
```

---

## 4. SAFETY — behaviours that are not obvious

### Stopping the driver

| Action | What actually happens |
|---|---|
| **Ctrl-C** | Runs `safe_home()` — **the arm MOVES to the zero pose**. Not a stop. |
| **SIGTERM** (`kill`) | Same — handler added so it homes first rather than dying. |
| **SIGKILL** (`kill -9`) | Cannot be intercepted. Motors **latch the last setpoint and keep holding**. |
| **`/rebotarm/disable`** | Motors de-energize → **arm goes limp and DROPS**. Support it first. |
| **Power switch** | The real emergency stop. |

**Killing the driver does not de-energize the arm.** RobStride motors in MIT
mode hold their last commanded setpoint indefinitely with no host present — the
control loop only *refreshes* the target, it isn't what generates torque. Worse:
killing mid-trajectory latches a stale mid-swing setpoint at full `kp`.

### Zero pose is not gravity-neutral

The arm sags from zero when de-energized. Support it before `/rebotarm/disable`.

### Thermal

The arm gets warm **just holding a pose** — continuous current, no motion to
shed heat. Observed: joint3 reached **129 °F (54 °C)** holding position for a
few minutes; joints 2 and 3 run hottest. Disable the arm when you step away.

---

## 5. How the control loop actually works

**It is PD + gravity feedforward — not PID. There is no integral term.**

```
τ = kp·(q_des − q) + kd·(qd_des − qd) + τ_gravity(q)
```

`kp = [50, 150, 150, 50, 50, 50]`, `kd = [3, 10, 10, 5, 4, 4]`.

**The PD math runs on each motor's firmware, not the Jetson.** The host only
streams `(q_des, qd_des, kp, kd, τ_ff)` over CAN. This is why CAN bandwidth
isn't limiting, and why the arm keeps holding when the driver dies.

**Gravity compensation is already active** during all motion —
`use_gravity_ff=True` in the SDK's endpose controller computes pinocchio's
generalized gravity every cycle, with a hardcoded `1.55×` on joints 2 and 3.
Note the `tau_scale` in `rebotarm_hardware.yaml` applies only to the *standalone*
`GravityCompensation` mode, not this path.

**No integral term means steady-state droop is expected** — measured 0.007 rad
at zero, 0.044 rad extended. The fix is better feedforward, not an I term.

### Rates

| Loop | Rate |
|---|---|
| Motor internal PD | kHz, on-motor |
| SDK control loop | 500 Hz nominal. Before the Aug 7 gravity-FF fix the loop body did 6 blocking CAN reads per tick and really ran 50–160 Hz |
| Trajectory setpoints | **100 Hz** (was 50, briefly 200, walked back — each tick takes `_cmd_lock`) |
| Trajectory feedback | 20 Hz (throttled) |
| Chunk executor | **100 Hz** (`policy_control_hz`; docs previously said 200) |
| `joint_states` | configured 100 Hz; delivered ~53 Hz while the bus was CAN-bound — re-measure after the Aug 7 fixes |
| Thermal guard | 1 Hz |

---

## 6. Local modifications to the Seeed repos

A `git pull` will conflict with these. `git diff` in `~/rebotarm_ros2` and
`~/rebotarm_ros2/third_party/reBotArm_control_py` shows them all.

| File | Change | Why |
|---|---|---|
| `rebotarm_moveit_config/launch/hardware.launch.py` | added `Path`, `yaml`, `PackageNotFoundError`, `get_package_share_directory` imports | **Upstream bug** — the file cannot launch without them. Worth reporting. |
| `ros_actions.py` | 50 Hz → **100 Hz** setpoints (was briefly 200, walked back) | 50 Hz against the MIT loop held each target for 10 cycles → visible stepping; 200 Hz lost too many `_cmd_lock` acquisitions. |
| `ros_actions.py` | **cubic Hermite** interpolation using MoveIt's `point.velocities` | Driver discarded MoveIt's velocities and interpolated linearly → piecewise-constant velocity, stepping at every waypoint. |
| `ros_actions.py` | real **velocity feedforward** | `qd_des` was forced to 0 every setpoint, so `−kd·qd` braked against the motion — the "rocking backwards" symptom. |
| `ros_actions.py` | feedback **throttled to 20 Hz** | `get_joint_positions()` costs one CAN param read *per joint*; doing that every 5 ms saturated the bus and made trajectories overrun → MoveIt aborted them. |
| `hardware_manager.py` | `set_joint_position_velocity_target()` | Position setter forcibly zeroed `qd_target`. |
| `hardware_manager.py` | `get_joint_temperatures()` / `get_gripper_temperatures()` | SDK dropped temperature. |
| `actuator/rebotarm.py` (SDK) | `get_temperatures()` on `JointGroup`, `RebotArm`, `NoOpGroup` | `MotorState` carries `t_mos`/`t_rotor`; `get_state()` forwarded only pos/vel/torque. |
| `JointMotorState.msg` | `temperature_mos`, `temperature_rotor` | Needs `colcon build`. |
| `thermal_guard.py` *(new)* | monitoring + protective stop | See below. |
| `chunk_executor.py` *(new)* | learned-policy execution | See §8. |
| `temperature_display.py`, `home_button.py`, `mock_policy.py` *(new)* | GUIs / test tools | |

**Aug 7 additions (uncommitted, after the teleop jitter root-cause investigation —
full analysis in `~/Desktop/teleop/README.md` §7):**

| File | Change | Why |
|---|---|---|
| `controllers/rebotarm_endpose_controller.py` (SDK) | gravity FF evaluated at `_q_target`, cached | The old per-tick `get_positions()` = **6 blocking 0x7019 CAN round-trips inside the 500 Hz loop body** → real rate 50–160 Hz, `_cmd_lock` held throughout, ~3000 frames/s of bus load. |
| `chunk_executor.py` | step clamp budgets **measured** elapsed time (≤4× nominal); timer on a mutually-exclusive group | Nominal-dt budgeting collapsed the velocity ceiling below the teleop's budget on late ticks; reentrant timer ticks raced on `_last_target`. |
| `rebotarm_controller.py` | `policy_ensemble_m` 0.01→**5.0**, `policy_ensemble_favor` oldest→**newest**; two dedicated timer callback groups | With only 2 overlapping teleop chunks, near-uniform weights blended a live segment 50/50 against a stopped hold tail and snapped at expiry — a 20 Hz staircase. |
| `ros_publishers.py` + `hardware_manager.py` | all publisher CAN reads inside a bounded-wait `cmd_lock_window()`; skip cycle on timeout | Unlocked request/response reads could consume the control loop's reply frame and stall it up to the SDK's 1 s timeout (seen as 1 s `joint_states` gaps). |
| `rebotarm_controller.py` | SIGTERM handler, guard + executor wiring | "never just stops" — always homes first. |
| `hardware_manager.py` + `ros_services.py` + `rebotarm_controller.py` | **grasp-aware gripper close** (`close_gripper_grasp`): ramped target, stall detection, torque re-anchor; `gripper_*` params, live-tunable | The close was a pure MIT spring to the mechanical limit — any grasped object saw kp×(remaining stroke), **saturated at the rs-00's 14 Nm MIT frame limit, indefinitely**, and the service reported the successful grasp as a "timeout". See §6b. |

### 6b. Gripper compliance — grasp-aware close (Aug 7)

**What the close does now** (`/rebotarm/gripper/close`, also inherited by
`safe_home()`):

1. Ramps the MIT target closed at `gripper_close_speed` (3 rad/s default;
   full 5 rad stroke ≈ 1.7 s) instead of stepping it to 0.0.
2. Declares a **grasp** when the fingers move < `gripper_stall_dpos` (0.01 rad)
   over `gripper_stall_window` (0.15 s) while the ramp has run ≥ 0.1 rad ahead.
3. Re-anchors the target to `stall_pos − gripper_hold_torque/gripper_kp`, so the
   steady squeeze is **~`gripper_hold_torque` (2 Nm default)** at the motor
   shaft. (The ±14 Nm figure is the rs-00 row of the MIT-frame range table
   extracted from the compiled `libmotor_abi.so` — nothing in the stack ever
   writes `limit_cur`/`limit_torque`, so that saturation was the only cap.)
4. Grasp → service returns `success=True`, message `"grasped object at X rad"`.
   Only a genuine no-motion-no-stall run returns a timeout.

**Gripper parameters** (all live via `ros2 param set /reBotArmController …`;
kp/kd/hold_torque affect even a hold already in progress because the 500 Hz
loop re-reads the gain arrays every tick):

| Parameter | Default | Notes |
|---|---|---|
| `gripper_kp` | 20.0 | MIT spring stiffness; overrides the SDK yaml's 50. Transient pinch while the detector converges ≈ kp × close_speed × stall_window. |
| `gripper_kd` | 2.0 | Damping; yaml shipped 4. |
| `gripper_hold_torque` | 2.0 | Nm at the shaft — **the** firmness knob. Steady-state effort in `joint_states` should settle near this during a hold. |
| `gripper_close_speed` | 3.0 | rad/s target ramp = speed the object is met at. |
| `gripper_stall_window` | 0.15 | s stationary before "grasp". Raise to 0.25–0.3 for squishy objects. |
| `gripper_stall_dpos` | 0.01 | rad motion threshold defining "stationary". |

Tuning guide (also in the teleop README §3): slipping → raise `hold_torque` in
0.5 Nm steps; crushing → lower it; oscillation → lower `kp` first, then raise
`kd`; harsh impacts → lower `close_speed`; premature "grasp" on foam → raise
`stall_window`. A hold that trips the 75 °C thermal stop means `hold_torque` is
too high. Bench check: `close on air` → "complete"; close on a wooden block →
"grasped at X rad" within ~0.3 s of contact, effort ≈ hold_torque, temperature
flat over minutes.

### Thermal guard

Polls at 1 Hz, warns at 60 °C / 140 °F, and on 3 consecutive readings above
75 °C / 167 °F performs a **protective stop: `safe_home()` to zero, *then*
disable** — never dropped mid-pose. Latched so it can't loop. If `safe_home`
fails it deliberately does **not** disable (holding beats dropping).

It **fails permissive**: NaN, exactly `0.0`, and >200 °C are treated as "no
reading" and never trip it, because a spurious stop on a moving arm is itself a
hazard. If no valid readings appear it says so loudly and stands down.

**Only `t_mos` works.** `t_rotor` reads a constant `0.0` on RobStride and is
filtered out — so you have MOSFET temperature only, not winding temperature.

**The thresholds are placeholders, not RobStride datasheet values.** Verify
against the rs-06/rs-00 spec.

---

## 7. Debugging lessons

### ⚠️ If the arm goes limp but still responds: POWER-CYCLE THE ARM FIRST

The costliest lesson of the session. Symptoms:

- Motors scan fine, 7/7
- Report live position
- Fault register **`0x0`** — clean
- `run_mode = 0` (MIT) — correct
- Bus voltage **48.3 V** — full
- `enable()` returns success
- **Zero torque, from every software path**

Four independent stacks failed identically — motorbridge-studio, the stock
upstream ROS driver, a modified driver, and raw motorbridge. The motors were in
a **latched internal state that only power removal clears**, and it advertised
nothing in any readable register.

**Power the arm off, wait ~15 s for the bus caps to bleed down, power on.** It
worked immediately afterwards. Hours were lost suspecting software, then the
power supply. Neither was at fault.

### Other traps

- **`arm_status` is a latched topic** (`TRANSIENT_LOCAL`, depth 1). `echo --once`
  can return a stale message. Trust the driver's own log for live state.
- **CAN frame-rate arithmetic (superseding an old "~230 writes/s is normal" note,
  which was wrong):** the MIT loop sends 7 frames/tick (6 arm + gripper), so a
  healthy post-fix loop is ~3500 TX frames/s at 500 Hz, plus the joint-state
  poll (~700 req + 700 replies/s at 100 Hz). Before the Aug 7 fix the loop also
  did 6 param reads/tick — ~80% bus load and the loop itself was the bottleneck.
  A *low* TX rate now means the loop is stalled, not that things are calm.
- **PCAN netdev error counters are unreliable.** `ip -s link` showed 0 errors
  while transmitting into a dead bus. `/proc/pcan`'s `read` column is the honest
  one — `read = 0` means nothing is answering.
- **`per_joint_status_code` 1/2 is not a fault** — it tracks the rs-06/rs-00
  split.
- **RobStride cached `MotorState` position freezes.** `get_state().pos` can be
  stale; the live value is parameter `0x7019` (`mechPos`). Cost me a false
  "didn't move" reading.
- **RViz segfaults on window close.** Harmless Ogre teardown issue on Jetson.
  Closing the RViz window does **not** stop the launch — Ctrl-C the terminal.
- **Don't `pkill -f "driver.launch.py"`** — the pattern matches your own shell
  wrapper and kills the relaunch. Kill by PID.

### Known open issues

1. **`driver_params.yaml` is not loaded.** The thermal guard started with
   `thermal_enabled: false` set. `driver.launch.py` isn't passing the config to
   the node — edit params at launch instead until fixed.
2. `joint_state_rate` advertised 100 Hz but delivered ~53 Hz while the MIT loop
   was doing 6 param reads/tick (fixed Aug 7 — re-measure; it may also now skip
   the occasional cycle by design when the command lock is busy).
3. `limit_spd = 1.0 rad/s` on all motors — unusually low; unexamined.
4. The SDK's `open_gripper()` destructively zeroes gripper `kp`/`kd` and never
   restores them. The ROS driver doesn't call it, so you're safe under ROS — but
   not if you script the SDK directly.

---

## 8. Running a learned policy — the action-chunk executor

### The problem

Policies (ACT, Diffusion Policy, π₀, SmolVLA) don't emit one action per control
tick. They emit a **chunk**: a sequence of future actions from one observation,
arriving slowly (5–10 Hz) because inference is expensive. The arm wants a
setpoint every few milliseconds. Feeding it 5 Hz steps reproduces exactly the
stepping we removed from the trajectory path.

Three sub-problems: **upsample** 5–10 Hz → 100 Hz, **blend** overlapping chunks
that disagree about the same instant, and **contain** a policy that may emit
nonsense.

### What the state of the art does

- **Temporal ensembling** (ACT, Zhao et al. RSS 2023) — average every chunk's
  prediction for a given instant with exponential weights. Purely consumer-side.
  **This is what's implemented.**
- **Receding horizon** (Diffusion Policy) — execute the first *k* of *H*, replan.
  Equivalent to ensembling with all weight on the newest chunk.
- **Real-Time Chunking** (π₀, Physical Intelligence 2025) — freeze the actions
  that will execute during inference latency and treat the chunk overlap as an
  **inpainting** problem, guiding the flow-matching denoiser so the new chunk
  continues the old one. Strictly better, but it lives **inside the policy's
  sampling loop** and cannot be done by an executor. If you adopt it, publish
  already-continuous chunks and set `policy_ensemble_favor: newest` with a large
  `policy_ensemble_m` so this node stops blending and only upsamples.

### How it works here

Runs **inside the driver process**, driving the existing 500 Hz loop directly
(no per-setpoint ROS round-trip).

**Upsampling** — cubic Hermite between chunk waypoints, with the analytic
derivative fed forward as velocity. If the policy sends velocities they're used;
otherwise Catmull-Rom tangents are synthesised. Verified offline at the original
200 Hz tick (now 100 Hz): max step 0.01 rad, no jerk discontinuity. The step
budget uses **measured** elapsed time since Aug 7 — a late tick no longer
silently shrinks the velocity ceiling.

**Blending** — ACT's `w_i = exp(-m·i)`. ACT weights the **oldest** prediction
highest (commitment over reactivity) with `m = 0.01` making it nearly uniform —
correct for dozens of overlapping policy chunks, **wrong for the teleop**, whose
short chunks overlap at most two-deep with the older one in a zero-velocity hold
tail (50/50 blend against a stopped target, snapping at expiry = 20 Hz
staircase). **Defaults are now `m=5.0, favor=newest`** (the live chunk gets ~99%
of the weight). A genuine multi-chunk policy should set `m` back toward 0.01.

**Safety** — the layer that matters with an opaque policy:

| Guard | Behaviour |
|---|---|
| Joint limits | hard clip |
| Per-tick step | clamped to `max_joint_velocity × dt` — any bad chunk becomes a bounded ramp |
| First command | **refuses to start** if the first target is >0.35 rad from the current pose |
| Stale chunks | holds position; never coasts |
| Mode conflict | stands down during `safe_home` / trajectories / gravity comp |
| Activation | inactive until `/rebotarm/policy/start`; publishing alone cannot move the arm |

### Publishing chunks

`trajectory_msgs/JointTrajectory` on `/rebotarm/policy/action_chunk`:

```
header.stamp              time the FIRST action should execute.
                          Stamping with OBSERVATION time instead? set
                          policy_chunk_time_offset to your inference latency.
joint_names               any order; remapped to hardware order.
points[i].positions       absolute joint targets, radians. REQUIRED.
points[i].velocities      optional -> Hermite; omitted -> Catmull-Rom.
points[i].time_from_start offset from header.stamp; all-zero -> policy_dt assumed.
```

### Operating it

```bash
ros2 service call /rebotarm/enable       std_srvs/srv/Trigger
ros2 run rebotarmcontroller MockPolicy   # test without a real policy
ros2 service call /rebotarm/policy/start std_srvs/srv/Trigger
ros2 topic echo /rebotarm/policy/status
ros2 service call /rebotarm/policy/stop  std_srvs/srv/Trigger   # holds position
```

`MockPolicy` supports `stamp_mode:=delay` to simulate inference latency, plus
`amplitude`, `period`, `joints`, `chunk_size`, `publish_hz`.

### Parameters

| Parameter | Default | Notes |
|---|---|---|
| `policy_control_hz` | 100.0 | setpoint rate (was 200; walked back for `_cmd_lock` headroom) |
| `policy_dt` | 0.1 | assumed action spacing when `time_from_start` is unset |
| `policy_ensemble_m` | 5.0 | newest chunk gets ~99% weight; set toward ACT's 0.01 for real multi-chunk policies |
| `policy_ensemble_favor` | `newest` | reactive (right for teleop); `oldest` = ACT's commitment scheme |
| `policy_chunk_time_offset` | 0.0 | set to inference latency if stamping observations |
| `policy_stale_timeout` | 0.5 | seconds before "holding" warning |
| `policy_max_joint_velocity` | 2.0 | rad/s clamp (was 1.0 — throttled horizontal teleop asymmetrically; still 5x under URDF limits) |
| `policy_max_position_jump` | 0.35 | rad; refuses to start beyond this |

### ⚠️ Not yet tested on hardware

The maths is verified offline (12/12 checks: interpolation, ensembling weights,
smoothness, velocity derivative, clamps) but **no chunk has driven the real
arm**. First run: `MockPolicy` with small `amplitude`, `joints:="[0]"` (base
rotation, gravity-neutral), hand near the power switch.

---

## 9. LeRobot — not installed, and why

The originally-requested LeRobot path was deferred. Blockers:

1. **No leader arm.** `lerobot-teleoperate` and `lerobot-record` both require a
   reBot Arm 102 leader. Without it there is no way to collect demonstrations.
2. **RS support is unfinished.** `lerobot-robot-seeed-b601` registers
   `seeed_b601_rs_follower` as *"registered, still being refined"*; the DM
   variant is the supported path.

If you pursue it: use Python **3.10**, not the wiki's 3.12 — the Jetson CUDA
PyTorch wheels are `cp310` only (`pypi.jetson-ai-lab.io/jp6/cu126`); on 3.12 you
get a CPU-only torch. Also expect to train off-box: 8 GB won't fit SmolVLA at
batch 64, and ACT at 300k steps is impractical here. Record on the Jetson, train
elsewhere, run inference back on the Jetson.

---

## 10. Scripts

| Script | Purpose |
|---|---|
| `~/rebot-setup/can-up.sh` | bring `can1` up at 1 Mbit |
| `~/rebot-setup/reset-can.sh` | full PCAN driver reload (needs sudo) |
| `~/rebot-setup/probe-can.sh` | sweep bitrates to identify one |
| `~/rebot-setup/build-ws.sh` | rebuild workspace with correct env |
| `~/rebot-setup/install-ros2.sh` | original ROS 2 + MoveIt install |
