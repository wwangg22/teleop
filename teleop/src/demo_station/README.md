# demo_station — teleop demo collection

One package for collecting training demos with the Quest teleop stack:
an episode **recorder** toggled by squeezing the controller's side/grip
button, **multi-camera** bringup from one config file, and a **web GUI**
that manages every process, shows live video, and browses past demos.

```bash
# build once (symlink install: later edits need no rebuild)
cd ~/Desktop/teleop && colcon build --symlink-install --packages-select demo_station

# run the GUI (starts nothing by itself)
ros2 run demo_station demo_gui          # -> http://<jetson-ip>:8800
```

From the GUI's **Processes** tab start, in order: cameras → MoveIt → Quest
link → driver (confirmation dialog — it energizes the motors) → teleop
(then type `ARM` in the card's input box after preflight passes) → recorder.
Everything can also be run by hand in terminals exactly as before; the GUI
is optional glue, not a new code path.

## Recording

- **Side-squeeze** (grip, `press_middle > 0.8`) the driving controller to
  **start** an episode; squeeze again to **stop**. 1 s lockout between
  toggles; the squeeze must relax below 0.3 to re-arm.
- The GUI's Live tab has equivalent Record/Stop buttons and an unmissable
  red border + byte counter while recording (the counter is bytes actually
  on disk — if it ticks, data is flowing).
- **Nothing is ever discarded automatically, and the GUI cannot delete.**
  Crashes, disengages, dead processes: the bag stays; `info.json` records
  how the episode ended (`stop_reason`, `clean_close`).

## What a demo is

`<demos_root>/YYYY-MM-DD/ep_HHMMSS/` containing:

- `bag/` — one raw rosbag2 bag (MCAP if the plugin is installed, else
  sqlite3) holding **everything**: joint states (100 Hz), per-joint motor
  states, gripper state, commanded action chunks, the teleop gripper
  command topic, all `/camera/*` topics (RGB + depth + camera_info +
  extrinsics), raw Quest poses/twists/inputs, `/tf`(+static),
  `/robot_description`, teleop diagnostics, thermal status.
- `info.json` — start/stop wall times, start/stop source, duration, bytes.
- `params/` — best-effort `ros2 param dump` of the teleop and driver at
  record start (scales, gripper gains — so a weird demo can be traced to
  its settings).

Every message keeps its publisher's header stamp; nothing is resampled.
Align offline at training time (LeRobot/HDF5 conversion happens on a PC,
not here).

## Cameras

`config/station.yaml` → `cameras:` list. One entry per camera (name,
serial, profile); `ros2 launch demo_station cameras.launch.py` starts all
of them under `/camera/<name>/`, and the record manifest catches them by
regex. Adding a camera = plug in + add an entry.

## ⚠ Disk throughput budget (measured 2026-08-08)

The Jetson's eMMC sustains **60 MB/s**. Uncompressed RGB-D costs, per camera:

| profile | nominal write rate |
|---|---|
| 848×480×30 | ~61 MB/s — **saturates the disk**, thrashes the system |
| 640×480×30 | ~46 MB/s — current default, OK for ONE camera |
| 424×240×30 | ~15 MB/s — needed per-camera when running several |

A 5-minute 848×480 test episode wrote 16.7 GB and pushed load to 5+ with
multi-minute DDS backlogs (controller inputs arrived minutes late). Stay
inside the budget: the Live tab shows write rate and system load while
recording. When a second camera arrives, either drop profiles or revisit
compression (MCAP+zstd) — raw at two full-res cameras is physically
impossible on this disk.

## Config files

- `config/station.yaml` — demos root, controller side, squeeze thresholds,
  cameras, record manifest (topics + regex).
- `config/units.yaml` — the processes the GUI manages (command, health
  topics, confirm/stdin flags, singleton guards).

Both are symlink-installed: edit → restart the affected node. No rebuild.

## Notes / known limits

- rosbag2 Humble silently records **nothing** if given explicit topics
  *and* `--regex` together — the recorder therefore compiles its whole
  manifest into a single regex. Don't "simplify" that back.
- MCAP plugin: `sudo apt-get install ros-humble-rosbag2-storage-mcap`,
  then restart the recorder (it auto-detects). Until then bags are sqlite3.
- The GUI never SIGKILLs anything: stop = SIGINT (driver safe-homes, bags
  finalize), SIGTERM only after a 10 s grace period.
- Playback in the Demos tab streams straight out of the bag (no extra
  files); the joint plot reads `/rebotarm/joint_states` from the same bag.
