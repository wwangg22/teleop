# reBot B601-RS Teleop + Demo-Collection Stack

Quest-controller teleoperation of a Seeed reBot Arm B601-RS (6-DOF + gripper,
RobStride motors on CAN), plus a demo-recording pipeline (RGB-D cameras +
rosbag2 episodes) and a web GUI that manages the whole stack.

This repo is a **working snapshot pushed 2026-08-08 from the machine that runs
the robot** (Jetson Orin Nano). Everything here has run on hardware. The docs
in `docs/` and `teleop/README.md` are hard-won ground truth from days of
debugging — where a doc and the vendor's claims disagree, trust these docs;
where these docs and the code disagree, trust the code.

## Repo layout

| Path | What it is |
|---|---|
| `teleop/` | ROS 2 workspace #2: Quest teleop (`Quest2ROS2/q2r2_bringup`), Quest message defs (`quest2ros`), patched TCP endpoint (`ros_tcp_communication`), demo recorder + web GUI (`demo_station`) |
| `rebotarm_ros2/` | ROS 2 workspace #1: arm driver (`rebotarmcontroller`), launch (`rebotarm_bringup`), MoveIt config, messages — plus the **patched vendor SDK** in `third_party/reBotArm_control_py` (URDFs/meshes included; the gravity model needs them) |
| `teleop/README.md` | **The operational manual**: how to run everything, controls, safety, how control actually works, troubleshooting, local-patch inventory |
| `docs/ARM.md` | Arm bring-up + control internals: CAN, rates, MIT mode, thermal guard, gripper compliance (§6b), debugging lessons |
| `docs/HANDOFF.md` | Session handoff: what was broken, what was fixed (Aug 7–8), what is still unverified |
| `docs/PATCHES.md` | How the vendor-repo patches were originally managed |
| `setup/` | Shell helpers from the robot host (CAN up, ROS install, workspace build) |

⚠ `teleop/src/quest2ros` and `teleop/src/ros_tcp_communication` contain
**hand-applied bug fixes** with no upstream (see `teleop/README.md` §8).
`rebotarm_ros2` contains uncommitted-upstream fixes throughout. Do not
"update" any of these from their original repos — you will reintroduce the
bugs they fix.

---

# Fresh-machine setup (Ubuntu 22.04 desktop)

Instructions written for a Claude agent doing the setup. Steps are ordered;
don't skip the verifies. Machine roles: the **robot host** needs the CAN
adapter + cameras physically attached (currently the Jetson). A desktop
without the robot can still build everything, run the teleop **dry run**
against RViz, develop, and train — the hardware-only steps are marked.

## 0. Preconditions

```bash
lsb_release -a          # must be Ubuntu 22.04 (jammy)
sudo -v                 # need sudo
```

## 1. NVIDIA driver (desktop GPU)

```bash
sudo apt-get update
sudo apt-get install -y ubuntu-drivers-common
mokutil --sb-state 2>/dev/null   # note the result BEFORE installing
sudo ubuntu-drivers install      # picks the recommended driver (>=535 expected)
sudo reboot
```

After reboot, verify — this must print the GPU and a driver version:

```bash
nvidia-smi
```

- **If Secure Boot is enabled** (`mokutil --sb-state` said so): the driver
  modules are signed with a MOK during install and a blue "Perform MOK
  management" screen appears on reboot — the *user must be told beforehand*
  to choose *Enroll MOK* and enter the password set during install.
  If `nvidia-smi` fails afterwards with "driver not loaded", Secure Boot
  rejection is the first suspect (`sudo dmesg | grep -i nvidia`).
- **Do not install the CUDA toolkit** for this stack. The driver alone is
  enough: PyTorch/JAX pip wheels bundle their own CUDA runtime. Only install
  the toolkit if someone later needs `nvcc`.

## 2. ROS 2 Humble

```bash
sudo apt-get install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
sudo apt-get install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions python3-rosdep
sudo rosdep init 2>/dev/null || true
rosdep update
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

## 3. Stack dependencies

```bash
# ROS packages the stack needs beyond desktop:
sudo apt-get install -y \
  ros-humble-moveit \
  ros-humble-pinocchio \
  ros-humble-realsense2-camera \
  ros-humble-rosbag2-storage-mcap \
  python3-pip can-utils

# Python (user-level):
pip3 install --user motorbridge==0.5.0 fastapi 'uvicorn[standard]'
```

Notes:
- `ros-humble-pinocchio` provides the `pinocchio` Python module the SDK's
  gravity feedforward imports. Do NOT pip-install pinocchio alongside it.
- `motorbridge` is the CAN motor library (Rust core, Python bindings). 0.5.0
  is the version validated on the robot. If pip has no wheel for x86_64,
  that only blocks *driving the arm from this machine* — everything else
  still works; note it and move on.
- OpenCV comes with ros-humble-desktop's python3-opencv; don't pip-install
  opencv-python over it.

## 4. Clone and build (order matters)

```bash
git clone git@github.com:wwangg22/teleop.git ~/stack
cd ~/stack

# Workspace 1: arm driver (+ vendor SDK). third_party/ MUST stay a sibling
# of src/ -- the driver locates the SDK by walking up from its own source
# dir to the workspace root and adding third_party to sys.path
# (rebotarmcontroller/hardware_config.py). No pip install of the SDK needed.
cd ~/stack/rebotarm_ros2
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# Workspace 2: teleop + demo station (needs ws1 sourced for rebotarm_msgs)
cd ~/stack/teleop
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Add to `~/.bashrc` (after the /opt/ros line):

```bash
source ~/stack/rebotarm_ros2/install/setup.bash
source ~/stack/teleop/install/setup.bash
```

Both are `--symlink-install`: **editing Python source takes effect on next
node start with no rebuild.** Rebuild only when adding files/entry points.

## 5. Verify (no robot needed)

```bash
# imports that must all succeed:
python3 -c "import rclpy, pinocchio, fastapi; print('ok')"
python3 -c "from rebotarmcontroller import hardware_manager; print('driver imports ok')"

# teleop dry-run preflight (fails on 'Quest pose stream' only -- that is correct
# with no headset connected; everything else should PASS):
ros2 run q2r2_bringup QuestTeleopReal --ros-args -p dry_run:=true

# demo GUI:
ros2 run demo_station demo_gui   # then open http://localhost:8800
```

## 6. Robot-host-only steps (skip on a plain desktop)

```bash
# CAN (PCAN-USB adapter, 1 Mbit) -- per boot:
sudo ip link set can1 up type can bitrate 1000000    # see setup/can-up.sh
# The channel name follows the adapter, not the machine: check `ip -br link`.

# RealSense camera(s): plug in, then configure serial numbers in
# teleop/src/demo_station/config/station.yaml (cameras: list).
```

Then follow `teleop/README.md` §2 to run the full stack, or start
`demo_gui` and drive everything from the browser.

## 7. Two machines (desktop + robot host on one network)

ROS 2 discovers peers by multicast on the same subnet with the same
`ROS_DOMAIN_ID` (default 0, both machines). If topics don't appear across
machines, check firewalls (`sudo ufw status` — allow or disable on the LAN)
and that both are on the same L2 network. The Quest app connects to whichever
machine runs `ros_tcp_endpoint` — its IP is hard-coded as `ROS_IP` in
`teleop/src/ros_tcp_communication/launch/endpoint.py`; update it there.

---

# Critical nuances (read before touching the robot)

The long-form versions live in `teleop/README.md` §4–§8 and `docs/ARM.md`.
The ones that cause damage or lost days:

1. **`model:=rs` on every arm launch.** The vendor default `dm` is the wrong
   kinematics with sign-flipped limits for this arm.
2. **Never run two drivers.** Each runs its own 500 Hz MIT loop; two of them
   fight at kp=150 and the arm thrashes violently.
3. **The driver energizes motors the moment it connects** and MIT mode has
   **no firmware velocity ceiling** — torque is kp × position-error.
4. **Motors latch on unclean shutdown.** SIGKILL — or a *second* SIGINT
   during shutdown — leaves motors holding torque with no process running.
   One SIGINT only; the driver ignores extra signals during shutdown and the
   GUI signals the parent only (both fixed 2026-08-08). If a driver start
   then fails with "gripper did not enter mit mode", a wedged motor from an
   unclean stop is the likely cause: power-cycle the arm (support it — it
   drops).
5. **Gripper compliance is driver-side logic** (`close_gripper_grasp`):
   ramped close, stall detection, ~2 Nm bounded hold. Without it the MIT
   spring saturates at 14 Nm on anything grasped. Tuning: `docs/ARM.md` §6b.
6. **Quest app frame follows the head until aligned**: press **A+B** (right)
   / X+Y (left) with the controller in driving grip before driving, or
   translation axes depend on where the operator looks.
7. **Recording disk budget** (robot host): the Jetson eMMC sustains 60 MB/s;
   uncompressed RGB-D at 848×480×30 is ~61 MB/s *per camera* and thrashes
   the machine. 640×480×30 fits one camera. Details in
   `teleop/src/demo_station/README.md`.
8. **rosbag2 Humble silently records nothing** when given explicit topics
   AND `--regex` together. The recorder compiles one combined regex — keep
   it that way.
9. **Don't trust `driver_params.yaml`** (dead config, loaded by nothing) and
   don't trust READMEs older than the ones in this repo.
