#!/usr/bin/env bash
# Build the reBot ROS 2 workspace.
#
# Two Jetson-specific deviations from the Seeed wiki:
#   1. Parallelism capped at 2 -- colcon defaults to one job per core, and
#      6 concurrent MoveIt/C++ compiles will OOM an 8 GB Orin Nano.
#   2. Conda is stripped from PATH. Miniforge base is Python 3.13; ROS 2
#      Humble is built against system Python 3.10. If conda leaks into the
#      build or runtime, `import pinocchio` fails and nodes crash on startup.
# NOTE: no `set -u` -- ROS 2's setup.bash references unset variables
# (AMENT_TRACE_SETUP_FILES) and dies under nounset.
set -eo pipefail

WS=/home/willy/rebotarm_ros2

# --- Strip conda from this shell entirely -------------------------------
PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniforge3 | paste -sd: -)
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_EXE PYTHONPATH
export PATH
echo "==> python3 is now: $(command -v python3) ($(python3 --version 2>&1))"
if [[ "$(command -v python3)" != "/usr/bin/python3" ]]; then
  echo "!! Expected /usr/bin/python3. Aborting rather than building against the wrong interpreter."
  exit 1
fi

source /opt/ros/humble/setup.bash

echo "==> Sanity check: pinocchio under system python"
python3 -c "import pinocchio; print('    pinocchio', pinocchio.__version__)"

# --- Python deps for the ROS nodes (system python, NOT the conda env) ----
# The SDK itself is NOT pip-installed: rebotarmcontroller/hardware_config.py
# locates third_party/reBotArm_control_py and sys.path.insert()s it at runtime.
# It also can't be pip-installed -- its pyproject.toml has no [build-system]
# and a flat layout setuptools auto-discovery chokes on.
#
# We install its deps manually, MINUS 'pin' (the pip pinocchio): ROS already
# ships ros-humble-pinocchio 4.0.0, and a second copy risks shadowing it.
# numpy is pinned to EXACTLY 1.23.5 -- a narrow window:
#   >= 1.23.4  required by the reBotArm SDK
#   <  1.24    because numpy 1.24 removed np.float, which Ubuntu's packaged
#              transforms3d still uses -> tf_transformations import fails
#              -> the driver node dies on startup.
# Do not "upgrade" this without checking transforms3d.
# setuptools is held <80 -- colcon-core requires it.
echo "==> Installing python deps into system python3 --user"
python3 -m pip install --user --upgrade motorbridge
python3 -m pip install --user "setuptools<80" "numpy==1.23.5" meshcat pyyaml matplotlib

# --- Build --------------------------------------------------------------
cd "$WS"
echo "==> colcon build (2 parallel workers; expect 15-30 min on an Orin Nano)"
colcon build --symlink-install \
  --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "==> Build complete. Executables in rebotarmcontroller:"
source install/setup.bash
ros2 pkg executables rebotarmcontroller

cat <<'EOF'

=== NEXT ===
  ~/rebot-setup/can-up.sh
  ros2 launch rebotarm_bringup bringup.launch.py model:=rs channel:=can1

NOTE: channel:=can1 -- the PEAK adapter is can1, not the can0 default.
NOTE: run ROS in a shell WITHOUT conda active (conda auto-activate is now off).
EOF
