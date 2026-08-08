#!/usr/bin/env bash
# Phase 5, step 1: ROS 2 Humble + reBot arm build dependencies
# Jetson Orin Nano / Ubuntu 22.04 arm64
set -euo pipefail

echo "==> Adding ROS 2 apt repository"
apt-get update -qq
apt-get install -y software-properties-common curl gnupg
add-apt-repository -y universe
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list
apt-get update -qq

echo "==> Installing ros-humble-desktop (~2 GB, this is the slow part)"
apt-get install -y ros-humble-desktop

echo "==> Installing build tools"
apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  git

echo "==> Installing reBot driver dependencies"
apt-get install -y \
  ros-humble-control-msgs \
  ros-humble-trajectory-msgs \
  ros-humble-tf-transformations \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-pinocchio \
  ros-humble-xacro

echo "==> Installing MoveIt 2 + ros2_control"
apt-get install -y \
  ros-humble-moveit \
  ros-humble-moveit-configs-utils \
  ros-humble-moveit-kinematics \
  ros-humble-moveit-planners-ompl \
  ros-humble-moveit-simple-controller-manager \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers

echo "==> rosdep init"
rosdep init 2>/dev/null || echo "    (already initialized, skipping)"

echo "==> can-utils + CAN bring-up permissions"
apt-get install -y can-utils
# Let willy bring CAN interfaces up without a password prompt every session
cat > /etc/sudoers.d/99-can-willy <<'EOF'
willy ALL=(ALL) NOPASSWD: /usr/sbin/ip link set can*, /usr/sbin/ip link set dev can*
EOF
chmod 440 /etc/sudoers.d/99-can-willy

echo "==> Adding willy to docker + dialout groups"
usermod -aG docker,dialout willy

echo
echo "=== DONE ==="
echo "Log out and back in (or reboot) for the group changes to take effect."
