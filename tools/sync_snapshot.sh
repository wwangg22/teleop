#!/usr/bin/env bash
# Re-sync this repo from the live workspaces on the robot host, then review
# and commit. Run FROM THE REPO ROOT on the machine that runs the robot.
# This repo is a snapshot: the live workspaces are the source of truth.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
EXC=(--exclude=.git --exclude=build/ --exclude=install/ --exclude=log/
     --exclude=__pycache__/ --exclude='*.pyc' --exclude=.claude)

rsync -a --delete "${EXC[@]}" ~/Desktop/teleop/src ~/Desktop/teleop/README.md "$REPO/teleop/"
rsync -a --delete "${EXC[@]}" ~/rebotarm_ros2/src ~/rebotarm_ros2/third_party \
      ~/rebotarm_ros2/media ~/rebotarm_ros2/README.md ~/rebotarm_ros2/README_zh.md \
      ~/rebotarm_ros2/API_zh.md "$REPO/rebotarm_ros2/"
cp ~/rebot-arm-private/README_ARM.md "$REPO/docs/ARM.md"
cp ~/rebot-arm-private/HANDOFF.md   "$REPO/docs/HANDOFF.md"
cp ~/rebot-setup/*.sh "$REPO/setup/"

cd "$REPO"
git status --short
echo "Review the diff, then: git add -A && git commit && git push"
