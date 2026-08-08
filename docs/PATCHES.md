# rebot-arm-private

**PRIVATE.** Work on a Seeed reBot Arm B601-RS running on a Jetson Orin Nano.

This repo holds *our* changes to two upstream repos we cannot push to, plus the
setup scripts and operating notes. It deliberately does **not** contain a copy
of the upstream code — only patches — so it stays small and it is always obvious
which lines are ours.

## Layout

| Path | What |
|---|---|
| `README_ARM.md` | **The main document.** Hardware facts, how to run the arm, safety behaviours, control-loop internals, every local modification, debugging lessons, and how the policy executor works. |
| `HANDOFF.md` | Session state snapshot and prioritised next steps. |
| `patches/rebotarm_ros2/` | Our commits against `Seeed-Projects/reBotArmController_ROS2` |
| `patches/reBotArm_control_py/` | Our commits against `vectorBH6/reBotArm_control_py` |
| `setup/` | Helper scripts (CAN bring-up, build, PCAN reset, ROS 2 install) |
| `restore.sh` | Re-clone upstream and re-apply our patches |

## Restoring a working checkout

```bash
./restore.sh              # clones upstream, applies our branches
```

## Updating this repo after further work

```bash
cd ~/rebotarm_ros2
git format-patch main..jetson-setup-and-policy-executor \
    -o ~/rebot-arm-private/patches/rebotarm_ros2

cd ~/rebotarm_ros2/third_party/reBotArm_control_py
git format-patch main..expose-motor-temperature \
    -o ~/rebot-arm-private/patches/reBotArm_control_py

cp ~/rebot-setup/README.md  ~/rebot-arm-private/README_ARM.md
cp ~/rebot-setup/HANDOFF.md ~/rebot-arm-private/HANDOFF.md
cp ~/rebot-setup/*.sh       ~/rebot-arm-private/setup/
```

Delete stale patch files first if commits were rebased or squashed — otherwise
`restore.sh` will try to apply both the old and new versions.

## Upstream

| Repo | License | Notes |
|---|---|---|
| [reBotArmController_ROS2](https://github.com/Seeed-Projects/reBotArmController_ROS2) | Apache-2.0 | ROS 2 driver + MoveIt config |
| [reBotArm_control_py](https://github.com/vectorBH6/reBotArm_control_py) | see repo | kinematics/dynamics SDK |

The SDK patch (exposing motor temperature) is self-contained and would be
reasonable to offer upstream. The `hardware.launch.py` import fix in the
workspace patch is a genuine upstream bug — that file cannot launch on Humble
without it — and is also worth reporting.

## ⚠️ Keep this repo private

If you push it to GitHub, create the remote as **private**. A repo made public
cannot be reliably un-published — forks and caches persist.
