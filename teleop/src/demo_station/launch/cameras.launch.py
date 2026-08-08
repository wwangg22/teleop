"""Launch every camera declared in config/station.yaml.

Each entry becomes one realsense2_camera node under /camera/<name>/.
Adding a camera = plug it in + add a station.yaml entry (name, serial,
profile). Non-RealSense cameras: add a `type` and extend _camera_actions.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _camera_actions():
    cfg_path = os.path.join(
        get_package_share_directory("demo_station"), "config", "station.yaml"
    )
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    rs_launch = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
    )

    actions = []
    for cam in cfg.get("cameras", []):
        name = cam["name"]
        ctype = cam.get("type", "realsense")
        if ctype != "realsense":
            actions.append(LogInfo(msg=(
                f"[demo_station] camera '{name}' has unsupported type "
                f"'{ctype}' -- skipped (extend cameras.launch.py)")))
            continue
        profile = str(cam.get("profile", "848x480x30"))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                "camera_name": name,
                "camera_namespace": "camera",
                "serial_no": f"_{cam['serial']}",
                # D405 serves color from the depth module; other RealSense
                # models use rgb_camera -- set both, extras are ignored.
                "depth_module.depth_profile": profile,
                "depth_module.color_profile": profile,
                "rgb_camera.color_profile": profile,
            }.items(),
        ))
    return actions


def generate_launch_description():
    return LaunchDescription(_camera_actions())
