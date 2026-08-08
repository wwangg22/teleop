#!/usr/bin/env python3
"""Episode recorder for teleop demo collection.

Side-squeezing the driving controller (press_middle > trigger threshold)
toggles recording. Each episode is one raw, uncompressed rosbag2 bag
(MCAP when the plugin is installed, sqlite3 otherwise) plus an info.json,
under <demos_root>/YYYY-MM-DD/ep_HHMMSS/.

Design rules, per the collection plan:
- NOTHING is ever discarded or deleted, by anyone, for any reason. Stop
  reasons are recorded in info.json, never acted on.
- No live resampling/alignment: every stream keeps its publisher's header
  stamps; training-time alignment happens offline.
- The 2 Hz status topic reports bytes actually on disk, so a "REC" light
  driven by it is proof of writing, not intent.
"""

from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

from quest2ros.msg import OVR2ROSInputs


def _default_config() -> str:
    return os.path.join(
        get_package_share_directory("demo_station"), "config", "station.yaml"
    )


def detect_storage() -> str:
    """mcap if its rosbag2 plugin is installed, else sqlite3."""
    try:
        get_package_share_directory("rosbag2_storage_mcap")
        return "mcap"
    except Exception:
        return "sqlite3"


def dir_bytes(path: Path) -> int:
    total = 0
    try:
        for root, _dirs, files in os.walk(path):
            for f in files:
                try:
                    total += os.path.getsize(os.path.join(root, f))
                except OSError:
                    pass
    except OSError:
        pass
    return total


class DemoRecorder(Node):
    def __init__(self) -> None:
        super().__init__("demo_recorder")

        self.declare_parameter("config", _default_config())
        cfg_path = str(self.get_parameter("config").value)
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)

        self.demos_root = Path(cfg["demos_root"]).expanduser()
        self.demos_root.mkdir(parents=True, exist_ok=True)
        side = cfg.get("side", "right")
        toggle = cfg.get("toggle", {})
        self._trigger = float(toggle.get("trigger", 0.8))
        self._rearm = float(toggle.get("rearm", 0.3))
        self._min_gap = float(toggle.get("min_gap_s", 1.0))
        record = cfg.get("record", {})
        self._topics = list(record.get("topics", []))
        self._regex = str(record.get("regex", "") or "")
        self._storage = detect_storage()

        # toggle state
        self._armed = True
        self._last_toggle = 0.0

        # episode state
        self._proc: subprocess.Popen | None = None
        self._episode_dir: Path | None = None
        self._episode_name: str | None = None
        self._started_at: float | None = None
        self._info: dict = {}
        self._last_bytes = 0
        self._last_bytes_t = 0.0
        self._bytes_rate = 0.0
        self._last_episode: str | None = None

        self.create_subscription(
            OVR2ROSInputs,
            f"/q2r_{side}_hand_inputs",
            self._on_inputs,
            qos_profile_sensor_data,
        )
        self._status_pub = self.create_publisher(
            String, "/demo_station/recorder_status", 10
        )
        self.create_timer(0.5, self._publish_status)
        self.create_service(
            Trigger, "/demo_station/start_recording",
            lambda req, res: self._service_toggle(res, want_recording=True))
        self.create_service(
            Trigger, "/demo_station/stop_recording",
            lambda req, res: self._service_toggle(res, want_recording=False))

        if self._storage == "sqlite3":
            self.get_logger().warn(
                "mcap storage plugin not installed -- recording sqlite3. "
                "For crash-safe bags: sudo apt-get install ros-humble-rosbag2-storage-mcap"
            )
        self.get_logger().info(
            f"demo_recorder ready: root={self.demos_root} storage={self._storage} "
            f"side={side} squeeze>{self._trigger} to toggle"
        )

    # ------------------------------------------------------------------
    # controller toggle
    # ------------------------------------------------------------------

    def _on_inputs(self, msg: OVR2ROSInputs) -> None:
        v = float(msg.press_middle)
        if self._armed and v >= self._trigger:
            now = time.monotonic()
            if now - self._last_toggle >= self._min_gap:
                self._last_toggle = now
                self._armed = False
                self._toggle(source="squeeze")
        elif not self._armed and v <= self._rearm:
            self._armed = True

    def _service_toggle(self, res, *, want_recording: bool):
        recording = self._proc is not None
        if recording == want_recording:
            res.success = False
            res.message = f"already {'recording' if recording else 'idle'}"
            return res
        self._toggle(source="api")
        res.success = True
        res.message = self._episode_name or (self._last_episode or "")
        return res

    def _toggle(self, source: str) -> None:
        if self._proc is None:
            self._start_episode(source)
        else:
            self._stop_episode(f"{source}")

    # ------------------------------------------------------------------
    # episode lifecycle
    # ------------------------------------------------------------------

    def _start_episode(self, source: str) -> None:
        now = datetime.now()
        day_dir = self.demos_root / now.strftime("%Y-%m-%d")
        name = "ep_" + now.strftime("%H%M%S")
        ep_dir = day_dir / name
        # never overwrite: suffix on collision instead
        i = 1
        while ep_dir.exists():
            ep_dir = day_dir / f"{name}_{i}"
            i += 1
        ep_dir.mkdir(parents=True)

        # Humble's rosbag2 SILENTLY records nothing when given explicit
        # topics AND --regex together (verified 2026-08-08), so compile
        # the whole manifest into one regex: each explicit topic anchored
        # exactly, OR'd with the user pattern.
        parts = [f"^{re.escape(t)}$" for t in self._topics]
        if self._regex:
            parts.append(self._regex)
        cmd = ["ros2", "bag", "record", "-s", self._storage,
               "-o", str(ep_dir / "bag"), "-e", "|".join(parts)]

        log = open(ep_dir / "bag_record.log", "w")
        self._proc = subprocess.Popen(
            cmd,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self._episode_dir = ep_dir
        self._episode_name = str(ep_dir.relative_to(self.demos_root))
        self._started_at = time.time()
        self._last_bytes = 0
        self._last_bytes_t = time.monotonic()
        self._bytes_rate = 0.0
        self._info = {
            "episode": self._episode_name,
            "started_at": now.isoformat(timespec="seconds"),
            "started_at_unix": self._started_at,
            "start_source": source,
            "storage": self._storage,
            "topics": self._topics,
            "regex": self._regex,
        }
        self._write_info()
        self._snapshot_params(ep_dir)
        self.get_logger().info(f"RECORDING -> {ep_dir}")

    def _stop_episode(self, reason: str) -> None:
        proc, ep_dir = self._proc, self._episode_dir
        self._proc = None
        if proc is not None and proc.poll() is None:
            try:
                # SIGINT the whole group so rosbag2 finalizes metadata.yaml
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("bag record slow to stop; SIGTERM")
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=5)
                except Exception:
                    pass
            except ProcessLookupError:
                pass
        if ep_dir is not None:
            self._info["stopped_at"] = datetime.now().isoformat(timespec="seconds")
            self._info["stop_reason"] = reason
            self._info["duration_s"] = (
                round(time.time() - self._started_at, 3) if self._started_at else None
            )
            self._info["bytes"] = dir_bytes(ep_dir)
            self._write_info()
            self.get_logger().info(
                f"SAVED {self._episode_name} "
                f"({self._info['duration_s']}s, {self._info['bytes']/1e6:.1f} MB, "
                f"stop: {reason})"
            )
        self._last_episode = self._episode_name
        self._episode_dir = None
        self._episode_name = None
        self._started_at = None

    def _write_info(self) -> None:
        if self._episode_dir is None:
            return
        tmp = self._episode_dir / "info.json.tmp"
        with open(tmp, "w") as f:
            json.dump(self._info, f, indent=2)
        tmp.rename(self._episode_dir / "info.json")

    def _snapshot_params(self, ep_dir: Path) -> None:
        """Best-effort dump of the teleop/driver params active during this
        episode (scales, gripper gains, ...). Failure is fine -- the nodes
        may simply not be up."""
        pdir = ep_dir / "params"
        pdir.mkdir(exist_ok=True)
        for node_name in ("/quest_teleop_real", "/reBotArmController"):
            out = pdir / (node_name.strip("/") + ".yaml")
            # timeout: `ros2 param dump` HANGS FOREVER if the node is down,
            # and its cmdline containing the node name then trips the driver
            # unit's singleton pgrep check ("already running elsewhere").
            subprocess.Popen(
                f"timeout 10 ros2 param dump {node_name} > {out} 2>/dev/null"
                f" || rm -f {out}",
                shell=True,
                start_new_session=True,
            )

    # ------------------------------------------------------------------
    # status
    # ------------------------------------------------------------------

    def _publish_status(self) -> None:
        recording = self._proc is not None
        if recording and self._proc.poll() is not None:
            # bag process died underneath us -- keep everything, say so
            code = self._proc.returncode
            self._stop_episode(f"bag_process_exit_{code}")
            recording = False

        status = {
            "state": "recording" if recording else "idle",
            "storage": self._storage,
            "episode": self._episode_name,
            "last_episode": self._last_episode,
        }
        if recording and self._episode_dir is not None:
            b = dir_bytes(self._episode_dir)
            now = time.monotonic()
            dt = now - self._last_bytes_t
            if dt >= 1.0:
                self._bytes_rate = (b - self._last_bytes) / dt
                self._last_bytes, self._last_bytes_t = b, now
            status["elapsed_s"] = round(time.time() - self._started_at, 1)
            status["bytes"] = b
            status["bytes_per_s"] = round(self._bytes_rate)
        try:
            st = os.statvfs(self.demos_root)
            status["disk_free_gb"] = round(st.f_bavail * st.f_frsize / 1e9, 1)
        except OSError:
            pass
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    def shutdown(self) -> None:
        if self._proc is not None:
            self._stop_episode("recorder_shutdown")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
