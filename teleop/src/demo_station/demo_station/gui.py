#!/usr/bin/env python3
"""Demo station web GUI.

FastAPI + one rclpy node in a background thread. Serves:
- process manager (units.yaml) with per-topic health
- live camera tiles (MJPEG, encoded only while watched)
- recorder state (big REC banner driven by bytes-on-disk status)
- read-only demo browser with in-bag video playback and joint traces

Run: ros2 run demo_station demo_gui   (then open http://<jetson>:8800)
"""

from __future__ import annotations

import argparse
import json
import os
import threading
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI
from fastapi.responses import (FileResponse, JSONResponse, PlainTextResponse,
                               StreamingResponse)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from . import bag_utils
from .health import ImageHub, TopicHealth, decode_image
from .procman import ProcessManager

try:
    import cv2
except ImportError:
    cv2 = None

SHARE = Path(get_package_share_directory("demo_station"))
BOUNDARY = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"


class GuiNode(Node):
    def __init__(self, station_cfg: dict, units_cfg: list[dict]) -> None:
        super().__init__("demo_gui")
        self.station = station_cfg
        self.camera_names = [c["name"] for c in station_cfg.get("cameras", [])]

        self.latest: dict[str, dict] = {"recorder": {}, "teleop": {}}
        self._latest_stamp: dict[str, float] = {}
        self.create_subscription(
            String, "/demo_station/recorder_status",
            lambda m: self._on_json("recorder", m), 10)
        self.create_subscription(
            String, "/quest_teleop_real/diagnostics",
            lambda m: self._on_json("teleop", m), 10)

        self.health = TopicHealth(self)
        for u in units_cfg:
            for h in u.get("health", []) or []:
                self.health.watch(h["topic"])
        self.images = ImageHub(self, self.camera_names)

        self._start_cli = self.create_client(
            Trigger, "/demo_station/start_recording")
        self._stop_cli = self.create_client(
            Trigger, "/demo_station/stop_recording")

    def _on_json(self, key: str, msg: String) -> None:
        try:
            self.latest[key] = json.loads(msg.data)
            self._latest_stamp[key] = time.monotonic()
        except json.JSONDecodeError:
            pass

    def fresh(self, key: str, max_age: float = 4.0) -> dict:
        if time.monotonic() - self._latest_stamp.get(key, 0.0) > max_age:
            return {}
        return self.latest.get(key, {})

    def call_recorder(self, start: bool) -> dict:
        cli = self._start_cli if start else self._stop_cli
        if not cli.service_is_ready():
            return {"success": False, "message": "recorder not running"}
        future = cli.call_async(Trigger.Request())
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            return {"success": False, "message": "recorder did not answer"}
        res = future.result()
        return {"success": res.success, "message": res.message}


def build_app(node: GuiNode, pm: ProcessManager, units_cfg: list[dict]) -> FastAPI:
    app = FastAPI(title="demo_station")
    demos_root = Path(node.station["demos_root"]).expanduser()

    @app.get("/")
    def index():
        return FileResponse(SHARE / "web" / "index.html")

    # -------------------------------------------------- status

    @app.get("/api/status")
    def status():
        units = []
        for u in units_cfg:
            s = pm.status(u["name"])
            checks = []
            for h in u.get("health", []) or []:
                checks.append({
                    "topic": h["topic"],
                    "hz": node.health.hz(h["topic"]),
                    "min_hz": h.get("min_hz", 1),
                })
            if u["name"] == "cameras":
                for key, hz in node.images.rates().items():
                    checks.append({"topic": f"/camera/{key}",
                                   "hz": hz, "min_hz": 5})
            s["health"] = checks
            units.append(s)
        try:
            st = os.statvfs(demos_root)
            disk = {"free_gb": round(st.f_bavail * st.f_frsize / 1e9, 1),
                    "total_gb": round(st.f_blocks * st.f_frsize / 1e9, 1)}
        except OSError:
            disk = {}
        return {
            "units": units,
            "recorder": node.fresh("recorder"),
            "teleop": node.fresh("teleop"),
            "disk": disk,
            "cameras": node.camera_names,
            "load": os.getloadavg()[0],
        }

    # -------------------------------------------------- process control

    @app.post("/api/units/{name}/start")
    def unit_start(name: str):
        unit = pm.units.get(name)
        if unit is None:
            return JSONResponse({"ok": False, "message": "unknown unit"}, 404)
        if unit.type == "check":
            return {"ok": True, **pm.run_check(name)}
        ok, msg = pm.start(name)
        return {"ok": ok, "message": msg}

    @app.post("/api/units/{name}/stop")
    def unit_stop(name: str):
        ok, msg = pm.stop(name)
        return {"ok": ok, "message": msg}

    @app.post("/api/units/{name}/stdin")
    async def unit_stdin(name: str, body: dict):
        ok, msg = pm.send_stdin(name, str(body.get("text", "")))
        return {"ok": ok, "message": msg}

    @app.get("/api/units/{name}/log", response_class=PlainTextResponse)
    def unit_log(name: str, lines: int = 120):
        return pm.log_tail(name, lines)

    # -------------------------------------------------- recorder control

    @app.post("/api/recorder/{action}")
    def recorder_action(action: str):
        if action not in ("start", "stop"):
            return JSONResponse({"success": False}, 404)
        return node.call_recorder(start=(action == "start"))

    # -------------------------------------------------- live streams

    def _mjpeg_live(key: str):
        while True:
            frame = node.images.jpeg(key)
            if frame is not None:
                yield BOUNDARY + frame + b"\r\n"
            time.sleep(0.1)  # ~10 fps to the browser

    @app.get("/stream/{camera}/{kind}")
    def stream(camera: str, kind: str):
        return StreamingResponse(
            _mjpeg_live(f"{camera}/{kind}"),
            media_type="multipart/x-mixed-replace; boundary=frame")

    # -------------------------------------------------- demo browser

    @app.get("/api/episodes")
    def episodes():
        return bag_utils.list_episodes(demos_root)

    def _safe_ep_dir(date: str, name: str) -> Path | None:
        p = (demos_root / date / name).resolve()
        if not str(p).startswith(str(demos_root.resolve())) or not p.is_dir():
            return None
        return p

    @app.get("/api/episodes/{date}/{name}/joints")
    def ep_joints(date: str, name: str):
        ep = _safe_ep_dir(date, name)
        if ep is None:
            return JSONResponse({}, 404)
        return bag_utils.joint_trace(ep / "bag")

    def _mjpeg_playback(bag_dir: Path, topic: str, speed: float):
        last_t = None
        last_emit = 0.0
        for t_ns, msg in bag_utils.iter_topic(bag_dir, topic):
            if last_t is not None:
                dt = (t_ns - last_t) / 1e9 / max(speed, 0.1)
                if dt > 0:
                    time.sleep(min(dt, 0.5))
            last_t = t_ns
            now = time.monotonic()
            if now - last_emit < 1.0 / 15.0:  # cap browser rate
                continue
            img = decode_image(msg)
            if img is None or cv2 is None:
                continue
            ok, buf = cv2.imencode(".jpg", img,
                                   [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                last_emit = now
                yield BOUNDARY + buf.tobytes() + b"\r\n"

    @app.get("/playback/{date}/{name}")
    def playback(date: str, name: str, topic: str, speed: float = 1.0):
        ep = _safe_ep_dir(date, name)
        if ep is None:
            return JSONResponse({}, 404)
        return StreamingResponse(
            _mjpeg_playback(ep / "bag", topic, speed),
            media_type="multipart/x-mixed-replace; boundary=frame")

    return app


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8800)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--config", default=str(SHARE / "config" / "station.yaml"))
    parser.add_argument("--units", default=str(SHARE / "config" / "units.yaml"))
    args, _ros_args = parser.parse_known_args()

    with open(args.config) as f:
        station_cfg = yaml.safe_load(f)
    with open(args.units) as f:
        units_cfg = yaml.safe_load(f)["units"]

    rclpy.init()
    node = GuiNode(station_cfg, units_cfg)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    pm = ProcessManager(units_cfg)
    app = build_app(node, pm, units_cfg)

    import uvicorn
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    finally:
        pm.shutdown()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
