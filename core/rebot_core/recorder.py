"""MCAP episode recorder without ROS (plan.md Phase 3).

Replaces the rosbag2 subprocess with an in-process MCAP writer. Ported
intact from demo_station/recorder.py: episode dir layout with collision
suffixes, atomic info.json, stop reasons recorded-never-acted-on, the
bytes-ON-DISK counter ("proof of writing, not intent"), squeeze-toggle
hysteresis, params snapshot, NOTHING IS EVER DELETED.

Storage decisions (recorded in info.json):
- structured channels: JSON messages, jsonschema-lite schemas ("json" enc);
- camera color: raw JPEG bytes per message (message_encoding "jpeg");
- camera depth: raw little-endian z16 frames (message_encoding "z16",
  dimensions in channel metadata). ~18 MB/s per 640x480x30 camera -- nothing
  for this machine's NVMe; revisit (PNG) only if disk ever binds again.
- log_time = wall clock ns; every JSON payload also carries t_mono for
  in-process alignment. Raw stamps per stream; align OFFLINE only.

The clean-close flag replaces rosbag2's metadata.yaml-presence convention:
info.json gains "clean_close": true only on a successful finalize.
"""

from __future__ import annotations

import hashlib
import json
import logging
import os
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from mcap.writer import Writer

from .cameras import CameraRig, encode_jpeg
from .quest_link import _RateMeter

log = logging.getLogger("recorder")

SCHEMA_VERSION = 1

# docs/DATA.md §3: units and conventions travel WITH the data, in-band.
CONVENTIONS = {
    "schema_version": SCHEMA_VERSION,
    "angles": "radians",
    "angular_velocity": "rad/s",
    "torque": "Nm",
    "depth_encoding": "z16 raw little-endian; meters = value * depth_scale_m",
    "quaternion_order": "xyzw",
    "base_frame": "base_link",
    "ee_frame": "gripper_end",
    "joint_order": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    "clocks": {
        "log_time": "unix wall ns, host clock, all channels",
        "t_wall": "unix wall seconds (same clock as log_time)",
        "t_mono": "host monotonic seconds; ordering/latency only, resets on boot",
        "t_device": "RealSense global-time ms (disciplined to host clock)",
    },
    "alignment": "match streams on log_time/t_wall; NEVER assume fixed dt -- "
                 "read per-sample stamps (control rates may change between "
                 "episodes)",
}


def dir_bytes(path: Path) -> int:
    total = 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total


def _atomic_write_json(path: Path, data: dict) -> None:
    tmp = path.with_suffix(".json.tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, sort_keys=True)
    os.replace(tmp, path)


class RecordToggle:
    """press_middle squeeze-toggle with hysteresis (fire >=0.8, re-arm <=0.3,
    1.0 s min gap) -- the operator contract, ported verbatim."""

    def __init__(
        self, trigger: float = 0.8, rearm: float = 0.3, min_gap_s: float = 1.0
    ) -> None:
        self.trigger = trigger
        self.rearm = rearm
        self.min_gap_s = min_gap_s
        self._armed = True
        self._last_fire = 0.0

    def resync(self, press_middle: float) -> None:
        """After a stream gap: adopt current state without firing (a squeeze
        that happened during the outage must not toggle on reconnect)."""
        self._armed = press_middle <= self.rearm

    def update(self, press_middle: float, now_mono: float | None = None) -> bool:
        """Returns True exactly when a toggle should fire."""
        now = time.monotonic() if now_mono is None else now_mono
        if self._armed and press_middle >= self.trigger:
            if now - self._last_fire >= self.min_gap_s:
                self._armed = False
                self._last_fire = now
                return True
            return False
        if press_middle <= self.rearm:
            self._armed = True
        return False


@dataclass
class _Channel:
    channel_id: int
    count: int = 0
    rate: _RateMeter = None

    def __post_init__(self):
        if self.rate is None:
            self.rate = _RateMeter()


class EpisodeRecorder:
    """One recorder; at most one live episode at a time."""

    STRUCTURED_TOPICS = (
        "/robot/joint_states",
        "/robot/target",
        "/robot/gripper",
        "/quest/right/pose",
        "/quest/right/inputs",
        "/quest/left/pose",
        "/quest/left/inputs",
        "/teleop/diagnostics",
        "/health/alerts",
        "/episode/marks",
        # policy-rollout insertion (contract C3, policy_bridge.py): channels
        # exist on every episode but carry messages only when a PolicyBridge
        # writes them -- inert during ordinary teleop (count 0), same as the
        # unused-hand /quest topics. write_json drops unregistered topics
        # silently, so registration must happen here.
        "/policy/obs_proprio",
        "/policy/action",
        "/policy/chunk_meta",
    )

    def __init__(
        self,
        demos_root: str | Path,
        rig: CameraRig | None = None,
        params_provider=None,          # () -> dict, snapshotted per episode
        metadata_provider=None,        # () -> dict merged into info.json
                                       #   (rates, provenance, calibration)
        attachments_provider=None,     # () -> [(name, media_type, bytes)]
                                       #   embedded into the mcap (URDF, config)
        jpeg_quality: int = 90,
    ) -> None:
        self.demos_root = Path(demos_root).expanduser()
        self.rig = rig
        self.params_provider = params_provider
        self.metadata_provider = metadata_provider
        self.attachments_provider = attachments_provider
        self.jpeg_quality = jpeg_quality

        self._lock = threading.Lock()
        # Serializes whole start/stop transitions (the inner _lock guards
        # state, but stop() must release it to join threads -- two stops
        # racing through that window once crashed finalize on a None writer,
        # triggered by TCP-replayed squeeze toggles after a WiFi stall).
        self._transition_lock = threading.Lock()
        self._writer: Writer | None = None
        self._file = None
        self._channels: dict[str, _Channel] = {}
        self._episode_dir: Path | None = None
        self._episode_name: str | None = None
        self._last_episode: str | None = None
        self._start_mono = 0.0
        self._start_wall = 0.0
        self._start_source = ""
        self._bytes = 0
        self._bytes_rate = 0.0
        self._bytes_prev: tuple[float, int] | None = None
        self._write_q: queue.Queue = queue.Queue(maxsize=512)
        self._threads: list[threading.Thread] = []
        self._halt = threading.Event()
        self._drops_write_q = 0

    # -- lifecycle ------------------------------------------------------

    @property
    def recording(self) -> bool:
        return self._writer is not None

    def start(self, source: str = "api") -> tuple[bool, str]:
        if not self._transition_lock.acquire(blocking=False):
            return False, "start/stop already in progress"
        try:
            return self._start_inner(source)
        finally:
            self._transition_lock.release()

    def _start_inner(self, source: str) -> tuple[bool, str]:
        with self._lock:
            if self.recording:
                return False, "already recording"
            day = time.strftime("%Y-%m-%d")
            base = time.strftime("ep_%H%M%S")
            ep_dir = self.demos_root / day / base
            n = 0
            while ep_dir.exists():          # never overwrite
                n += 1
                ep_dir = self.demos_root / day / f"{base}_{n}"
            ep_dir.mkdir(parents=True)
            (ep_dir / "params").mkdir()

            self._episode_dir = ep_dir
            self._episode_name = ep_dir.name
            self._start_mono = time.monotonic()
            self._start_wall = time.time()
            self._start_source = source
            self._bytes = 0
            self._bytes_rate = 0.0
            self._bytes_prev = None
            self._drops_write_q = 0
            self._halt.clear()

            self._file = open(ep_dir / "episode.mcap", "wb")
            self._writer = Writer(self._file)
            self._writer.start(profile="", library="rebot_core-recorder")
            self._channels = {}
            json_schema = self._writer.register_schema(
                "rebot_core/json", "jsonschema", b'{"type": "object"}'
            )
            for topic in self.STRUCTURED_TOPICS:
                cid = self._writer.register_channel(topic, "json", json_schema)
                self._channels[topic] = _Channel(cid)
            cam_meta: dict[str, str] = {}
            if self.rig is not None:
                none_schema = self._writer.register_schema("raw", "none", b"")
                for name, cam in self.rig.cameras.items():
                    p = cam.profile
                    ccid = self._writer.register_channel(
                        f"/camera/{name}/color", "jpeg", none_schema,
                        metadata={"width": str(p.width), "height": str(p.height)},
                    )
                    dcid = self._writer.register_channel(
                        f"/camera/{name}/depth", "z16", none_schema,
                        metadata={"width": str(p.width), "height": str(p.height),
                                  "dtype": "uint16le"},
                    )
                    self._channels[f"/camera/{name}/color"] = _Channel(ccid)
                    self._channels[f"/camera/{name}/depth"] = _Channel(dcid)
                    # per-frame precision stamps sidecar (docs/DATA.md §2):
                    # seq + t_wall + t_mono + RealSense t_device per frame.
                    scid = self._writer.register_channel(
                        f"/camera/{name}/stamps", "json", json_schema
                    )
                    self._channels[f"/camera/{name}/stamps"] = _Channel(scid)
                    cam_meta[name] = str(p)

            # In-band artifacts (docs/DATA.md §3): the episode must be
            # interpretable from its own bytes, years out.
            attachments: list[str] = []
            if self.attachments_provider is not None:
                try:
                    for name, media_type, data in self.attachments_provider():
                        self._writer.add_attachment(
                            log_time=int(self._start_wall * 1e9),
                            create_time=int(self._start_wall * 1e9),
                            name=name,
                            media_type=media_type,
                            data=data,
                        )
                        attachments.append(name)
                except Exception as exc:
                    log.error("attachment embed failed: %s", exc)

            info = {
                "episode": self._episode_name,
                "schema_version": SCHEMA_VERSION,
                "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "started_at_unix": self._start_wall,
                "start_source": source,
                "storage": "mcap",
                "encodings": {"structured": "json", "color": "jpeg",
                              "depth": "z16-raw"},
                "cameras": cam_meta,
                "conventions": CONVENTIONS,
                "attachments": attachments,
            }
            if self.metadata_provider is not None:
                try:
                    info.update(self.metadata_provider())
                except Exception as exc:
                    log.error("metadata provider failed: %s", exc)
            _atomic_write_json(ep_dir / "info.json", info)
            self._snapshot_params(ep_dir)

            t = threading.Thread(target=self._writer_loop, name="rec-writer",
                                 daemon=True)
            t.start()
            self._threads = [t]
            if self.rig is not None:
                for name, cam in self.rig.cameras.items():
                    cq = cam.attach_recorder()
                    ct = threading.Thread(
                        target=self._camera_pump, args=(name, cam, cq),
                        name=f"rec-cam-{name}", daemon=True,
                    )
                    ct.start()
                    self._threads.append(ct)
            bt = threading.Thread(target=self._bytes_loop, name="rec-bytes",
                                  daemon=True)
            bt.start()
            self._threads.append(bt)

            log.info("recording %s (source=%s)", ep_dir, source)
            return True, self._episode_name

    def stop(self, reason: str = "api") -> tuple[bool, str]:
        if not self._transition_lock.acquire(blocking=False):
            return False, "start/stop already in progress"
        try:
            return self._stop_inner(reason)
        finally:
            self._transition_lock.release()

    def _stop_inner(self, reason: str) -> tuple[bool, str]:
        with self._lock:
            if not self.recording:
                return False, "already idle"
            name = self._episode_name or ""
            ep_dir = self._episode_dir
            self._halt.set()
            if self.rig is not None:
                for cam in self.rig.cameras.values():
                    cam.detach_recorder()
        # join writer threads outside the lock. The rec-writer thread drains
        # the write backlog before exiting; at 2-camera 720p-depth rates that
        # backlog can approach 1 GB (512-slot queue), far beyond a 5 s join.
        # finish()ing the (non-thread-safe) mcap Writer while add_message is
        # still running wedges the whole stop -- the "stop button does
        # nothing" hang -- so the writer MUST be joined to completion (hard
        # cap below), and on cap-expiry we skip finalize instead of racing.
        writer_thread = self._threads[0] if self._threads else None
        for t in self._threads[1:]:
            t.join(timeout=5.0)
        if writer_thread is not None:
            deadline = time.monotonic() + 60.0
            while writer_thread.is_alive() and time.monotonic() < deadline:
                writer_thread.join(timeout=2.0)
                if writer_thread.is_alive():
                    log.info(
                        "stop: draining write backlog (%d queued)",
                        self._write_q.qsize(),
                    )
        drain_failed = writer_thread is not None and writer_thread.is_alive()
        with self._lock:
            counts = {k: c.count for k, c in self._channels.items()}
            cam_drops = (
                {n: c.stats.drops_recorder_queue
                 for n, c in self.rig.cameras.items()}
                if self.rig is not None else {}
            )
            try:
                if drain_failed:
                    # Writer still mid-add_message after the cap: do NOT
                    # touch the Writer/file from this thread (corruption /
                    # deadlock). MCAP chunk CRCs make the truncated file
                    # partially recoverable.
                    raise RuntimeError(
                        "writer thread still draining after 60 s -- "
                        "skipping finalize (episode left truncated)"
                    )
                self._writer.finish()
                self._file.close()
                clean = True
            except Exception as exc:
                log.error("mcap finalize failed: %s", exc)
                clean = False
            self._writer = None
            self._file = None

            # Integrity digest (docs/DATA.md §5): catches silent bit-rot and
            # proves completeness of copies/backups.
            sha256 = None
            if clean:
                try:
                    h = hashlib.sha256()
                    with open(ep_dir / "episode.mcap", "rb") as f:
                        for block in iter(lambda: f.read(1 << 20), b""):
                            h.update(block)
                    sha256 = h.hexdigest()
                except OSError as exc:
                    log.error("sha256 failed: %s", exc)

            info_path = ep_dir / "info.json"
            try:
                info = json.loads(info_path.read_text())
            except Exception:
                info = {}
            info.update({
                "stopped_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "stop_reason": reason,
                "duration_s": round(time.monotonic() - self._start_mono, 3),
                "bytes": dir_bytes(ep_dir),
                "message_counts": counts,
                "camera_queue_drops": cam_drops,
                "write_queue_drops": self._drops_write_q,
                "clean_close": clean,
                "episode_mcap_sha256": sha256,
            })
            _atomic_write_json(info_path, info)
            self._last_episode = name
            self._episode_dir = None
            self._episode_name = None
            log.info("stopped %s (reason=%s, clean=%s)", name, reason, clean)
            return True, name

    def toggle(self, source: str) -> tuple[bool, str]:
        if self.recording:
            return self.stop(reason=source)
        return self.start(source=source)

    # -- data in --------------------------------------------------------

    def write_json(self, topic: str, payload: dict) -> None:
        """Thread-safe structured write; no-op when idle. Payload gets t_mono
        and t_wall added if missing."""
        if not self.recording:
            return
        payload.setdefault("t_mono", time.monotonic())
        payload.setdefault("t_wall", time.time())
        data = json.dumps(payload).encode("utf-8")
        self._enqueue(topic, int(payload["t_wall"] * 1e9), data)

    def _enqueue(self, topic: str, log_time_ns: int, data: bytes) -> None:
        try:
            self._write_q.put_nowait((topic, log_time_ns, data))
        except queue.Full:
            self._drops_write_q += 1

    # -- internals ------------------------------------------------------

    def _writer_loop(self) -> None:
        while True:
            try:
                topic, log_time_ns, data = self._write_q.get(timeout=0.25)
            except queue.Empty:
                if self._halt.is_set():
                    return
                continue
            ch = self._channels.get(topic)
            w = self._writer
            if ch is None or w is None:
                continue
            try:
                w.add_message(ch.channel_id, log_time_ns, data, log_time_ns,
                              ch.count)
                ch.count += 1
                ch.rate.tick(time.monotonic())
            except Exception as exc:
                log.error("mcap write failed on %s: %s", topic, exc)

    def _camera_pump(self, name: str, cam, cq: queue.Queue) -> None:
        while not self._halt.is_set():
            try:
                slot = cq.get(timeout=0.25)
            except queue.Empty:
                continue
            ns = int(slot.t_wall * 1e9)
            if slot.color is not None:
                self._enqueue(
                    f"/camera/{name}/color", ns,
                    encode_jpeg(slot.color, self.jpeg_quality),
                )
            if slot.depth is not None:
                self._enqueue(f"/camera/{name}/depth", ns,
                              slot.depth.astype("<u2").tobytes())
            self._enqueue(
                f"/camera/{name}/stamps", ns,
                json.dumps({
                    "seq": slot.seq,
                    "t_wall": slot.t_wall,
                    "t_mono": slot.t_mono,
                    "t_device_ms": slot.t_device,
                }).encode("utf-8"),
            )

    def _bytes_loop(self) -> None:
        while not self._halt.is_set():
            ep = self._episode_dir
            if ep is not None:
                b = dir_bytes(ep)
                now = time.monotonic()
                if self._bytes_prev is not None:
                    dt = now - self._bytes_prev[0]
                    if dt >= 1.0:
                        self._bytes_rate = (b - self._bytes_prev[1]) / dt
                        self._bytes_prev = (now, b)
                else:
                    self._bytes_prev = (now, b)
                self._bytes = b
            time.sleep(0.5)

    def _snapshot_params(self, ep_dir: Path) -> None:
        if self.params_provider is None:
            return
        try:
            params = self.params_provider()
            _atomic_write_json(ep_dir / "params" / "config.json", params)
        except Exception as exc:
            log.warning("params snapshot failed: %s", exc)

    # -- status ---------------------------------------------------------

    def status(self) -> dict:
        out = {
            "state": "recording" if self.recording else "idle",
            "storage": "mcap",
            "episode": self._episode_name,
            "last_episode": self._last_episode,
        }
        if self.recording:
            out["elapsed_s"] = round(time.monotonic() - self._start_mono, 1)
            out["bytes"] = self._bytes
            out["bytes_per_s"] = round(self._bytes_rate)
            out["write_queue_drops"] = self._drops_write_q
            # live store rates per channel (what is actually being written)
            out["channel_hz"] = {
                topic: round(ch.rate.hz(), 1)
                for topic, ch in self._channels.items()
                if ch.rate.hz() > 0.05
            }
        try:
            st = os.statvfs(self.demos_root if self.demos_root.exists() else "/")
            out["disk_free_gb"] = round(st.f_bavail * st.f_frsize / 1e9, 1)
        except OSError:
            pass
        return out
