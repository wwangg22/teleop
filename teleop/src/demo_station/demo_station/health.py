"""Topic-rate health monitoring + latest-image hub for the GUI.

TopicHealth measures message rates for arbitrary topics by resolving each
topic's type from the ROS graph on the fly (topics may not exist yet when
the GUI starts -- resolution retries until a publisher appears).

ImageHub keeps the latest frame per camera stream and JPEG-encodes ONLY
when a browser is actually watching, so idle cost is one memcpy per frame.
"""

from __future__ import annotations

import threading
import time
from collections import deque

import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


class _RateCounter:
    def __init__(self, window_s: float = 3.0) -> None:
        self._stamps: deque[float] = deque(maxlen=1000)
        self._window = window_s
        self._lock = threading.Lock()

    def tick(self) -> None:
        with self._lock:
            self._stamps.append(time.monotonic())

    def hz(self) -> float:
        now = time.monotonic()
        with self._lock:
            while self._stamps and self._stamps[0] < now - self._window:
                self._stamps.popleft()
            n = len(self._stamps)
        return round(n / self._window, 1)


class TopicHealth:
    def __init__(self, node: Node) -> None:
        self._node = node
        self._counters: dict[str, _RateCounter] = {}
        self._unresolved: set[str] = set()
        self._lock = threading.Lock()
        node.create_timer(2.0, self._resolve_pending)

    def watch(self, topic: str) -> None:
        with self._lock:
            if topic in self._counters or topic in self._unresolved:
                return
            self._unresolved.add(topic)

    def hz(self, topic: str) -> float | None:
        counter = self._counters.get(topic)
        return counter.hz() if counter else None

    def _resolve_pending(self) -> None:
        with self._lock:
            pending = list(self._unresolved)
        if not pending:
            return
        graph = dict(self._node.get_topic_names_and_types())
        for topic in pending:
            types = graph.get(topic)
            if not types:
                continue
            try:
                cls = get_message(types[0])
            except Exception:
                continue
            counter = _RateCounter()
            # raw=True: we only count arrivals; skip deserialization cost
            self._node.create_subscription(
                cls, topic,
                lambda _data, c=counter: c.tick(),
                qos_profile_sensor_data,
                raw=True,
            )
            with self._lock:
                self._counters[topic] = counter
                self._unresolved.discard(topic)


class ImageHub:
    """Latest color/depth frame per camera, encoded to JPEG on demand.

    Subscribes RAW (serialized bytes): deserializing two full-res image
    streams in Python just to count them cost ~50% of a core and made the
    measured Hz lie (the subscriber dropped frames, not the camera).
    Frames are deserialized only when a browser actually requests one.
    """

    def __init__(self, node: Node, camera_names: list[str]) -> None:
        from sensor_msgs.msg import Image

        self._image_type = Image
        self._latest: dict[str, bytes] = {}
        self._counters: dict[str, _RateCounter] = {}
        self._lock = threading.Lock()

        for name in camera_names:
            for kind, topic in (
                ("color", f"/camera/{name}/color/image_raw"),
                ("depth", f"/camera/{name}/depth/image_rect_raw"),
            ):
                key = f"{name}/{kind}"
                self._counters[key] = _RateCounter()
                node.create_subscription(
                    Image, topic,
                    lambda data, k=key: self._store(k, data),
                    qos_profile_sensor_data,
                    raw=True,
                )

    def _store(self, key: str, data: bytes) -> None:
        with self._lock:
            self._latest[key] = data
        self._counters[key].tick()

    def rates(self) -> dict[str, float]:
        return {k: c.hz() for k, c in self._counters.items()}

    def jpeg(self, key: str, quality: int = 80) -> bytes | None:
        if cv2 is None:
            return None
        with self._lock:
            data = self._latest.get(key)
        if data is None:
            return None
        from rclpy.serialization import deserialize_message
        try:
            msg = deserialize_message(data, self._image_type)
        except Exception:
            return None
        img = decode_image(msg)
        if img is None:
            return None
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes() if ok else None


def decode_image(msg) -> "np.ndarray | None":
    """sensor_msgs/Image -> BGR uint8 ndarray (depth gets colorized)."""
    if cv2 is None:
        return None
    h, w = msg.height, msg.width
    enc = msg.encoding
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc == "rgb8":
        return cv2.cvtColor(data.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    if enc == "bgr8":
        return data.reshape(h, w, 3)
    if enc in ("16UC1", "mono16"):
        depth = data.view(np.uint16).reshape(h, w)
        # 0.07-1.5 m mapped over the colormap; invalid (0) rendered black
        clipped = np.clip(depth, 0, 1500).astype(np.float32) / 1500.0
        img = cv2.applyColorMap((clipped * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
        img[depth == 0] = 0
        return img
    if enc == "mono8":
        return cv2.cvtColor(data.reshape(h, w), cv2.COLOR_GRAY2BGR)
    return None
