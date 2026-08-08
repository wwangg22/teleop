"""Read-only helpers over the demos tree. NOTHING here writes or deletes."""

from __future__ import annotations

import json
from pathlib import Path

import yaml


def _read_json(path: Path) -> dict:
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return {}


def _read_bag_metadata(bag_dir: Path) -> dict:
    """rosbag2's metadata.yaml -- only exists after a clean close."""
    try:
        with open(bag_dir / "metadata.yaml") as f:
            return yaml.safe_load(f)["rosbag2_bagfile_information"]
    except Exception:
        return {}


def list_episodes(root: Path) -> list[dict]:
    episodes = []
    if not root.exists():
        return episodes
    for day in sorted(root.iterdir(), reverse=True):
        if not day.is_dir():
            continue
        for ep in sorted(day.iterdir(), reverse=True):
            if not ep.is_dir():
                continue
            info = _read_json(ep / "info.json")
            meta = _read_bag_metadata(ep / "bag")
            counts = {
                t["topic_metadata"]["name"]: t["message_count"]
                for t in meta.get("topics_with_message_count", [])
            }
            image_topics = sorted(
                t["topic_metadata"]["name"]
                for t in meta.get("topics_with_message_count", [])
                if t["topic_metadata"]["type"] == "sensor_msgs/msg/Image"
                and t["message_count"] > 0
            )
            size = sum(
                f.stat().st_size for f in ep.rglob("*") if f.is_file()
            )
            duration_s = info.get("duration_s")
            if duration_s is None and meta:
                duration_s = round(meta.get("duration", {}).get(
                    "nanoseconds", 0) / 1e9, 1)
            episodes.append({
                "id": f"{day.name}/{ep.name}",
                "date": day.name,
                "name": ep.name,
                "started_at": info.get("started_at"),
                "duration_s": duration_s,
                "stop_reason": info.get("stop_reason"),
                "clean_close": bool(meta),
                "bytes": size,
                "messages": sum(counts.values()) if counts else None,
                "image_topics": image_topics,
                "counts": counts,
            })
    return episodes


def open_reader(bag_dir: Path):
    import rosbag2_py

    meta = _read_bag_metadata(bag_dir)
    storage_id = meta.get("storage_identifier", "sqlite3")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def iter_topic(bag_dir: Path, topic: str):
    """Yield (t_ns, deserialized_msg) for one topic."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = open_reader(bag_dir)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_map:
        return
    cls = get_message(type_map[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    while reader.has_next():
        name, raw, t_ns = reader.read_next()
        yield t_ns, deserialize_message(raw, cls)


def joint_trace(bag_dir: Path, topic: str = "/rebotarm/joint_states",
                max_points: int = 1500) -> dict:
    """Downsampled joint positions for plotting: {names, t: [...], q: [[...]]}"""
    stamps, rows, names = [], [], []
    for t_ns, msg in iter_topic(bag_dir, topic):
        if not names:
            names = list(msg.name)
        stamps.append(t_ns)
        rows.append(list(msg.position))
    if not stamps:
        return {"names": [], "t": [], "q": []}
    step = max(1, len(stamps) // max_points)
    t0 = stamps[0]
    return {
        "names": names,
        "t": [round((s - t0) / 1e9, 3) for s in stamps[::step]],
        "q": [[round(v, 4) for v in row] for row in rows[::step]],
    }
