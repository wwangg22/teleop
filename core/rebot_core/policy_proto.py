"""Inference-server wire protocol (contract C2 of the policy-rollout infra).

Tiny, versioned, stdlib-only framing over a unix stream socket (msgpack is
not in the teleop env, and adding a dependency for ~3.3 Hz of 45 KB frames
buys nothing):

    frame := magic(4) | header_len(u32 LE) | header(JSON utf-8) | payload
    magic  = b"RBP1"
    header = {"v": 1, "kind": "...", "info": {...},
              "arrays": [{"name", "dtype", "shape", "nbytes"}, ...]}
    payload = the arrays' raw bytes, concatenated in header order

One request -> one reply on a persistent connection. The INFERENCE PROCESS
is the server (it holds the model); the bridge is the client. Keeping torch
in a separate process is load-bearing, not taste: the 500 Hz MIT loop lives
on this process's GIL, and the sentinel's gil_starvation check exists
because in-process heavy compute has starved it before (violent-chatter
failure mode -- see hardware.py's _CMD_LOCK_WAIT comment).

Request kinds:
    "infer": obs arrays in, one "chunk" (N, action_dim) f32 array out.
    "ping":  liveness/handshake; reply info echoes the server description.
"""

from __future__ import annotations

import json
import logging
import os
import socket
import struct
import time

import numpy as np

log = logging.getLogger("policy.proto")

MAGIC = b"RBP1"
PROTO_VERSION = 1
_MAX_HEADER = 1 << 20          # sanity bound; a header is ~1 KB in practice
_MAX_PAYLOAD = 256 << 20


def pack_frame(kind: str, arrays: dict[str, np.ndarray] | None = None,
               info: dict | None = None) -> bytes:
    metas, blobs = [], []
    for name, arr in (arrays or {}).items():
        arr = np.ascontiguousarray(arr)
        raw = arr.tobytes()
        metas.append({"name": name, "dtype": arr.dtype.str,
                      "shape": list(arr.shape), "nbytes": len(raw)})
        blobs.append(raw)
    header = json.dumps(
        {"v": PROTO_VERSION, "kind": kind, "info": info or {}, "arrays": metas}
    ).encode("utf-8")
    return b"".join([MAGIC, struct.pack("<I", len(header)), header, *blobs])


def _read_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        got = sock.recv(n - len(buf))
        if not got:
            raise ConnectionError("peer closed mid-frame")
        buf.extend(got)
    return bytes(buf)


def read_frame(sock: socket.socket) -> tuple[str, dict[str, np.ndarray], dict]:
    magic = _read_exact(sock, 4)
    if magic != MAGIC:
        raise ConnectionError(f"bad magic {magic!r}")
    (hlen,) = struct.unpack("<I", _read_exact(sock, 4))
    if hlen > _MAX_HEADER:
        raise ConnectionError(f"header too large ({hlen})")
    header = json.loads(_read_exact(sock, hlen).decode("utf-8"))
    if int(header.get("v", -1)) != PROTO_VERSION:
        raise ConnectionError(f"protocol version {header.get('v')} != "
                              f"{PROTO_VERSION}")
    total = sum(int(m["nbytes"]) for m in header.get("arrays", []))
    if total > _MAX_PAYLOAD:
        raise ConnectionError(f"payload too large ({total})")
    payload = _read_exact(sock, total) if total else b""
    arrays: dict[str, np.ndarray] = {}
    off = 0
    for meta in header.get("arrays", []):
        n = int(meta["nbytes"])
        arr = np.frombuffer(payload[off:off + n], dtype=np.dtype(meta["dtype"]))
        arrays[meta["name"]] = arr.reshape(meta["shape"]).copy()
        off += n
    return str(header["kind"]), arrays, dict(header.get("info", {}))


class PolicyClient:
    """Bridge-side endpoint. Blocking request/reply; the caller decides the
    thread it runs on (the bridge uses a dedicated inference thread so the
    50 Hz control loop never blocks on the socket)."""

    def __init__(self, socket_path: str, timeout_s: float = 5.0) -> None:
        self.socket_path = socket_path
        self.timeout_s = timeout_s
        self._sock: socket.socket | None = None

    def connect(self, retry_for_s: float = 10.0) -> dict:
        deadline = time.monotonic() + retry_for_s
        last: Exception | None = None
        while time.monotonic() < deadline:
            try:
                s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                s.settimeout(self.timeout_s)
                s.connect(self.socket_path)
                self._sock = s
                return self.request("ping")[1]
            except (OSError, ConnectionError) as exc:
                last = exc
                time.sleep(0.25)
        raise ConnectionError(
            f"no inference server at {self.socket_path} ({last})"
        )

    def request(self, kind: str, arrays: dict[str, np.ndarray] | None = None,
                info: dict | None = None) -> tuple[dict[str, np.ndarray], dict]:
        if self._sock is None:
            raise ConnectionError("not connected")
        self._sock.sendall(pack_frame(kind, arrays, info))
        rkind, rarrays, rinfo = read_frame(self._sock)
        if rkind == "error":
            raise RuntimeError(f"inference server error: {rinfo.get('message')}")
        return rarrays, rinfo

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None


class PolicyServer:
    """Inference-side loop: bind a unix socket, serve one client at a time,
    sequentially. Subclass and implement infer(); ping is handled here.

    Deliberately synchronous -- the bridge is the only client, requests come
    at the replan rate (a few Hz), and simplicity beats throughput here.
    """

    description = "base"

    def __init__(self, socket_path: str) -> None:
        self.socket_path = socket_path
        self.requests = 0

    # override -----------------------------------------------------------
    def infer(self, arrays: dict[str, np.ndarray], info: dict
              ) -> tuple[dict[str, np.ndarray], dict]:
        raise NotImplementedError

    # ---------------------------------------------------------------------
    def serve_forever(self) -> None:
        try:
            os.unlink(self.socket_path)
        except FileNotFoundError:
            pass
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(self.socket_path)
        srv.listen(1)
        log.info("policy server (%s) on %s", self.description, self.socket_path)
        print(f"POLICY_SERVER_READY {self.socket_path}", flush=True)
        while True:
            conn, _ = srv.accept()
            log.info("client connected")
            try:
                while True:
                    kind, arrays, info = read_frame(conn)
                    t0 = time.monotonic()
                    if kind == "ping":
                        conn.sendall(pack_frame(
                            "pong", info={"description": self.description}))
                        continue
                    try:
                        out_arrays, out_info = self.infer(arrays, info)
                        self.requests += 1
                        out_info.setdefault(
                            "latency_ms", round((time.monotonic() - t0) * 1e3, 1))
                        conn.sendall(pack_frame("chunk", out_arrays, out_info))
                    except Exception as exc:   # reply, don't die: the arm-side
                        log.exception("infer failed")   # bridge decides what a
                        conn.sendall(pack_frame(        # failed replan means
                            "error", info={"message": str(exc)}))
            except (ConnectionError, OSError) as exc:
                log.info("client gone (%s)", exc)
            finally:
                conn.close()
