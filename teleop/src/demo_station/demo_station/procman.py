"""Process manager for the demo station GUI.

Starts each unit from units.yaml under bash with the ROS environment
sourced, in its own process group. Stop = SIGINT to the group (so ROS
nodes run their shutdown paths -- the driver safe-homes, rosbag finalizes),
escalating to SIGTERM only after a grace period. Never SIGKILL.
"""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

SOURCING = (
    "source /opt/ros/humble/setup.bash && "
    "source ~/rebotarm_ros2/install/setup.bash 2>/dev/null; "
    "source ~/Desktop/teleop/install/setup.bash 2>/dev/null; "
)

LOG_DIR = Path.home() / ".demo_station" / "logs"


@dataclass
class Unit:
    name: str
    label: str
    cmd: str = ""
    type: str = "process"          # "process" | "check"
    confirm: bool = False
    stdin: bool = False
    singleton: str = ""
    note: str = ""
    health: list = field(default_factory=list)

    proc: subprocess.Popen | None = None
    started_at: float | None = None
    last_exit: int | None = None
    stopping: bool = False


class ProcessManager:
    def __init__(self, units_cfg: list[dict]) -> None:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.units: dict[str, Unit] = {}
        for u in units_cfg:
            unit = Unit(
                name=u["name"],
                label=u.get("label", u["name"]),
                cmd=u.get("cmd", ""),
                type=u.get("type", "process"),
                confirm=bool(u.get("confirm", False)),
                stdin=bool(u.get("stdin", False)),
                singleton=u.get("singleton", ""),
                note=u.get("note", ""),
                health=list(u.get("health", []) or []),
            )
            self.units[unit.name] = unit

    # ------------------------------------------------------------------

    def _external_pid(self, unit: Unit) -> int | None:
        """PID of an instance of this unit we did not start (e.g. it survived
        a GUI restart -- units run in their own sessions and are deliberately
        NOT killed when the GUI exits)."""
        if not unit.cmd:
            return None
        out = subprocess.run(
            ["pgrep", "-f", unit.cmd], capture_output=True, text=True
        ).stdout.split()
        own = {str(unit.proc.pid)} if unit.proc is not None else set()
        own.add(str(os.getpid()))
        for pid in out:
            if pid not in own:
                return int(pid)
        return None

    def start(self, name: str) -> tuple[bool, str]:
        unit = self.units[name]
        if unit.type == "check":
            return False, "check units are run with run_check()"
        if unit.proc is not None and unit.proc.poll() is None:
            return False, "already running"
        ext = self._external_pid(unit)
        if ext is not None:
            return False, (f"already running outside this GUI (pid {ext}) -- "
                           "stop it first")
        if unit.singleton:
            out = subprocess.run(
                ["pgrep", "-af", unit.singleton], capture_output=True, text=True
            ).stdout.strip()
            if out:
                return False, f"refusing to start -- already running elsewhere:\n{out}"

        log = open(LOG_DIR / f"{name}.log", "ab", buffering=0)
        log.write(f"\n===== start {time.strftime('%F %T')} =====\n".encode())
        unit.proc = subprocess.Popen(
            ["bash", "-c", SOURCING + "exec " + unit.cmd],
            stdin=subprocess.PIPE if unit.stdin else subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        unit.started_at = time.time()
        unit.last_exit = None
        unit.stopping = False
        return True, f"started pid {unit.proc.pid}"

    def stop(self, name: str, grace: float = 10.0) -> tuple[bool, str]:
        unit = self.units[name]
        proc = unit.proc
        if proc is None or proc.poll() is not None:
            # adopted/external instance: stop it via its process group
            ext = self._external_pid(unit)
            if ext is None:
                return False, "not running"
            try:
                pgid = int(subprocess.run(
                    ["ps", "-o", "pgid=", "-p", str(ext)],
                    capture_output=True, text=True).stdout.strip())
                os.killpg(pgid, signal.SIGINT)
            except (ValueError, ProcessLookupError, PermissionError) as exc:
                return False, f"could not stop external pid {ext}: {exc}"
            deadline = time.time() + grace
            while time.time() < deadline:
                if self._external_pid(unit) is None:
                    return True, "stopped (external instance)"
                time.sleep(0.2)
            try:
                os.killpg(pgid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            return True, "stopped external instance (needed SIGTERM)"
        unit.stopping = True
        try:
            # SIGINT the PARENT ONLY -- never the group. `ros2 launch`
            # forwards SIGINT to its children itself; signaling the group
            # delivers a SECOND SIGINT to the node, which aborts its
            # shutdown path mid-way (for the arm driver that means
            # safe_home dies and the motors stay latched at torque).
            proc.send_signal(signal.SIGINT)
        except ProcessLookupError:
            return True, "already gone"
        deadline = time.time() + grace
        while time.time() < deadline:
            if proc.poll() is not None:
                return True, "stopped"
            time.sleep(0.2)
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        return True, "stopped (needed SIGTERM)"

    def send_stdin(self, name: str, text: str) -> tuple[bool, str]:
        unit = self.units[name]
        proc = unit.proc
        if proc is None or proc.poll() is not None or proc.stdin is None:
            return False, "not running or no stdin"
        proc.stdin.write((text + "\n").encode())
        proc.stdin.flush()
        return True, f"sent {text!r}"

    def run_check(self, name: str) -> dict:
        unit = self.units[name]
        try:
            out = subprocess.run(
                ["bash", "-c", unit.cmd], capture_output=True, text=True, timeout=10
            )
            text = (out.stdout + out.stderr).strip()
        except subprocess.TimeoutExpired:
            text = "(check timed out)"
        return {"output": text}

    def status(self, name: str) -> dict:
        unit = self.units[name]
        running = unit.proc is not None and unit.proc.poll() is None
        if unit.proc is not None and unit.proc.poll() is not None and unit.last_exit is None:
            unit.last_exit = unit.proc.returncode
        external = None
        if not running and unit.type == "process":
            external = self._external_pid(unit)
        return {
            "name": unit.name,
            "label": unit.label,
            "type": unit.type,
            "note": unit.note,
            "confirm": unit.confirm,
            "stdin": unit.stdin,
            "running": running or external is not None,
            "external": external is not None,
            "pid": unit.proc.pid if running else external,
            "uptime_s": round(time.time() - unit.started_at, 0)
            if running and unit.started_at else None,
            "last_exit": unit.last_exit if not external else None,
        }

    def log_tail(self, name: str, lines: int = 120) -> str:
        path = LOG_DIR / f"{name}.log"
        if not path.exists():
            return ""
        if shutil.which("tail"):
            return subprocess.run(
                ["tail", "-n", str(lines), str(path)],
                capture_output=True, text=True).stdout
        return path.read_text(errors="replace")[-20000:]

    def shutdown(self) -> None:
        """Deliberately does NOT stop managed units. They run in their own
        sessions and must survive a GUI restart -- killing the arm driver
        because the GUI was restarted is never acceptable. Adopted again
        (via _external_pid) when the GUI comes back."""
