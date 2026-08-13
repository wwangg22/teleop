"""Thermal protective stop for policy rollouts.

Port of the ROS stack's thermal_guard (rebotarmcontroller/thermal_guard.py)
into rebot_core, because the core HealthSentinel is OBSERVE-ONLY and the
only stop today is the power switch -- while joint 3 has measured 129 F
just holding a pose. Started ONLY by the PolicyBridge, and only when the
manifest asks (safety.thermal_watch: true); ordinary teleop behavior is
untouched.

Kept behaviors (each was a deliberate ROS-side decision):
- FAIL PERMISSIVE on missing/implausible data: a reading <= 1 C is the
  RobStride "never populated" signature and >= 200 C is a decode error;
  neither may trip a moving arm. Only t_mos is used (t_rotor reads 0.0).
- Trip needs `consecutive_to_trip` consecutive over-critical polls.
- protective_stop = safe_home() THEN disable(); if homing fails the motors
  stay ENERGIZED and holding -- disabling from an unknown pose DROPS the
  arm, which is worse than heat.
- Latched: runs at most once; motion on a worker thread.

Data source: StateCache.temperatures() -- the cached ~2 s CAN poll that
already exists. This watcher adds ZERO bus traffic.

Thresholds default to the ROS config (driver_params.yaml): warn 60 C,
critical 75 C, 3 consecutive.
"""

from __future__ import annotations

import logging
import math
import threading
import time

log = logging.getLogger("policy.thermal")

_MIN_PLAUSIBLE_C = 1.0
_MAX_PLAUSIBLE_C = 200.0


def _valid(v: float) -> bool:
    return (v is not None and not math.isnan(v)
            and _MIN_PLAUSIBLE_C < v < _MAX_PLAUSIBLE_C)


class PolicyThermalWatcher:
    def __init__(
        self,
        state_cache,
        hardware,
        on_trip=None,                 # callable(reason) -- bridge stops policy
        warn_c: float = 60.0,
        critical_c: float = 75.0,
        poll_hz: float = 1.0,
        consecutive_to_trip: int = 3,
    ) -> None:
        self._states = state_cache
        self._hw = hardware
        self._on_trip = on_trip
        self._warn_c = float(warn_c)
        self._critical_c = float(critical_c)
        self._period = 1.0 / float(poll_hz)
        self._need = max(1, int(consecutive_to_trip))
        self._over = 0
        self.tripped = False
        self._seen_valid = False
        self._warned_no_data = False
        self._last_warn = 0.0
        self.worst_c = 0.0
        self._halt = threading.Event()
        self._thread: threading.Thread | None = None

    # -- lifecycle --------------------------------------------------------

    def start(self) -> None:
        self._halt.clear()
        self._thread = threading.Thread(
            target=self._loop, name="policy-thermal", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._halt.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    # -- loop -------------------------------------------------------------

    def _loop(self) -> None:
        while not self._halt.is_set():
            try:
                self._poll()
            except Exception as exc:
                log.warning("thermal poll failed: %s", exc)
            time.sleep(self._period)

    def _poll(self) -> None:
        temps = self._states.temperatures()
        if temps is None:
            self._no_data()
            return
        t_mos, g_mos = temps
        readings = [float(v) for v in t_mos] + [float(g_mos)]
        valid = [v for v in readings if _valid(v)]
        if not valid:
            self._no_data()
            return
        self._seen_valid = True
        hottest = max(valid)
        self.worst_c = max(self.worst_c, hottest)
        now = time.monotonic()
        if hottest >= self._critical_c:
            self._over += 1
            log.error("motor temp %.1f C >= critical %.1f C (%d/%d)",
                      hottest, self._critical_c, self._over, self._need)
            if self._over >= self._need:
                self.protective_stop(
                    f"motor temperature {hottest:.1f} C over critical "
                    f"{self._critical_c:.1f} C for {self._need} polls"
                )
        else:
            self._over = 0
            if hottest >= self._warn_c and now - self._last_warn > 30.0:
                self._last_warn = now
                log.warning("motor temp %.1f C over warn %.1f C",
                            hottest, self._warn_c)

    def _no_data(self) -> None:
        if not self._seen_valid and not self._warned_no_data:
            self._warned_no_data = True
            log.warning(
                "no valid motor temperatures yet -- thermal guard is "
                "PERMISSIVE by design (a spurious trip on a moving arm is "
                "itself a hazard)"
            )

    # -- the stop ----------------------------------------------------------

    def protective_stop(self, reason: str) -> None:
        with threading.Lock():
            if self.tripped:
                return
            self.tripped = True
        log.error("PROTECTIVE STOP: %s -- safe_home then disable", reason)
        if self._on_trip is not None:
            try:
                self._on_trip(reason)     # policy stops streaming FIRST
            except Exception as exc:
                log.error("on_trip hook failed: %s", exc)
        threading.Thread(
            target=self._run_stop, name="thermal-stop", daemon=True
        ).start()

    def _run_stop(self) -> None:
        homed = False
        try:
            self._hw.safe_home()
            homed = True
            log.info("protective stop: reached home pose")
        except Exception as exc:
            log.error(
                "protective stop: safe_home FAILED (%s). Leaving motors "
                "ENERGIZED and holding rather than dropping the arm. "
                "Support the arm before cutting power.", exc
            )
        if not homed:
            return
        try:
            self._hw.disable()
            log.info("protective stop: motors disabled at home")
        except Exception as exc:
            log.error("protective stop: disable failed (%s)", exc)

    def snapshot(self) -> dict:
        return {
            "tripped": self.tripped,
            "worst_c": round(self.worst_c, 1),
            "seen_valid": self._seen_valid,
        }
