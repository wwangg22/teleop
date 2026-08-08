"""Thermal monitoring and protective stop for the reBot arm.

Two responsibilities:

1. Watch motor temperatures, warn when they get high, and trigger a protective
   stop when they get dangerous.
2. Provide protective_stop() -- the single primitive for "bring the arm down
   safely", used both here and by the node's shutdown path. It ALWAYS runs
   safe_home() (which drives to the zero pose) BEFORE disabling, so the arm is
   never left energized in an arbitrary pose or dropped from one.

SAFETY NOTE on missing data: RobStride firmware may not populate temperature in
the cached MotorState at all (see JointGroup.get_temperatures). This guard is
built to FAIL SAFE IN THE PERMISSIVE DIRECTION -- it will never trigger a stop
from missing, NaN, or implausible readings, because a spurious trip on a moving
arm is itself a hazard. If no valid readings are seen, it says so loudly once
and then stays out of the way.
"""

from __future__ import annotations

import math
import threading

from std_msgs.msg import String

# A motor reporting exactly 0.0 forever is the documented "never populated"
# signature, not a real measurement. Anything at or below this is ignored.
_MIN_PLAUSIBLE_C = 1.0
# Nothing above this is believable from these sensors; treat as a bad decode
# rather than an emergency, so a garbled frame cannot trip the arm.
_MAX_PLAUSIBLE_C = 200.0


def _valid(value: float) -> bool:
    return (
        value is not None
        and not math.isnan(value)
        and _MIN_PLAUSIBLE_C < value < _MAX_PLAUSIBLE_C
    )


def c_to_f(celsius: float) -> float:
    return celsius * 9.0 / 5.0 + 32.0


def _fmt(celsius: float) -> str:
    """Thresholds and messages stay in Celsius internally (SI/REP-103);
    both units are shown so logs are readable either way."""
    return f"{c_to_f(celsius):.1f}F ({celsius:.1f}C)"


class ThermalGuard:
    def __init__(
        self,
        node,
        hardware,
        namespace: str,
        *,
        warn_c: float,
        critical_c: float,
        poll_hz: float,
        consecutive_to_trip: int,
    ) -> None:
        self._node = node
        self._hardware = hardware
        self._warn_c = float(warn_c)
        self._critical_c = float(critical_c)
        self._consecutive_to_trip = max(1, int(consecutive_to_trip))

        self._over_critical_count = 0
        self._tripped = False
        self._stop_lock = threading.Lock()
        self._stop_thread: threading.Thread | None = None
        self._seen_valid_reading = False
        self._warned_no_data = False
        self._last_warn_log = 0.0

        self._status_publisher = node.create_publisher(
            String,
            f"/{namespace}/thermal_status",
            10,
            callback_group=node.reentrant_group,
        )

        period = 1.0 / max(float(poll_hz), 0.1)
        # slow_group: the trip path can block for seconds; keep it off the
        # callback group serving joint state publishing.
        self._timer = node.create_timer(
            period,
            self._poll,
            callback_group=node.slow_group,
        )
        node.get_logger().info(
            f"thermal guard active: warn={_fmt(self._warn_c)} "
            f"critical={_fmt(self._critical_c)} "
            f"(trip after {self._consecutive_to_trip} consecutive readings)"
        )

    # ------------------------------------------------------------------
    # monitoring
    # ------------------------------------------------------------------

    def _collect(self) -> list[tuple[str, float]]:
        """[(joint_name, temperature)] for readings that look real."""
        readings: list[tuple[str, float]] = []
        try:
            t_mos, t_rotor = self._hardware.get_joint_temperatures(request=False)
        except Exception as exc:
            self._node.get_logger().warn(f"temperature read failed: {exc}")
            return readings

        for i, name in enumerate(self._hardware.joint_names):
            for label, series in (("mos", t_mos), ("rotor", t_rotor)):
                if i >= len(series):
                    continue
                value = float(series[i])
                if _valid(value):
                    readings.append((f"{name}.{label}", value))

        if self._hardware.has_gripper:
            g_mos, g_rotor = self._hardware.get_gripper_temperatures(request=False)
            for label, value in (("mos", g_mos), ("rotor", g_rotor)):
                if _valid(value):
                    readings.append((f"gripper.{label}", float(value)))
        return readings

    def _poll(self) -> None:
        readings = self._collect()

        if not readings:
            if not self._warned_no_data:
                self._warned_no_data = True
                self._node.get_logger().warn(
                    "thermal guard: no valid temperature readings from any motor. "
                    "The arm is NOT thermally protected. This is expected if "
                    "RobStride firmware does not populate MotorState temperature."
                )
                self._publish("no_data")
            # Never trip on absent data.
            self._over_critical_count = 0
            return

        if not self._seen_valid_reading:
            self._seen_valid_reading = True
            self._node.get_logger().info(
                f"thermal guard: receiving temperature from {len(readings)} sensors"
            )

        hottest_name, hottest = max(readings, key=lambda pair: pair[1])

        if hottest >= self._critical_c:
            self._over_critical_count += 1
            self._node.get_logger().error(
                f"CRITICAL temperature {_fmt(hottest)} on {hottest_name} "
                f"(limit {_fmt(self._critical_c)}) "
                f"[{self._over_critical_count}/{self._consecutive_to_trip}]"
            )
            self._publish(f"critical {hottest_name}={c_to_f(hottest):.1f}F")
            if self._over_critical_count >= self._consecutive_to_trip:
                self.protective_stop(
                    f"{hottest_name} reached {_fmt(hottest)} "
                    f"(critical limit {_fmt(self._critical_c)})"
                )
            return

        self._over_critical_count = 0

        if hottest >= self._warn_c:
            now = self._node.get_clock().now().nanoseconds * 1e-9
            if now - self._last_warn_log > 10.0:
                self._last_warn_log = now
                self._node.get_logger().warn(
                    f"high temperature {_fmt(hottest)} on {hottest_name} "
                    f"(warn {_fmt(self._warn_c)}, critical {_fmt(self._critical_c)})"
                )
            self._publish(f"warn {hottest_name}={c_to_f(hottest):.1f}F")
        else:
            self._publish(f"ok max={hottest_name}={c_to_f(hottest):.1f}F")

    def _publish(self, text: str) -> None:
        try:
            msg = String()
            msg.data = text
            self._status_publisher.publish(msg)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # protective stop -- ALWAYS returns to zero before disabling
    # ------------------------------------------------------------------

    def protective_stop(self, reason: str) -> None:
        """Drive to the zero pose via safe_home(), then disable.

        Latched: runs at most once. Idempotent and non-blocking -- the actual
        motion happens on a worker thread so a timer callback is never held for
        the duration of the homing move.
        """
        with self._stop_lock:
            if self._tripped:
                return
            self._tripped = True

        self._node.get_logger().error(
            f"PROTECTIVE STOP: {reason}. Returning to zero pose before disabling."
        )
        self._publish(f"protective_stop {reason}")

        self._stop_thread = threading.Thread(
            target=self._run_protective_stop,
            args=(reason,),
            daemon=True,
        )
        self._stop_thread.start()

    def _run_protective_stop(self, reason: str) -> None:
        homed = False
        try:
            # safe_home() targets home_pos = zeros -- this is the "always go
            # back to zero first" guarantee.
            self._hardware.safe_home()
            homed = True
            self._node.get_logger().info("protective stop: reached zero pose")
        except Exception as exc:
            self._node.get_logger().error(
                f"protective stop: safe_home FAILED ({exc}). "
                "Leaving motors ENERGIZED and holding rather than dropping the arm. "
                "Support the arm before cutting power."
            )

        if not homed:
            # Deliberately do NOT disable. Disabling from an unknown pose drops
            # the arm; holding is the safer failure mode for a thermal event.
            self._publish("protective_stop failed_to_home holding")
            return

        try:
            self._hardware.disable()
            self._node.get_logger().info("protective stop: motors disabled at zero")
            self._publish(f"protective_stop complete {reason}")
        except Exception as exc:
            self._node.get_logger().error(f"protective stop: disable failed ({exc})")

    @property
    def tripped(self) -> bool:
        return self._tripped
