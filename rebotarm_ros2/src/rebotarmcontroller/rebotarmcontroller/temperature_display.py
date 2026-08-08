"""Live motor temperature window for the reBot arm.

Subscribes to /<ns>/joints/<joint>/state, /<ns>/gripper/state and
/<ns>/thermal_status and shows a colour-coded dashboard in degrees Fahrenheit.

The ROS messages stay in Celsius (REP-103 / SI); conversion happens here so
other tooling is unaffected.

    ros2 run rebotarmcontroller TemperatureDisplay
    ros2 run rebotarmcontroller TemperatureDisplay --ros-args \
        -p warn_f:=140.0 -p critical_f:=167.0
"""

from __future__ import annotations

import math
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from rebotarm_msgs.msg import JointMotorState

_BG = "#14171c"
_FG = "#e6e6e6"
_DIM = "#8a94a6"
_OK = "#3ddc84"
_WARN = "#ffb020"
_CRIT = "#ff4d4d"
_STALE = "#5a6070"


def c_to_f(celsius: float) -> float:
    return celsius * 9.0 / 5.0 + 32.0


class TemperatureDisplay(Node):
    def __init__(self) -> None:
        super().__init__("temperature_display")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("joints", ["joint1", "joint2", "joint3",
                                          "joint4", "joint5", "joint6"])
        # Defaults mirror the driver's 60C / 75C in Fahrenheit.
        self.declare_parameter("warn_f", 140.0)
        self.declare_parameter("critical_f", 167.0)

        ns = str(self.get_parameter("arm_namespace").value).strip("/")
        self.joints = list(self.get_parameter("joints").value)
        self.warn_f = float(self.get_parameter("warn_f").value)
        self.critical_f = float(self.get_parameter("critical_f").value)

        # name -> (fahrenheit or None, monotonic stamp)
        self.readings: dict[str, tuple[float | None, float]] = {}
        self.status_text = "waiting for driver..."
        self._lock = threading.Lock()

        for joint in self.joints:
            self.create_subscription(
                JointMotorState,
                f"/{ns}/joints/{joint}/state",
                self._make_cb(joint),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            JointMotorState,
            f"/{ns}/gripper/state",
            self._make_cb("gripper"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String, f"/{ns}/thermal_status", self._status_cb, 10
        )

    def _make_cb(self, name: str):
        def cb(msg: JointMotorState) -> None:
            celsius = float(msg.temperature_mos)
            # 0.0 is the "never populated" signature (t_rotor does this), and
            # NaN means no frame yet -- show both as "--" rather than a number.
            value = None if (math.isnan(celsius) or celsius <= 1.0) else c_to_f(celsius)
            with self._lock:
                self.readings[name] = (value, self.now_s())
        return cb

    def _status_cb(self, msg: String) -> None:
        with self._lock:
            self.status_text = msg.data

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def snapshot(self) -> tuple[dict[str, tuple[float | None, float]], str, float]:
        with self._lock:
            return dict(self.readings), self.status_text, self.now_s()


class Window:
    def __init__(self, node: TemperatureDisplay) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title("reBot Arm - Motor Temperature")
        self.root.configure(bg=_BG)
        self.root.geometry("460x420")

        tk.Label(
            self.root, text="MOTOR TEMPERATURE", bg=_BG, fg=_DIM,
            font=("DejaVu Sans", 11, "bold"),
        ).pack(pady=(14, 2))
        tk.Label(
            self.root, text="MOSFET sensor  -  °F", bg=_BG, fg=_STALE,
            font=("DejaVu Sans", 9),
        ).pack(pady=(0, 10))

        self.rows: dict[str, tuple[tk.Label, tk.Label]] = {}
        grid = tk.Frame(self.root, bg=_BG)
        grid.pack(fill="x", padx=26)
        for name in list(node.joints) + ["gripper"]:
            row = tk.Frame(grid, bg=_BG)
            row.pack(fill="x", pady=3)
            label = tk.Label(row, text=name, bg=_BG, fg=_FG, width=10,
                             anchor="w", font=("DejaVu Sans Mono", 13))
            label.pack(side="left")
            value = tk.Label(row, text="--", bg=_BG, fg=_STALE, anchor="e",
                             font=("DejaVu Sans Mono", 19, "bold"))
            value.pack(side="right")
            self.rows[name] = (label, value)

        self.status = tk.Label(
            self.root, text="", bg=_BG, fg=_DIM, wraplength=420,
            justify="center", font=("DejaVu Sans", 10),
        )
        self.status.pack(side="bottom", pady=14)

        self.limits = tk.Label(
            self.root,
            text=f"warn ≥ {node.warn_f:.0f}°F     "
                 f"critical ≥ {node.critical_f:.0f}°F",
            bg=_BG, fg=_STALE, font=("DejaVu Sans", 9),
        )
        self.limits.pack(side="bottom")

        self.root.after(250, self.refresh)

    def refresh(self) -> None:
        readings, status, now = self.node.snapshot()
        for name, (_, value_label) in self.rows.items():
            entry = readings.get(name)
            if entry is None:
                value_label.config(text="--", fg=_STALE)
                continue
            fahrenheit, stamp = entry
            if fahrenheit is None:
                value_label.config(text="n/a", fg=_STALE)
            elif now - stamp > 3.0:
                # Driver stopped publishing; show the last value greyed out
                # rather than pretending it is current.
                value_label.config(text=f"{fahrenheit:5.1f} (stale)", fg=_STALE)
            else:
                if fahrenheit >= self.node.critical_f:
                    colour = _CRIT
                elif fahrenheit >= self.node.warn_f:
                    colour = _WARN
                else:
                    colour = _OK
                value_label.config(text=f"{fahrenheit:5.1f}", fg=colour)

        colour = _DIM
        lowered = status.lower()
        if "protective_stop" in lowered or "critical" in lowered:
            colour = _CRIT
        elif lowered.startswith("warn"):
            colour = _WARN
        elif "no_data" in lowered:
            colour = _STALE
        self.status.config(text=status, fg=colour)

        self.root.after(250, self.refresh)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TemperatureDisplay()

    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True
    )
    spin_thread.start()

    window = Window(node)
    try:
        window.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
