"""One-button 'return to zero' panel for the reBot arm.

Calls /<ns>/safe_home, which drives every joint to the zero pose. Shows live
joint deviation so you can see it converge, and disables the button while a
home is in flight so it cannot be double-fired.

    ros2 run rebotarmcontroller HomeButton
"""

from __future__ import annotations

import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

_BG = "#14171c"
_FG = "#e6e6e6"
_DIM = "#8a94a6"
_OK = "#3ddc84"
_BUSY = "#ffb020"
_ERR = "#ff4d4d"
_BTN = "#2563eb"
_BTN_ACTIVE = "#1d4ed8"
_BTN_OFF = "#39404d"

# Below this every joint counts as "at zero" (matches the arm's steady-state
# droop -- there is no integral term, so it settles slightly short).
_AT_ZERO_RAD = 0.05


class HomeButtonNode(Node):
    def __init__(self) -> None:
        super().__init__("home_button")
        self.declare_parameter("arm_namespace", "rebotarm")
        ns = str(self.get_parameter("arm_namespace").value).strip("/")

        self._lock = threading.Lock()
        self.positions: dict[str, float] = {}
        self.have_state = False

        self.create_subscription(
            JointState,
            f"/{ns}/joint_states",
            self._joint_cb,
            qos_profile_sensor_data,
        )
        self._home_client = self.create_client(Trigger, f"/{ns}/safe_home")

    def _joint_cb(self, msg: JointState) -> None:
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                # gripper_joint* are prismatic mimics, not arm joints
                if name.startswith("joint"):
                    self.positions[name] = float(position)
            self.have_state = True

    def snapshot(self) -> tuple[dict[str, float], bool]:
        with self._lock:
            return dict(self.positions), self.have_state

    def service_ready(self) -> bool:
        return self._home_client.service_is_ready()

    def call_home(self, done):
        """Fire safe_home asynchronously; done(ok, message) on completion."""
        if not self._home_client.service_is_ready():
            done(False, "safe_home service not available")
            return
        future = self._home_client.call_async(Trigger.Request())

        def _cb(fut):
            try:
                response = fut.result()
                done(bool(response.success), str(response.message))
            except Exception as exc:
                done(False, str(exc))

        future.add_done_callback(_cb)


class Window:
    def __init__(self, node: HomeButtonNode) -> None:
        self.node = node
        self.busy = False

        self.root = tk.Tk()
        self.root.title("reBot Arm - Home")
        self.root.configure(bg=_BG)
        self.root.geometry("380x300")

        tk.Label(
            self.root, text="RETURN TO ZERO", bg=_BG, fg=_DIM,
            font=("DejaVu Sans", 11, "bold"),
        ).pack(pady=(18, 2))
        tk.Label(
            self.root, text="the arm will move", bg=_BG, fg=_ERR,
            font=("DejaVu Sans", 9),
        ).pack(pady=(0, 14))

        self.button = tk.Button(
            self.root,
            text="GO TO ZERO",
            command=self.on_press,
            bg=_BTN, fg="white", activebackground=_BTN_ACTIVE,
            activeforeground="white", relief="flat",
            font=("DejaVu Sans", 17, "bold"),
            width=14, height=2, cursor="hand2",
            highlightthickness=0, bd=0,
        )
        self.button.pack(pady=4)

        self.status = tk.Label(
            self.root, text="", bg=_BG, fg=_DIM, wraplength=340,
            justify="center", font=("DejaVu Sans", 10),
        )
        self.status.pack(pady=(14, 4))

        self.deviation = tk.Label(
            self.root, text="", bg=_BG, fg=_DIM,
            font=("DejaVu Sans Mono", 11),
        )
        self.deviation.pack()

        self.root.after(200, self.refresh)

    def on_press(self) -> None:
        if self.busy:
            return
        self.busy = True
        self.button.config(state="disabled", bg=_BTN_OFF, text="HOMING...")
        self.status.config(text="safe_home running -- driving to zero", fg=_BUSY)

        def done(ok: bool, message: str) -> None:
            # Called from the rclpy thread; hop back onto the Tk thread.
            self.root.after(0, lambda: self.finish(ok, message))

        self.node.call_home(done)

    def finish(self, ok: bool, message: str) -> None:
        self.busy = False
        self.button.config(state="normal", bg=_BTN, text="GO TO ZERO")
        self.status.config(
            text=message or ("homed" if ok else "failed"),
            fg=_OK if ok else _ERR,
        )

    def refresh(self) -> None:
        positions, have_state = self.node.snapshot()

        if not have_state:
            self.deviation.config(text="waiting for joint_states...", fg=_DIM)
        elif positions:
            worst_name, worst = max(
                positions.items(), key=lambda kv: abs(kv[1])
            )
            at_zero = abs(worst) <= _AT_ZERO_RAD
            self.deviation.config(
                text=f"max offset  {worst_name} {worst:+.3f} rad"
                     + ("   AT ZERO" if at_zero else ""),
                fg=_OK if at_zero else _DIM,
            )

        if not self.busy:
            if not self.node.service_ready():
                self.button.config(state="disabled", bg=_BTN_OFF)
                self.status.config(text="driver not running", fg=_ERR)
            elif self.button["state"] == "disabled":
                self.button.config(state="normal", bg=_BTN)
                self.status.config(text="ready", fg=_DIM)

        self.root.after(200, self.refresh)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HomeButtonNode()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

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
