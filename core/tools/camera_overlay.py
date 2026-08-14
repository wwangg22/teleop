"""Live camera-vs-sim overlay for aligning the real cameras to the sim rig.

The G3 gate (docs/POLICY_ROLLOUT.md, irl_rollout_plan.md): the station
camera's real pose is THE known gap between sim training frames and real
rollouts — the policy was trained on a fixed sim viewpoint and bench
session 1 showed systematic misses consistent with a viewpoint offset.
This tool blends the live RealSense color stream over a sim reference
frame so the mount can be iterated until table edge, box and robot base
coincide, and the wrist FOV can be sanity-checked the same way.

Both images go through rebot_core.policy_map.preprocess_frame — the EXACT
pixel path the policy bridge uses (center_16x9 crop for the D405, PIL BOX
downscale) — so what looks aligned here is aligned in policy pixels, not
just to the eye. Display upscales to a viewing canvas afterwards.

Display is tkinter + PIL (cv2 is deliberately absent from the teleop env).
Keys:
  a / d    alpha down / up (blend mode)
  m        cycle mode: blend -> edges (sim edges in green over live) -> flicker
  s        save the current composite next to --sim as overlay_<n>.png
  q / Esc  quit

The cameras must be FREE: rebot_core.runtime (and the GUI launcher that
spawns it) holds every /dev/video* node exclusively — stop it first or
pipe.start fails with "Device or resource busy" (errno=16).

Usage (conda env teleop, arm power NOT required — cameras only):
  python core/tools/camera_overlay.py \
      --sim /home/asuka/Desktop/IsaacLab/sim2real/data/policy_ws/raw/ep_s21_0000/anchors/0000_station_rgb.png
  python core/tools/camera_overlay.py --camera wrist --sim .../0000_wrist_rgb.png
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter, ImageTk

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "core"))

from rebot_core.policy_map import preprocess_frame  # noqa: E402

# Serials/profiles mirror teleop/src/demo_station/config/station.yaml — the
# tool is standalone on purpose (no station bring-up, no recorder, no arm).
CAMERAS = {
    "workspace": {"serial": "147122074827", "profile": (1280, 720, 30), "crop": "none"},
    "wrist": {"serial": "260522275150", "profile": (640, 480, 30), "crop": "center_16x9"},
}
MODES = ("blend", "edges", "flicker")


def start_color_stream(serial: str, profile: tuple[int, int, int]):
    import pyrealsense2 as rs

    cfg = rs.config()
    cfg.enable_device(serial)
    w, h, fps = profile
    cfg.enable_stream(rs.stream.color, w, h, rs.format.rgb8, fps)
    pipe = rs.pipeline()
    pipe.start(cfg)
    return pipe


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sim", required=True, help="sim reference frame (anchor *_rgb.png)")
    ap.add_argument("--camera", choices=sorted(CAMERAS), default="workspace")
    ap.add_argument("--alpha", type=float, default=0.5, help="initial live-frame weight")
    ap.add_argument("--canvas-width", type=int, default=960,
                    help="viewing width; compare resolution is fixed at 320x180")
    args = ap.parse_args()

    cam = CAMERAS[args.camera]
    # 320x180 = 2x policy resolution: same 16:9 pipeline, but enough pixels
    # to see edge misalignment that 160x90 would blur away.
    cmp_w, cmp_h = 320, 180
    view = (args.canvas_width, args.canvas_width * cmp_h // cmp_w)

    sim_rgb = np.asarray(Image.open(args.sim).convert("RGB"))
    sim_small = preprocess_frame(sim_rgb, "none", (cmp_w, cmp_h))
    sim_edges = np.asarray(
        Image.fromarray(sim_small).convert("L").filter(ImageFilter.FIND_EDGES)
    )
    edge_mask = sim_edges > 40

    pipe = start_color_stream(cam["serial"], cam["profile"])

    import tkinter as tk

    root = tk.Tk()
    root.title(f"overlay: {args.camera} vs {Path(args.sim).name}")
    label = tk.Label(root)
    label.pack()
    status = tk.StringVar()
    tk.Label(root, textvariable=status, font=("monospace", 10)).pack()

    state = {"alpha": args.alpha, "mode": 0, "saves": 0, "quit": False,
             "composite": sim_small}

    def on_key(ev):
        k = ev.keysym.lower()
        if k == "a":
            state["alpha"] = max(0.0, state["alpha"] - 0.05)
        elif k == "d":
            state["alpha"] = min(1.0, state["alpha"] + 0.05)
        elif k == "m":
            state["mode"] = (state["mode"] + 1) % len(MODES)
        elif k == "s":
            state["saves"] += 1
            out = Path(args.sim).with_name(f"overlay_{state['saves']:02d}.png")
            Image.fromarray(state["composite"]).save(out)
            print(f"saved {out}")
        elif k in ("q", "escape"):
            state["quit"] = True

    root.bind("<Key>", on_key)

    try:
        while not state["quit"]:
            frames = pipe.wait_for_frames(5000)
            live = np.asanyarray(frames.get_color_frame().get_data())
            live_small = preprocess_frame(live, cam["crop"], (cmp_w, cmp_h))

            mode = MODES[state["mode"]]
            if mode == "blend":
                a = state["alpha"]
                comp = (a * live_small + (1 - a) * sim_small).astype(np.uint8)
            elif mode == "edges":
                comp = live_small.copy()
                comp[edge_mask] = (0, 255, 0)
            else:  # flicker at ~2 Hz: residual motion pops out to the eye
                comp = live_small if int(time.monotonic() * 4) % 2 else sim_small
            state["composite"] = comp

            photo = ImageTk.PhotoImage(
                Image.fromarray(comp).resize(view, Image.Resampling.NEAREST)
            )
            label.configure(image=photo)
            label.image = photo
            status.set(f"mode={mode}  alpha={state['alpha']:.2f}   "
                       "[a/d] alpha  [m] mode  [s] save  [q] quit")
            root.update()
    except tk.TclError:
        pass  # window closed
    finally:
        pipe.stop()


if __name__ == "__main__":
    main()
