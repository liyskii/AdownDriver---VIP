#!/usr/bin/env python3
"""
Bird's-eye calibration + HSV check for the Node 2 road camera.

1. Put the TurtleBot on the mat. Mark a rectangle on the floor with four tape
   dots at the ground points listed in config ``lane.calib_ground_points``
   (default: 25 cm and 55 cm ahead of the wheel axle, 15 cm left and right of
   the robot centre line).
2. Run   python3 tools/calibrate_lane.py            (on the robot, with a monitor / VNC)
   or    python3 tools/calibrate_lane.py --image snapshot.jpg   (on a laptop)
3. Click the 4 dots in the order: near-left, near-right, far-right, far-left.
   The bird's-eye preview updates; press  s  to print the YAML to paste into
   config/default.yaml, r to reset clicks, q to quit.

Without clicks the preview uses the pinhole model (camera height 19.5 cm, pitch 25 deg).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from adowndriver.config import load_config  # noqa: E402
from adowndriver.vehicle.lane import LaneDetector  # noqa: E402

clicks: list = []


def on_mouse(event, x, y, *_):
    if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 4:
        clicks.append([x, y])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", help="use a still image instead of the camera")
    ap.add_argument("--source", default=None, help="camera source (default: config vehicle.camera_source)")
    ap.add_argument("--config")
    a = ap.parse_args()
    cfg = load_config(a.config)
    grab = None
    if a.image:
        still = cv2.resize(cv2.imread(a.image), (cfg.camera.width, cfg.camera.height))
    else:
        from adowndriver.vehicle.io import FrameGrabber
        grab = FrameGrabber(a.source or cfg.vehicle.camera_source, cfg.camera.width, cfg.camera.height)
    win = "AdownDriver lane calibration"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    det = LaneDetector(cfg.lane, cfg.camera)
    while True:
        frame = still.copy() if a.image else grab.read()[0]
        if frame is None:
            cv2.waitKey(30)
            continue
        if len(clicks) == 4:
            cfg.lane.src_points = clicks
            det.recompute()
        res = det.process(frame, want_debug=True)
        view = frame.copy()
        for i, (x, y) in enumerate(clicks):
            cv2.circle(view, (x, y), 4, (0, 0, 255), -1)
            cv2.putText(view, str(i + 1), (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        dbg = cv2.resize(res.debug, (int(res.debug.shape[1] * view.shape[0] / res.debug.shape[0]), view.shape[0]))
        out = np.hstack([view, dbg])
        txt = f"clicks {len(clicks)}/4  C_lane {res.c_lane:.2f}  e {res.e_axle_m*100:+.1f} cm  width {res.width_m*100:.0f} cm"
        cv2.putText(out, txt, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        cv2.imshow(win, cv2.resize(out, None, fx=2, fy=2))
        k = cv2.waitKey(30) & 0xFF
        if k == ord("q"):
            break
        if k == ord("r"):
            clicks.clear()
            cfg.lane.src_points = []
            det.recompute()
        if k == ord("s"):
            print("\n# paste into config/default.yaml\nlane:\n"
                  f"  src_points: {clicks}\n  calib_ground_points: {cfg.lane.calib_ground_points}\n")
    if grab:
        grab.close()


if __name__ == "__main__":
    main()
