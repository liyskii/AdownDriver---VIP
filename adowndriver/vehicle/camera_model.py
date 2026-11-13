"""
Pinhole ground-plane camera model shared by the lane detector (default
bird's-eye homography) and the simulator (synthetic road images).

Robot frame on the ground: f = metres forward of the wheel axle, r = metres to
the right of the robot centre line.
"""
from __future__ import annotations

import math

import numpy as np

from ..config import CameraModelConfig, LaneConfig


class GroundCamera:
    def __init__(self, cfg: CameraModelConfig):
        self.cfg = cfg
        self.w, self.h = cfg.width, cfg.height
        self.cx, self.cy = self.w / 2.0, self.h / 2.0
        self.fx = self.cx / math.tan(math.radians(cfg.hfov_deg) / 2)
        self.fy = self.cy / math.tan(math.radians(cfg.vfov_deg) / 2)
        self.th = math.radians(cfg.pitch_deg)
        self.hc = cfg.height_m
        self.off = cfg.forward_offset_m

    def project(self, f: np.ndarray, r: np.ndarray):
        """Ground (f, r) -> pixel (u, v)."""
        df = np.asarray(f, dtype=np.float64) - self.off
        r = np.asarray(r, dtype=np.float64)
        zc = df * math.cos(self.th) + self.hc * math.sin(self.th)
        yc = -df * math.sin(self.th) + self.hc * math.cos(self.th)
        u = self.cx + self.fx * r / zc
        v = self.cy + self.fy * yc / zc
        return u, v

    def pixel_rays_to_ground(self):
        """For every pixel return (f, r, valid) arrays of the ground point it sees."""
        u, v = np.meshgrid(np.arange(self.w) + 0.5, np.arange(self.h) + 0.5)
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy
        fwd = math.cos(self.th) - y * math.sin(self.th)
        down = math.sin(self.th) + y * math.cos(self.th)
        valid = down > 1e-3
        t = np.where(valid, self.hc / np.maximum(down, 1e-3), 0.0)
        f = self.off + t * fwd
        r = t * x
        valid &= (f < 4.0)
        return f.astype(np.float32), r.astype(np.float32), valid


def bev_geometry(lane: LaneConfig):
    """Size of the bird's-eye image and pixel<->metre helpers."""
    ppm = lane.bev_px_per_m
    w = int(round(2 * lane.bev_lateral_m * ppm))
    h = int(round((lane.bev_far_m - lane.bev_near_m) * ppm))
    return w, h, ppm


def default_homography(cam_cfg: CameraModelConfig, lane: LaneConfig, frame_w: int, frame_h: int):
    """Homography camera-pixel -> BEV-pixel.

    Uses manually calibrated points if ``lane.src_points`` holds four [u, v]
    pairs (picked with tools/calibrate_lane.py on the calibration rectangle
    described by ``lane.calib_ground_points``); otherwise the pinhole model.
    """
    import cv2

    w, h, ppm = bev_geometry(lane)

    def ground_to_bev(f, r):
        return [(r + lane.bev_lateral_m) * ppm, (lane.bev_far_m - f) * ppm]

    if lane.src_points and len(lane.src_points) == 4:
        src = np.float32(lane.src_points)
        dst = np.float32([ground_to_bev(f, r) for f, r in lane.calib_ground_points])
    else:
        cam = GroundCamera(cam_cfg)
        corners = [(lane.bev_near_m, -lane.bev_lateral_m), (lane.bev_near_m, lane.bev_lateral_m),
                   (lane.bev_far_m, lane.bev_lateral_m), (lane.bev_far_m, -lane.bev_lateral_m)]
        u, v = cam.project(np.array([c[0] for c in corners]), np.array([c[1] for c in corners]))
        sx, sy = frame_w / cam_cfg.width, frame_h / cam_cfg.height
        src = np.float32([[a * sx, b * sy] for a, b in zip(u, v)])
        dst = np.float32([ground_to_bev(f, r) for f, r in corners])
    return cv2.getPerspectiveTransform(src, dst), (w, h)
