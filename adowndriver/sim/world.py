"""
Software-in-the-loop world: a stadium-shaped EVA-mat track with yellow (left)
and white (right) tape, a unicycle TurtleBot model, obstacles for the ToF
sensor, and a photometric road-camera renderer that reuses the real pinhole
model, so the *real* lane detector runs on the synthetic images.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

from ..config import CameraModelConfig, LaneConfig
from ..vehicle.camera_model import GroundCamera

RES = 0.004  # metres per texture pixel


@dataclass
class Obstacle:
    x: float
    y: float
    radius: float = 0.05
    active: bool = True


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    th: float = 0.0
    v: float = 0.0
    w: float = 0.0


class Track:
    def __init__(self, straight: float = 2.0, radius: float = 0.65, lane_width: float = 0.30,
                 tape: float = 0.025, gaps: Optional[list] = None):
        self.L, self.R, self.lw, self.tape = straight, radius, lane_width, tape
        self.margin = 0.6
        self.xmin, self.xmax = -straight / 2 - radius - self.margin, straight / 2 + radius + self.margin
        self.ymin, self.ymax = -radius - self.margin, radius + self.margin
        self.centre = self._centreline(0.005)
        self.length = float(np.sum(np.linalg.norm(np.diff(self.centre, axis=0), axis=1)))
        self.gaps = gaps or []
        self.texture = self._render_texture()

    def _centreline(self, ds: float) -> np.ndarray:
        L, R = self.L, self.R
        pts = []
        # counter-clockwise: bottom straight left->right, right arc, top straight right->left, left arc
        n = int(L / ds)
        pts += [(-L / 2 + i * ds, -R) for i in range(n)]
        m = int(math.pi * R / ds)
        pts += [(L / 2 + R * math.cos(-math.pi / 2 + math.pi * i / m), R * math.sin(-math.pi / 2 + math.pi * i / m)) for i in range(m)]
        pts += [(L / 2 - i * ds, R) for i in range(n)]
        pts += [(-L / 2 + R * math.cos(math.pi / 2 + math.pi * i / m), R * math.sin(math.pi / 2 + math.pi * i / m)) for i in range(m)]
        return np.array(pts, dtype=np.float64)

    def to_px(self, xy: np.ndarray) -> np.ndarray:
        return np.stack([(xy[:, 0] - self.xmin) / RES, (self.ymax - xy[:, 1]) / RES], axis=1)

    def _offset(self, d: float) -> np.ndarray:
        c = self.centre
        t = np.gradient(c, axis=0)
        t /= np.linalg.norm(t, axis=1, keepdims=True)
        n_left = np.stack([-t[:, 1], t[:, 0]], axis=1)
        return c + n_left * d

    def _render_texture(self) -> np.ndarray:
        w = int((self.xmax - self.xmin) / RES)
        h = int((self.ymax - self.ymin) / RES)
        rng = np.random.default_rng(1)
        tex = np.full((h, w, 3), 28, np.uint8)  # matte black EVA foam
        tex = cv2.add(tex, rng.integers(0, 8, (h, w, 1), dtype=np.uint8).repeat(3, 2))
        th = max(1, int(round(self.tape / RES)))
        left = self.to_px(self._offset(+self.lw / 2)).astype(np.int32)
        right = self.to_px(self._offset(-self.lw / 2)).astype(np.int32)
        cv2.polylines(tex, [left], True, (20, 200, 235), th, cv2.LINE_AA)   # yellow (BGR)
        cv2.polylines(tex, [right], True, (235, 235, 235), th, cv2.LINE_AA)  # white
        for s0, s1 in self.gaps:  # missing tape segments (lane-loss scenario)
            i0, i1 = int(s0 / 0.005), int(s1 / 0.005)
            seg = self.to_px(self.centre[i0:i1]).astype(np.int32)
            cv2.polylines(tex, [seg], False, (28, 28, 28), int((self.lw + 0.08) / RES))
        return tex

    def pose_at(self, s: float):
        i = int((s % self.length) / 0.005) % len(self.centre)
        j = (i + 1) % len(self.centre)
        p, q = self.centre[i], self.centre[j]
        return float(p[0]), float(p[1]), math.atan2(q[1] - p[1], q[0] - p[0])

    def lateral_error(self, x: float, y: float):
        """Signed distance to the centre line (+ = robot left of centre) and arc length."""
        d = self.centre - np.array([x, y])
        i = int(np.argmin(np.einsum("ij,ij->i", d, d)))
        j = (i + 1) % len(self.centre)
        t = self.centre[j] - self.centre[i]
        t /= np.linalg.norm(t)
        rel = np.array([x, y]) - self.centre[i]
        return float(t[0] * rel[1] - t[1] * rel[0]), i * 0.005


class SimWorld:
    def __init__(self, cam_cfg: CameraModelConfig, lane_cfg: LaneConfig, gaps=None, seed: int = 0):
        self.track = Track(lane_width=lane_cfg.lane_width_m, gaps=gaps)
        self.cam = GroundCamera(cam_cfg)
        self.f, self.r, self.valid = self.cam.pixel_rays_to_ground()
        x, y, th = self.track.pose_at(0.3)
        self.robot = RobotState(x, y, th)
        self.obstacles: list[Obstacle] = []
        self.rng = np.random.default_rng(seed)
        self.brightness = 1.0
        self.glare: Optional[tuple] = None    # (u, v, radius) in camera pixels
        self.lens_blocked = False
        self.tau = 0.08                       # motor time constant (s)
        self.front_offset = 0.09              # ToF sensor ahead of robot centre (m)
        self.travelled = 0.0

    # ------------------------------------------------------------------ #
    def step(self, v_cmd: float, w_cmd: float, dt: float) -> None:
        r = self.robot
        a = min(1.0, dt / self.tau)
        r.v += (v_cmd - r.v) * a
        r.w += (w_cmd - r.w) * a
        r.x += r.v * math.cos(r.th) * dt
        r.y += r.v * math.sin(r.th) * dt
        r.th = (r.th + r.w * dt + math.pi) % (2 * math.pi) - math.pi
        self.travelled += abs(r.v) * dt

    def place_obstacle_ahead(self, distance: float) -> Obstacle:
        _, s = self.track.lateral_error(self.robot.x, self.robot.y)
        x, y, _ = self.track.pose_at(s + distance)
        ob = Obstacle(x, y)
        self.obstacles.append(ob)
        return ob

    def tof_cm(self, noise: bool = True) -> float:
        r = self.robot
        sx = r.x + self.front_offset * math.cos(r.th)
        sy = r.y + self.front_offset * math.sin(r.th)
        best = 200.0
        for ob in self.obstacles:
            if not ob.active:
                continue
            dx, dy = ob.x - sx, ob.y - sy
            along = dx * math.cos(r.th) + dy * math.sin(r.th)
            perp = -dx * math.sin(r.th) + dy * math.cos(r.th)
            reach = ob.radius + 0.02 + along * math.tan(math.radians(13.5))  # VL53L1X 27 deg FoV
            if along > -ob.radius and abs(perp) < reach:
                dist = max(0.0, along - math.sqrt(max(0.0, ob.radius ** 2 - min(perp ** 2, ob.radius ** 2))))
                best = min(best, dist * 100)
        if noise:
            best += float(self.rng.normal(0, 0.5))
        return max(0.0, best)

    def min_clearance_cm(self) -> float:
        r = self.robot
        best = 999.0
        for ob in self.obstacles:
            if ob.active:
                best = min(best, (math.hypot(ob.x - r.x, ob.y - r.y) - ob.radius - self.front_offset) * 100)
        return best

    def render(self) -> np.ndarray:
        r = self.robot
        c, s = math.cos(r.th), math.sin(r.th)
        wx = r.x + self.f * c + self.r * s
        wy = r.y + self.f * s - self.r * c
        mx = ((wx - self.track.xmin) / RES).astype(np.float32)
        my = ((self.track.ymax - wy) / RES).astype(np.float32)
        img = cv2.remap(self.track.texture, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                        borderValue=(55, 55, 60))
        img[~self.valid] = (70, 65, 60)  # wall / background above the horizon
        # obstacles drawn as dark-red blobs on the ground footprint
        for ob in self.obstacles:
            if ob.active:
                d2 = (wx - ob.x) ** 2 + (wy - ob.y) ** 2
                img[(d2 < ob.radius ** 2) & self.valid] = (40, 40, 150)
        out = img.astype(np.float32) * self.brightness
        if self.glare is not None:
            u, v, rad = self.glare
            yy, xx = np.mgrid[0:img.shape[0], 0:img.shape[1]]
            g = np.exp(-(((xx - u) ** 2 + (yy - v) ** 2) / (2 * rad ** 2)))
            out += (g * 200)[..., None]
        out += self.rng.normal(0, 3.0, out.shape)
        if self.lens_blocked:
            out[:] = 8
        return np.clip(out, 0, 255).astype(np.uint8)
