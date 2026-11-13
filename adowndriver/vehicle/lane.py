"""
Node 2 road perception: bird's-eye warp, colour segmentation of the yellow
(left) and white (right) tape, sliding-window search, 2nd-order polynomial fit,
lane centre / heading estimation and the lane confidence C_lane.

Why not "mean of all lane pixels" (revision 1)?  When only one boundary is
visible - in every curve and whenever the robot drifts - the mean collapses onto
that single line and the robot steers *into* it. Here each boundary is tracked
separately and a single visible line is offset by half the known lane width.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

from ..config import CameraModelConfig, LaneConfig
from .camera_model import bev_geometry, default_homography


@dataclass
class LineFit:
    coeffs: Optional[np.ndarray] = None   # x = a*y^2 + b*y + c (BEV pixels)
    quality: float = 0.0                  # fraction of windows with tape
    points: int = 0
    xs: Optional[np.ndarray] = None
    ys: Optional[np.ndarray] = None

    @property
    def ok(self) -> bool:
        return self.coeffs is not None

    def x_at(self, y: float) -> float:
        a, b, c = self.coeffs
        return float(a * y * y + b * y + c)

    def slope_at(self, y: float) -> float:
        a, b, _ = self.coeffs
        return float(2 * a * y + b)


@dataclass
class LaneResult:
    c_lane: float = 0.0
    e_y_m: float = 0.0          # lateral error of lane centre at the look-ahead point (+ = lane centre to the right)
    e_axle_m: float = 0.0       # same convention, at the wheel axle (extrapolated; VehicleCore refines it by odometry)
    e_near_m: float = 0.0       # lane-centre offset at the nearest visible row (bev_near_m ahead of the axle), unfiltered
    heading_rad: float = 0.0    # lane direction vs. robot heading at the BEV bottom (+ = lane points right)
    curvature: float = 0.0      # 1/m, + = lane bends to the right (feed-forward term)
    width_m: float = 0.0
    left: LineFit = field(default_factory=LineFit)
    right: LineFit = field(default_factory=LineFit)
    proc_ms: float = 0.0
    mode: str = "NONE"          # BOTH | LEFT_ONLY | RIGHT_ONLY | NONE
    bev: Optional[np.ndarray] = None
    debug: Optional[np.ndarray] = None


class LaneDetector:
    def __init__(self, lane_cfg: LaneConfig, cam_cfg: CameraModelConfig):
        self.cfg = lane_cfg
        self.cam_cfg = cam_cfg
        self.bw, self.bh, self.ppm = bev_geometry(lane_cfg)
        self.M = None
        self.frame_size = None
        self.prev_base = {"left": None, "right": None}
        self.width_est_px = lane_cfg.lane_width_m * self.ppm
        self.e_f = self.h_f = self.k_f = self.el_f = None

    # ------------------------------------------------------------------ #
    def _ensure_homography(self, w: int, h: int) -> None:
        if self.frame_size != (w, h):
            self.M, _ = default_homography(self.cam_cfg, self.cfg, w, h)
            self.frame_size = (w, h)
            # BEV pixels that the camera really sees, shrunk by ~1 tape width: tape cut by the
            # field-of-view edge would otherwise bias the line position towards the centre
            seen = cv2.warpPerspective(np.full((h, w), 255, np.uint8), self.M, (self.bw, self.bh))
            k = max(3, int(self.ppm * 0.03) | 1)
            self.valid = cv2.erode((seen > 250).astype(np.uint8), np.ones((k, k), np.uint8))

    def recompute(self) -> None:
        self.frame_size = None
        self.bw, self.bh, self.ppm = bev_geometry(self.cfg)

    # ------------------------------------------------------------------ #
    def segment(self, bev: np.ndarray):
        c = self.cfg
        hls = cv2.cvtColor(bev, cv2.COLOR_BGR2HLS)
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        L, S = hls[..., 1], hls[..., 2]
        # Adaptive white threshold: robust to the LED-lamp brightness tests
        p99 = float(np.percentile(L, 99.5))
        med = float(np.median(L))
        l_thr = max(min(c.white_l_min, 0.72 * p99), med + 45, 60)
        white = ((L >= l_thr) & (S <= c.white_s_max)).astype(np.uint8)
        Hh, Sv, V = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        v_thr = max(min(c.yellow_v_min, 0.6 * float(np.percentile(V, 99.5))), 45)
        yellow = ((Hh >= c.yellow_h_lo) & (Hh <= c.yellow_h_hi) & (Sv >= c.yellow_s_min) & (V >= v_thr)).astype(np.uint8)
        white[yellow > 0] = 0
        # Glare suppression: specular blobs from the LED lamp are much wider than tape;
        # a morphological opening with a kernel wider than the tape keeps only the blobs.
        kb = max(5, int(self.ppm * 0.045) | 1)
        blobs = cv2.morphologyEx(white, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kb, kb)))
        if blobs.any():
            white[cv2.dilate(blobs, np.ones((5, 5), np.uint8)) > 0] = 0
        k = np.ones((3, 3), np.uint8)
        white = cv2.morphologyEx(white, cv2.MORPH_OPEN, k)
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, k)
        masks = {"yellow": yellow, "white": white}
        return masks[c.left_color], masks[c.right_color]

    def _sliding_window(self, mask: np.ndarray, side: str, windows_out: list) -> LineFit:
        c = self.cfg
        h, w = mask.shape
        expected = w / 2 + (-1 if side == "left" else 1) * self.width_est_px / 2
        base = self.prev_base[side]
        hist = mask[h // 2:, :].sum(axis=0).astype(np.float32)
        lo, hi = (0, int(w * 0.62)) if side == "left" else (int(w * 0.38), w)
        if base is None:
            prior = np.exp(-0.5 * ((np.arange(w) - expected) / (w * 0.25)) ** 2)
        else:
            prior = np.exp(-0.5 * ((np.arange(w) - base) / (w * 0.12)) ** 2)
        score = hist * prior
        score[:lo] = 0
        score[hi:] = 0
        if score.max() < c.min_window_px:
            self.prev_base[side] = None
            return LineFit()
        x_cur = float(np.argmax(score))
        margin = int(c.window_margin_m * self.ppm)
        win_h = h // c.n_windows
        ys, xs = np.nonzero(mask)
        sel_x, sel_y = [], []
        found = 0
        for i in range(c.n_windows):
            y_hi = h - i * win_h
            y_lo = y_hi - win_h
            x_lo, x_hi = int(x_cur - margin), int(x_cur + margin)
            inside = (ys >= y_lo) & (ys < y_hi) & (xs >= x_lo) & (xs < x_hi)
            n = int(inside.sum())
            area = win_h * 2 * margin
            if c.min_window_px <= n <= 0.65 * area:  # reject empty windows and glare blobs
                found += 1
                sel_x.append(xs[inside])
                sel_y.append(ys[inside])
                x_cur = float(np.mean(xs[inside]))
                windows_out.append((x_lo, y_lo, x_hi, y_hi, True))
            else:
                windows_out.append((x_lo, y_lo, x_hi, y_hi, False))
        if found < 3:
            self.prev_base[side] = None
            return LineFit()
        X = np.concatenate(sel_x)
        Y = np.concatenate(sel_y)
        deg = 2 if (Y.max() - Y.min()) > h * 0.5 else 1
        coeffs = np.polyfit(Y, X, deg)
        if deg == 1:
            coeffs = np.array([0.0, coeffs[0], coeffs[1]])
        fit = LineFit(coeffs, found / c.n_windows, int(X.size), X.astype(np.float64), Y.astype(np.float64))
        self.prev_base[side] = fit.x_at(h - 1)
        return fit

    @staticmethod
    def _joint_fit(left: LineFit, right: LineFit) -> None:
        """Lane boundaries are parallel: fit both with shared curvature and slope
        (x = a y^2 + b y + c_side). Halves the noise and stops one partially
        visible line from bending the estimated centre."""
        ys = np.concatenate([left.ys, right.ys])
        xs = np.concatenate([left.xs, right.xs])
        is_r = np.concatenate([np.zeros(left.ys.size), np.ones(right.ys.size)])
        A = np.stack([ys ** 2, ys, 1 - is_r, is_r], axis=1)
        w = np.concatenate([np.full(left.ys.size, 1 / left.ys.size), np.full(right.ys.size, 1 / right.ys.size)])
        sw = np.sqrt(w)[:, None]
        sol, *_ = np.linalg.lstsq(A * sw, xs * sw[:, 0], rcond=None)
        a, b, cl, cr = sol
        if cr - cl > 0:
            left.coeffs = np.array([a, b, cl])
            right.coeffs = np.array([a, b, cr])

    # ------------------------------------------------------------------ #
    def process(self, frame_bgr: np.ndarray, want_debug: bool = False) -> LaneResult:
        t0 = time.perf_counter()
        c = self.cfg
        fh, fw = frame_bgr.shape[:2]
        self._ensure_homography(fw, fh)
        bev = cv2.warpPerspective(frame_bgr, self.M, (self.bw, self.bh), flags=cv2.INTER_LINEAR)
        lmask, rmask = self.segment(bev)
        lmask &= self.valid
        rmask &= self.valid
        windows: list = []
        left = self._sliding_window(lmask, "left", windows)
        right = self._sliding_window(rmask, "right", windows)

        if left.ok and right.ok:
            self._joint_fit(left, right)
        res = LaneResult(left=left, right=right, bev=bev)
        h = self.bh
        y_look = float(np.clip((c.bev_far_m - c.lookahead_m) * self.ppm, 0, h - 1))
        y_bot = h - 1.0
        nominal = c.lane_width_m * self.ppm

        def centre(y):
            if left.ok and right.ok:
                return 0.5 * (left.x_at(y) + right.x_at(y)), 0.5 * (left.slope_at(y) + right.slope_at(y))
            if left.ok:
                return left.x_at(y) + self.width_est_px / 2, left.slope_at(y)
            return right.x_at(y) - self.width_est_px / 2, right.slope_at(y)

        if left.ok or right.ok:
            conf = 0.0
            if left.ok and right.ok:
                width = right.x_at(y_bot) - left.x_at(y_bot)
                ratio = width / nominal
                plaus = float(np.clip(1.0 - abs(ratio - 1.0) / 0.4, 0.0, 1.0))
                if plaus > 0.5:
                    self.width_est_px = 0.9 * self.width_est_px + 0.1 * width
                    conf = (0.5 + 0.5 * plaus) * (0.5 * (left.quality + right.quality)) ** 0.5
                    res.mode = "BOTH"
                else:  # implausible width - keep the better line only
                    if left.quality >= right.quality:
                        res.right = right = LineFit()
                    else:
                        res.left = left = LineFit()
                res.width_m = width / self.ppm
            if res.mode != "BOTH":
                q = left.quality if left.ok else right.quality
                conf = 0.8 * q ** 0.5
                res.mode = "LEFT_ONLY" if left.ok else "RIGHT_ONLY"
                res.width_m = self.width_est_px / self.ppm
            x_look, _ = centre(y_look)
            # evaluate the centre polynomial at the wheel axle (beyond the BEV bottom)
            y_axle = y_bot + c.bev_near_m * self.ppm
            x_axle, slope_axle = centre(y_axle)
            e_look = (x_look - self.bw / 2) / self.ppm
            e_axle = (x_axle - self.bw / 2) / self.ppm
            res.e_near_m = (centre(y_bot)[0] - self.bw / 2) / self.ppm
            heading = float(np.arctan(-slope_axle))
            # curvature of the centre line: kc = k_line / (1 - k_line * d), d = centre offset from the line
            half_w = self.width_est_px / (2 * self.ppm)
            kcs = []
            for fit, d in ((left, +half_w), (right, -half_w)):
                if fit.ok:
                    k_line = 2 * fit.coeffs[0] * self.ppm / (1 + fit.slope_at(y_bot) ** 2) ** 1.5
                    kcs.append(k_line / max(0.3, 1 - k_line * d))
            curv = float(np.mean(kcs))
            a = c.ema_alpha
            if self.e_f is None:
                self.e_f, self.h_f, self.k_f, self.el_f = e_axle, heading, curv, e_look
            else:
                self.e_f = a * e_axle + (1 - a) * self.e_f
                self.h_f = a * heading + (1 - a) * self.h_f
                self.k_f = a * curv + (1 - a) * self.k_f
                self.el_f = a * e_look + (1 - a) * self.el_f
            res.e_axle_m, res.heading_rad, res.curvature, res.e_y_m = self.e_f, self.h_f, self.k_f, self.el_f
            res.c_lane = float(np.clip(conf, 0.0, 1.0))
        else:
            self.e_f = self.h_f = None
        res.proc_ms = (time.perf_counter() - t0) * 1000.0
        if want_debug:
            res.debug = self.draw_debug(bev, lmask, rmask, res, windows, y_look)
        return res

    # ------------------------------------------------------------------ #
    def draw_debug(self, bev, lmask, rmask, res: LaneResult, windows, y_look) -> np.ndarray:
        dbg = (bev * 0.55).astype(np.uint8)
        dbg[lmask > 0] = (0, 210, 255)
        dbg[rmask > 0] = (255, 255, 255)
        for x0, y0, x1, y1, ok in windows:
            cv2.rectangle(dbg, (x0, y0), (x1, y1), (0, 200, 0) if ok else (60, 60, 160), 1)
        ys = np.arange(0, self.bh, 4, dtype=np.float32)
        for fit, col in ((res.left, (0, 160, 255)), (res.right, (255, 120, 0))):
            if fit.ok:
                pts = np.stack([fit.coeffs[0] * ys ** 2 + fit.coeffs[1] * ys + fit.coeffs[2], ys], 1)
                cv2.polylines(dbg, [pts.astype(np.int32)], False, col, 2)
        cx = int(self.bw / 2)
        cv2.line(dbg, (cx, 0), (cx, self.bh), (120, 120, 120), 1)
        if res.mode != "NONE":
            px = int(self.bw / 2 + res.e_y_m * self.ppm)
            cv2.circle(dbg, (px, int(y_look)), 5, (80, 80, 255), -1)
        return dbg
