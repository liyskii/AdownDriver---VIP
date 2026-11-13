"""
Node 1 driver-state analysis: EAR, MAR, head pose, driver confidence
C_driver, EMA smoothing and per-driver calibration.

Landmark indices refer to the MediaPipe Face Landmarker topology (478 points,
468 face-mesh points + 10 iris points).

    EAR = (|p2-p6| + |p3-p5|) / (2 |p1-p4|)
    MAR = (|m2-m8| + |m3-m7| + |m4-m6|) / (2 |m1-m5|)
"""
from __future__ import annotations

import json
import math
import statistics
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from ..common.filters import EMAFilter
from ..config import DriverMonitorConfig

LEFT_EYE_IDX = [33, 160, 158, 133, 153, 144]       # p1..p6
RIGHT_EYE_IDX = [362, 385, 387, 263, 373, 380]
INNER_LIP_IDX = [78, 81, 13, 311, 308, 402, 14, 178]  # m1..m8
NOSE_TIP = 1
FACE_OVAL = [10, 338, 297, 332, 284, 251, 389, 356, 454, 323, 361, 288, 397, 365, 379, 378, 400, 377,
             152, 148, 176, 149, 150, 136, 172, 58, 132, 93, 234, 127, 162, 21, 54, 103, 67, 109]


def _d(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def compute_ear(pts: Sequence, idx=LEFT_EYE_IDX) -> float:
    p1, p2, p3, p4, p5, p6 = (pts[i] for i in idx)
    h = _d(p1, p4)
    return 0.30 if h < 1e-6 else (_d(p2, p6) + _d(p3, p5)) / (2.0 * h)


def compute_mar(pts: Sequence, idx=INNER_LIP_IDX) -> float:
    m1, m2, m3, m4, m5, m6, m7, m8 = (pts[i] for i in idx)
    h = _d(m1, m5)
    return 0.0 if h < 1e-6 else (_d(m2, m8) + _d(m3, m7) + _d(m4, m6)) / (2.0 * h)


def head_pose_from_matrix(mat: np.ndarray) -> tuple[float, float]:
    """(yaw, pitch) in degrees from MediaPipe's facial transformation matrix."""
    R = np.asarray(mat)[:3, :3]
    yaw = math.degrees(math.asin(max(-1.0, min(1.0, -R[2, 0]))))
    pitch = math.degrees(math.atan2(R[2, 1], R[2, 2]))
    return yaw, pitch


def head_yaw_from_landmarks(pts: Sequence) -> float:
    """Geometric fallback: nose tip position between the outer eye corners."""
    l, r, n = pts[33], pts[263], pts[NOSE_TIP]
    span = r[0] - l[0]
    if abs(span) < 1e-6:
        return 0.0
    ratio = (n[0] - l[0]) / span
    return float(max(-90.0, min(90.0, (ratio - 0.5) * 110.0)))


@dataclass
class FaceObservation:
    face: bool = False
    ear_raw: float = 0.30
    ear_l: float = 0.30
    ear_r: float = 0.30
    mar_raw: float = 0.0
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    face_width_px: float = 0.0
    luminance: float = 128.0
    pts: Optional[list] = None
    bbox: Optional[tuple] = None


def observe(pts_px: Optional[list], frame_gray: Optional[np.ndarray], matrix=None) -> FaceObservation:
    """Turn a landmark list (pixel coordinates) into a FaceObservation."""
    if not pts_px:
        return FaceObservation()
    ear_l = compute_ear(pts_px, LEFT_EYE_IDX)
    ear_r = compute_ear(pts_px, RIGHT_EYE_IDX)
    if matrix is not None:
        yaw, pitch = head_pose_from_matrix(matrix)
    else:
        yaw, pitch = head_yaw_from_landmarks(pts_px), 0.0
    xs = [pts_px[i][0] for i in FACE_OVAL]
    ys = [pts_px[i][1] for i in FACE_OVAL]
    x0, x1, y0, y1 = int(min(xs)), int(max(xs)), int(min(ys)), int(max(ys))
    lum = 128.0
    if frame_gray is not None:
        h, w = frame_gray.shape[:2]
        roi = frame_gray[max(0, y0):min(h, y1), max(0, x0):min(w, x1)]
        if roi.size:
            lum = float(roi.mean())
    # When the head is turned, the far eye is foreshortened; trust the nearer one more
    if abs(yaw) > 15:
        ear = ear_r if yaw > 0 else ear_l
        ear = 0.5 * ear + 0.25 * (ear_l + ear_r)
    else:
        ear = 0.5 * (ear_l + ear_r)
    return FaceObservation(True, ear, ear_l, ear_r, compute_mar(pts_px), yaw, pitch,
                           float(x1 - x0), lum, pts_px, (x0, y0, x1, y1))


def driver_confidence(obs: FaceObservation, cfg: DriverMonitorConfig) -> float:
    """C_driver in [0, 1]: face present x size x pose x exposure."""
    if not obs.face:
        return 0.0
    mw = cfg.min_face_width_px
    size_f = float(np.clip((obs.face_width_px - 0.5 * mw) / (0.5 * mw), 0.0, 1.0))
    a = abs(obs.yaw_deg)
    m = cfg.max_reliable_yaw_deg
    if a <= m:
        pose_f = 1.0
    elif a <= 60:
        pose_f = 1.0 - 0.3 * (a - m) / (60 - m)
    else:
        pose_f = max(0.2, 0.7 - 0.5 * (a - 60) / 20)
    L = obs.luminance
    if 55 <= L <= 205:
        exp_f = 1.0
    elif L < 55:
        exp_f = max(0.3, 1.0 - (55 - L) / 50)
    else:
        exp_f = max(0.3, 1.0 - (L - 205) / 50)
    return round(size_f * pose_f * exp_f, 3)


# --------------------------------------------------------------------------- #
@dataclass
class DriverProfile:
    name: str = "default"
    ear_open: float = 0.0
    ear_th: float = 0.21
    mar_rest: float = 0.0
    created: str = ""

    def save(self, folder: Path) -> Path:
        folder.mkdir(parents=True, exist_ok=True)
        p = folder / f"{self.name}.json"
        p.write_text(json.dumps(self.__dict__, indent=2))
        return p

    @classmethod
    def load(cls, folder: Path, name: str) -> Optional["DriverProfile"]:
        p = folder / f"{name}.json"
        if p.is_file():
            return cls(**json.loads(p.read_text()))
        return None


@dataclass
class Calibrator:
    """Collects open-eye EAR for a few seconds and derives TH_EAR."""
    cfg: DriverMonitorConfig
    name: str = "default"
    t_start: float = field(default_factory=time.monotonic)
    ears: list = field(default_factory=list)
    mars: list = field(default_factory=list)

    @property
    def progress(self) -> float:
        return min(1.0, (time.monotonic() - self.t_start) / self.cfg.calibration_seconds)

    def add(self, obs: FaceObservation, c_driver: float) -> None:
        if obs.face and c_driver >= 0.65 and abs(obs.yaw_deg) < 20:
            self.ears.append(obs.ear_raw)
            self.mars.append(obs.mar_raw)

    def done(self) -> bool:
        return self.progress >= 1.0

    def result(self) -> Optional[DriverProfile]:
        if len(self.ears) < 20:
            return None
        # upper 70 % of samples - excludes blinks recorded during calibration
        ears = sorted(self.ears)[int(len(self.ears) * 0.3):]
        ear_open = statistics.median(ears)
        th = float(np.clip(self.cfg.ear_threshold_ratio * ear_open,
                           self.cfg.ear_threshold_min, self.cfg.ear_threshold_max))
        return DriverProfile(self.name, round(ear_open, 4), round(th, 4),
                             round(statistics.median(self.mars), 4), time.strftime("%Y-%m-%d %H:%M:%S"))


class DriverSignalProcessor:
    """EMA filtering + confidence; produces the numbers that go into a DriverPacket."""

    def __init__(self, cfg: DriverMonitorConfig):
        self.cfg = cfg
        self.ear = EMAFilter(cfg.ema_alpha, 0.30)
        self.mar = EMAFilter(cfg.ema_alpha, 0.10)
        self.yaw = EMAFilter(0.5, 0.0)
        self.ear_th = cfg.default_ear_threshold

    def update(self, obs: FaceObservation) -> dict:
        c = driver_confidence(obs, self.cfg)
        if obs.face and c >= 0.3:
            self.ear.update(obs.ear_raw)
            self.mar.update(obs.mar_raw)
            self.yaw.update(obs.yaw_deg)
        return {
            "face": obs.face, "ear": round(self.ear.value, 4), "mar": round(self.mar.value, 4),
            "ear_raw": round(obs.ear_raw, 4), "ear_th": self.ear_th, "c_driver": c,
            "yaw_deg": round(self.yaw.value if obs.face else obs.yaw_deg, 1),
            "pitch_deg": round(obs.pitch_deg, 1),
        }
