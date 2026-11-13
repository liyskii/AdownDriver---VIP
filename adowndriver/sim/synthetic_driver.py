"""
Scripted synthetic driver for software-in-the-loop tests and for running
Node 1 without a webcam (``--source sim``). Produces the same raw signals as the
real face pipeline (EAR, MAR, yaw, face-present, luminance) and pushes them
through the real DriverSignalProcessor (EMA + confidence).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..config import DriverMonitorConfig
from ..driver_monitor.analyzer import DriverSignalProcessor, FaceObservation

BEHAVIOURS = ("alert", "blink_fast", "talk", "closed", "yawn", "look_away", "occluded", "glare", "dim")


@dataclass
class Segment:
    t0: float
    t1: float
    behaviour: str
    label: str = ""   # ground truth: alert | drowsy | yawn | distracted | absent


class SyntheticDriver:
    def __init__(self, cfg: DriverMonitorConfig, timeline: list[Segment], seed: int = 0,
                 ear_open: float = 0.30, blink_period=(3.0, 5.0), blink_dur=(0.12, 0.30)):
        self.cfg = cfg
        self.timeline = timeline
        self.rng = np.random.default_rng(seed)
        self.proc = DriverSignalProcessor(cfg)
        self.ear_open = ear_open
        self.blink_period = blink_period
        self.blink_dur = blink_dur
        self.next_blink = self.rng.uniform(*blink_period)
        self.blink_end = -1.0
        self.blinks: list[tuple[float, float]] = []
        self.talk_spike_end = -1.0

    def segment_at(self, t: float) -> Segment:
        for s in self.timeline:
            if s.t0 <= t < s.t1:
                return s
        return Segment(t, t + 1, "alert", "alert")

    def observe(self, t: float) -> FaceObservation:
        seg = self.segment_at(t)
        b = seg.behaviour
        n = self.rng.normal
        ear = self.ear_open + n(0, 0.008)
        mar = 0.08 + abs(n(0, 0.02))
        yaw = n(0, 3.0)
        lum, width, face = 120.0, 160.0, True

        if b in ("alert", "talk", "glare", "dim", "blink_fast"):
            period = (1.8, 2.2) if b == "blink_fast" else self.blink_period
            dur = (0.28, 0.35) if b == "blink_fast" else self.blink_dur
            if t >= self.next_blink and t > self.blink_end:
                d = float(self.rng.uniform(*dur))
                self.blink_end = t + d
                self.blinks.append((t, d))
                self.next_blink = t + d + float(self.rng.uniform(*period))
            if t < self.blink_end:
                ear = 0.10 + n(0, 0.01)
        if b == "talk":
            mar = 0.30 + 0.18 * abs(math.sin(2 * math.pi * 2.7 * t)) + n(0, 0.03)
            if self.rng.random() < 0.01 and t > self.talk_spike_end:
                self.talk_spike_end = t + 0.3
            if t < self.talk_spike_end:
                mar = 0.70
        elif b == "closed":
            ear = 0.11 + n(0, 0.01)
            yaw = n(0, 2) + 3
        elif b == "yawn":
            p = (t - seg.t0) / max(0.1, seg.t1 - seg.t0)
            env = min(1.0, p / 0.2, (1 - p) / 0.15) if 0 <= p <= 1 else 0
            mar = 0.10 + 0.80 * max(0.0, env) + n(0, 0.02)
            ear = self.ear_open * (1 - 0.25 * max(0.0, env)) + n(0, 0.008)
        elif b == "look_away":
            yaw = 45 + n(0, 3)
        elif b == "occluded":
            face = False
        elif b == "glare":
            lum = 212 + n(0, 4)
            ear += n(0, 0.02)
        elif b == "dim":
            lum = 48 + n(0, 3)
        if not face:
            return FaceObservation()
        return FaceObservation(True, ear, ear, ear, max(0.0, mar), yaw, 0.0, width, lum)

    def sample(self, t: float) -> dict:
        obs = self.observe(t)
        out = self.proc.update(obs)
        out["gt"] = self.segment_at(t).label
        return out
