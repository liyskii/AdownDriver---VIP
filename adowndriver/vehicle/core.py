"""
Node 2 control step, shared by the real robot loop (vehicle/node.py) and the
software-in-the-loop validation runner (sim/runner.py):

    road frame -> LaneDetector -> (e_y, heading, C_lane)
    ToF        -> d_obs
    Node 1     -> DriverPacket (+ link age)
                    \\-> SafetyFSM -> v_target -> VelocityProfiler -> v_cmd
                                   lane     -> LaneKeepingController -> omega
"""
from __future__ import annotations

import collections
import json
import dataclasses
import math
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ..common.protocol import DriverPacket
from ..config import AppConfig
from .control import LaneKeepingController, VelocityProfiler
from .fsm import FsmInputs, FsmOutput, SafetyFSM, State
from .lane import LaneDetector, LaneResult


@dataclass
class StepInput:
    dt: float
    t_ms: int
    frame: Optional[np.ndarray] = None
    lane_override: Optional[LaneResult] = None
    d_obs_cm: Optional[float] = None
    tof_age_s: float = 0.0
    tof_enabled: bool = True
    driver: Optional[DriverPacket] = None
    driver_age_s: float = float("inf")
    rtt_ms: Optional[float] = None
    commands: tuple = ()
    want_debug: bool = False


@dataclass
class StepOutput:
    v_cmd: float
    omega: float
    fsm: FsmOutput
    lane: LaneResult
    proc_ms: float
    telemetry: dict = field(default_factory=dict)
    tick_row: dict = field(default_factory=dict)
    events: list = field(default_factory=list)


class VehicleCore:
    def __init__(self, cfg: AppConfig):
        self.cfg = cfg
        self.lane = LaneDetector(cfg.lane, cfg.camera)
        self.fsm = SafetyFSM(cfg.fsm)
        self.profiler = VelocityProfiler(cfg.fsm)
        self.lkc = LaneKeepingController(cfg.steering, cfg.fsm.V_NORMAL)
        self.last_lane = LaneResult()
        self.obstacle_seen_t: Optional[float] = None
        # odometry-based axle offset: a lane-centre point seen bev_near_m ahead is
        # remembered in an odometry frame and read back when the axle reaches it
        self.odo = [0.0, 0.0, 0.0]
        self.lane_pts: collections.deque = collections.deque(maxlen=240)
        self.prev_cmd = (0.0, 0.0)
        self._last_pt_pose = (0.0, 0.0)

    def _odometry(self, dt: float, lane: LaneResult, new_measurement: bool) -> Optional[float]:
        v, w = self.prev_cmd
        x, y, th = self.odo
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += w * dt
        self.odo = [x, y, th]
        near = self.cfg.lane.bev_near_m
        moved = self.lane_pts and math.hypot(x - self._last_pt_pose[0], y - self._last_pt_pose[1]) < 0.005
        if new_measurement and lane.mode != "NONE" and not moved:
            self._last_pt_pose = (x, y)
            fx, fy = near, -lane.e_near_m                     # robot frame: x forward, y left
            self.lane_pts.append((x + fx * math.cos(th) - fy * math.sin(th),
                                  y + fx * math.sin(th) + fy * math.cos(th)))
        best = None
        c, s = math.cos(th), math.sin(th)
        for px, py in self.lane_pts:
            dx, dy = px - x, py - y
            fwd, left = dx * c + dy * s, -dx * s + dy * c
            if abs(fwd) < 0.02 and (best is None or abs(fwd) < best[0]):
                best = (abs(fwd), -left)
        return None if best is None else best[1]

    def step(self, si: StepInput) -> StepOutput:
        t0 = time.perf_counter()
        if si.lane_override is not None:
            lane = si.lane_override
        elif si.frame is not None:
            lane = self.lane.process(si.frame, want_debug=si.want_debug)
        else:
            lane = self.last_lane
        if not self.cfg.vehicle.lane_keeping and si.lane_override is None:
            # demo without a taped track: keep the camera preview, but steer straight and never raise LANE_LOST
            lane = dataclasses.replace(lane, c_lane=1.0, e_y_m=0.0, e_axle_m=0.0, e_near_m=0.0,
                                       heading_rad=0.0, curvature=0.0, mode="OFF")
        new_meas = lane is not self.last_lane
        self.last_lane = lane
        e_dr = self._odometry(si.dt, lane, new_meas)
        # reported axle offset (metric M4): odometry-anchored when available, else extrapolated
        e_rep = e_dr if (e_dr is not None and lane.mode != "NONE") else lane.e_axle_m

        fout = self.fsm.update(FsmInputs(dt=si.dt, driver=si.driver, driver_age_s=si.driver_age_s,
                                         c_lane=lane.c_lane, d_obs_cm=si.d_obs_cm, tof_enabled=si.tof_enabled,
                                         commands=si.commands))
        prev_v = self.profiler.v
        self.lkc.v_nominal = self.cfg.fsm.V_NORMAL
        v_cmd = self.profiler.update(fout.v_target, si.dt, fout.hard_stop)
        lane_ok = lane.c_lane >= self.cfg.fsm.TH_CONF_MIN
        omega = self.lkc.compute(lane.e_axle_m, lane.heading_rad, v_cmd, si.dt, lane_ok, lane.curvature)
        if fout.state == State.EMERGENCY_STOP:
            omega = 0.0
        self.prev_cmd = (v_cmd, omega)
        proc_ms = (time.perf_counter() - t0) * 1000.0

        events = [(k, " ".join(str(x) for x in rest)) for k, *rest in fout.events]
        # ---- braking latency evidence for metric M5 -----------------------
        if fout.hard_stop and prev_v > 0 and v_cmd == 0.0:
            reason = fout.reason
            if reason == "HEARTBEAT_LOSS":
                lat = si.driver_age_s * 1000.0 + proc_ms
            elif reason == "OBSTACLE":
                lat = (si.tof_age_s + 1.0 / 30.0) * 1000.0 + proc_ms   # + one ToF period (worst case)
            else:
                lat = proc_ms
            events.append(("BRAKE", json.dumps({"reason": reason, "latency_ms": round(lat, 1),
                                                "v_before": round(prev_v, 3)})))

        snap = self.fsm.snapshot()
        drv = si.driver
        telemetry = {
            "state": fout.state.value, "reason": fout.reason, "cap": fout.speed_cap_reason,
            "v_cmd": round(v_cmd, 4), "v_target": round(fout.v_target, 4), "omega": round(omega, 4),
            "buzzer": fout.buzzer, "led": fout.led,
            "lane": {"c": round(lane.c_lane, 3), "e_y_cm": round(lane.e_y_m * 100, 2),
                     "e_axle_cm": round(e_rep * 100, 2), "heading_deg": round(math.degrees(lane.heading_rad), 1),
                     "width_cm": round(lane.width_m * 100, 1), "mode": lane.mode, "proc_ms": round(lane.proc_ms, 2)},
            "tof": {"d_cm": None if si.d_obs_cm is None else round(si.d_obs_cm, 1), "enabled": si.tof_enabled},
            "driver": None if drv is None else {
                "ear": drv.ear, "mar": drv.mar, "ear_th": drv.ear_th, "c": drv.c_driver, "face": drv.face,
                "yaw": drv.yaw_deg, "fps": drv.fps, "proc_ms": drv.proc_ms, "gt": drv.gt},
            "link": {"age_ms": None if not math.isfinite(si.driver_age_s) else round(si.driver_age_s * 1000, 1),
                     "rtt_ms": si.rtt_ms},
            "fsm": snap, "proc_ms": round(proc_ms, 2),
            "params": {"TH_MAR": self.cfg.fsm.TH_MAR, "D_SAFE_CM": self.cfg.fsm.D_SAFE_CM,
                       "D_SLOW_CM": self.cfg.fsm.D_SLOW_CM, "V_NORMAL": self.cfg.fsm.V_NORMAL,
                       "T_WARN": self.cfg.fsm.T_WARN, "T_SLOW": self.cfg.fsm.T_SLOW, "T_STOP": self.cfg.fsm.T_STOP},
        }
        tick_row = {
            "t_ms": si.t_ms, "state": fout.state.value, "reason": fout.reason, "v_cmd": round(v_cmd, 4),
            "omega": round(omega, 4), "ear": drv.ear if drv else None, "mar": drv.mar if drv else None,
            "ear_th": drv.ear_th if drv else None, "c_driver": drv.c_driver if drv else None,
            "c_lane": round(lane.c_lane, 3), "e_y_cm": round(lane.e_y_m * 100, 2),
            "e_axle_cm": round(e_rep * 100, 2), "heading_deg": round(math.degrees(lane.heading_rad), 2),
            "d_obs_cm": si.d_obs_cm, "t_closed": snap["t_closed"], "perclos": snap["perclos"],
            "link_age_ms": telemetry["link"]["age_ms"], "node1_proc_ms": drv.proc_ms if drv and drv.proc_ms >= 0 else None,
            "node2_proc_ms": round(proc_ms, 2), "rtt_ms": si.rtt_ms, "gt": drv.gt if drv else "",
            "buzzer": fout.buzzer, "led": fout.led,
        }
        return StepOutput(v_cmd, omega, fout, lane, proc_ms, telemetry, tick_row, events)
