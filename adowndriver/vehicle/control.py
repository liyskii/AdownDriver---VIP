"""
Longitudinal velocity profiler and lateral lane-keeping controller.

Velocity  : v_cmd follows v_target with slew limits A_ACCEL / A_DECEL, i.e. the
            graduated safe-stop ramp  v(t) = max(0, v0 - a_decel * (t - t0)).
            EMERGENCY_STOP bypasses the ramp (hard stop).
Steering  : omega = -v_cmd * kappa  -  (Kp e + Ki int(e) + Kd de/dt + Kh psi) * (v_cmd / V_NORMAL)
            kappa = lane curvature (feed-forward, + = bends right)       [1/m]
            e     = lateral offset of the lane centre at the wheel axle  [m]
            psi   = lane heading relative to the robot                    [rad]
            The feed-forward term makes curves free of steady-state error; the
            gain schedule by v_cmd/V_NORMAL keeps the path geometry identical
            while slowing down, so the robot stays centred during the stop and
            never spins in place at v = 0.
"""
from __future__ import annotations

import math

from ..config import FsmConfig, SteeringConfig


class VelocityProfiler:
    def __init__(self, cfg: FsmConfig):
        self.cfg = cfg
        self.v = 0.0

    def update(self, v_target: float, dt: float, hard_stop: bool = False) -> float:
        if hard_stop:
            self.v = 0.0
            return 0.0
        if v_target > self.v:
            self.v = min(v_target, self.v + self.cfg.A_ACCEL * dt)
        else:
            self.v = max(v_target, self.v - self.cfg.A_DECEL * dt)
        if self.v < 1e-4:
            self.v = 0.0
        return self.v


class LaneKeepingController:
    def __init__(self, cfg: SteeringConfig, v_nominal: float):
        self.cfg = cfg
        self.v_nominal = v_nominal
        self.integral = 0.0
        self.prev_e = None
        self.d_filt = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_e = None
        self.d_filt = 0.0

    def compute(self, e_y_m: float, heading_rad: float, v_cmd: float, dt: float, lane_ok: bool = True,
                curvature: float = 0.0) -> float:
        c = self.cfg
        if not lane_ok or v_cmd <= 0.0 or dt <= 0:
            # no steering without motion; bleed the integrator
            self.integral *= 0.9
            self.prev_e = e_y_m if lane_ok else None
            return 0.0
        if self.prev_e is None:
            self.prev_e = e_y_m
        raw_d = (e_y_m - self.prev_e) / dt
        self.d_filt = 0.7 * self.d_filt + 0.3 * raw_d          # derivative low-pass
        self.prev_e = e_y_m

        sched = min(1.5, v_cmd / max(1e-3, self.v_nominal))
        u = c.kp * e_y_m + c.ki * self.integral + c.kd * self.d_filt + c.kh * heading_rad
        omega = -u * sched - v_cmd * curvature * self.cfg.kff
        saturated = abs(omega) >= c.max_omega
        if not saturated:  # conditional integration = anti-windup
            self.integral = max(-c.integral_limit, min(c.integral_limit, self.integral + e_y_m * dt))
        return float(max(-c.max_omega, min(c.max_omega, omega)))


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
