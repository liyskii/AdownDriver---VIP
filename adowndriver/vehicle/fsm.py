"""
ALGORITHM 1 (revision 2): Supervisory Safety State Machine.

Pure logic - no I/O, no clocks. Every call to :meth:`SafetyFSM.update` receives
the elapsed time ``dt`` and the freshest inputs; that makes the FSM fully
deterministic and unit-testable (see tests/test_fsm.py).

States
------
STANDBY            robot armed but not moving (power-up state, after RESET)
NORMAL_CRUISE      driver alert, V_NORMAL, lane keeping
WARNING            early fatigue cue, V_NORMAL, intermittent buzzer
SPEED_REDUCTION    sustained fatigue / driver not visible, V_REDUCED
CONTROLLED_STOP    persistent drowsiness: ramp to 0 at A_DECEL *while lane keeping*; latched
EMERGENCY_STOP     hardware/perception fault, obstacle, link loss or manual E-stop: v=0 now

Changes vs. revision 1 of the specification (see docs/SPEC_REVISION.md):
* eye-closure timer measures *continuous* closure (blinks can no longer add up),
  with a 0.2 s open-eye tolerance so one noisy frame does not split an episode;
* yawning requires MAR > TH_MAR for >= 0.8 s, and 3 yawns per minute escalate;
* PERCLOS and head-yaw distraction cues (warning only);
* low C_driver is debounced (0.5 s) and escalates to CONTROLLED_STOP after 5 s;
* lane-loss is debounced (0.3 s) instead of single-frame;
* explicit recovery / reset rules for every state, STANDBY start-up state,
  obstacle slow-down zone and auto-resume after obstacle / link recovery.
"""
from __future__ import annotations

import collections
from dataclasses import dataclass, field
from enum import Enum
from typing import Iterable, Optional

from ..common.filters import PersistenceTimer, WindowFraction
from ..common.protocol import DriverPacket
from ..config import FsmConfig


class State(str, Enum):
    STANDBY = "STANDBY"
    NORMAL_CRUISE = "NORMAL_CRUISE"
    WARNING = "WARNING"
    SPEED_REDUCTION = "SPEED_REDUCTION"
    CONTROLLED_STOP = "CONTROLLED_STOP"
    EMERGENCY_STOP = "EMERGENCY_STOP"


LEVEL_STATE = {0: State.NORMAL_CRUISE, 1: State.WARNING, 2: State.SPEED_REDUCTION, 3: State.CONTROLLED_STOP}

INDICATION = {  # state -> (buzzer pattern, LED colour)
    State.STANDBY: ("OFF", "BLUE"),
    State.NORMAL_CRUISE: ("OFF", "GREEN"),
    State.WARNING: ("INTERMITTENT", "YELLOW"),
    State.SPEED_REDUCTION: ("FAST_PULSE", "ORANGE"),
    State.CONTROLLED_STOP: ("CONTINUOUS", "RED_BLINK"),
    State.EMERGENCY_STOP: ("CONTINUOUS", "RED"),
}

LATCHING_FAULTS = {"LANE_LOST", "TOF_FAULT", "MANUAL_ESTOP"}


@dataclass
class FsmInputs:
    dt: float
    driver: Optional[DriverPacket] = None
    driver_age_s: float = float("inf")     # time since the last driver packet arrived
    c_lane: float = 1.0
    d_obs_cm: Optional[float] = None       # None = invalid reading
    tof_enabled: bool = True
    commands: Iterable[str] = ()


@dataclass
class FsmOutput:
    state: State
    prev_state: State
    v_target: float
    hard_stop: bool                        # True -> bypass the deceleration ramp
    reason: str
    buzzer: str
    led: str
    speed_cap_reason: str = ""
    events: list = field(default_factory=list)

    @property
    def changed(self) -> bool:
        return self.state != self.prev_state


class SafetyFSM:
    def __init__(self, cfg: Optional[FsmConfig] = None):
        self.cfg = cfg or FsmConfig()
        self.reset_all()

    # ------------------------------------------------------------------ #
    def reset_all(self) -> None:
        c = self.cfg
        self.state = State.STANDBY
        self.t = 0.0
        self.reason = "POWER_ON"
        # driver timers
        self.closed_time = 0.0
        self.open_time = 0.0
        self.alert_time = 0.0
        self.yawn_timer = PersistenceTimer()
        self.yawn_counted = False
        self.yawn_events: collections.deque = collections.deque()
        self.perclos = WindowFraction(c.PERCLOS_WINDOW_S)
        self.distract_timer = PersistenceTimer()
        self.low_conf_timer = PersistenceTimer()
        self.eyes_closed = False
        self.driver_valid = False
        self.driver_level = 0
        self.driver_reason = "NONE"
        self.latched_level = 0
        self.latched_reason = "NONE"
        # vehicle faults
        self.lane_low_timer = PersistenceTimer()
        self.tof_bad_timer = PersistenceTimer()
        self.obstacle_clear_timer = PersistenceTimer()
        self.link_ok_timer = PersistenceTimer()
        self.faults: dict[str, float] = {}   # fault -> time raised
        self.link_ok = False
        self.ready_issues: list[str] = []

    # ------------------------------------------------------------------ #
    def snapshot(self) -> dict:
        """Internal timers for the dashboard / logs."""
        return {
            "t_closed": round(self.closed_time, 3),
            "t_alert": round(self.alert_time, 3),
            "t_low_conf": round(self.low_conf_timer.elapsed, 3),
            "t_distract": round(self.distract_timer.elapsed, 3),
            "perclos": round(self.perclos.value, 3),
            "yawns_window": len(self.yawn_events),
            "eyes_closed": self.eyes_closed,
            "driver_valid": self.driver_valid,
            "driver_level": self.driver_level,
            "latched_level": self.latched_level,
            "faults": sorted(self.faults),
            "ready_issues": list(self.ready_issues),
            "link_ok": self.link_ok,
        }

    # ------------------------------------------------------------------ #
    def update(self, inp: FsmInputs) -> FsmOutput:
        c = self.cfg
        dt = max(0.0, min(inp.dt, 0.5))
        self.t += dt
        prev = self.state
        events: list = []
        cmds = set(inp.commands or ())

        self._update_driver(inp, dt, events)
        self._update_faults(inp, dt, events, driving=self.state not in (State.STANDBY,))
        self._update_ready(inp)

        # ---------------- commands ---------------------------------------
        if "estop" in cmds:
            if "MANUAL_ESTOP" not in self.faults:
                self.faults["MANUAL_ESTOP"] = self.t
                events.append(("FAULT", "MANUAL_ESTOP"))
            self.state = State.EMERGENCY_STOP
        if "stop" in cmds and self.state not in (State.EMERGENCY_STOP,):
            self.state = State.STANDBY
            self.reason = "OPERATOR_STOP"
        if ("reset" in cmds or "ack" in cmds) and self.state in (State.EMERGENCY_STOP, State.CONTROLLED_STOP,
                                                                   State.STANDBY):
            self._try_reset(inp, events, operator="reset" in cmds)
        if "start" in cmds and self.state == State.STANDBY:
            if not self.ready_issues:
                self.state = LEVEL_STATE[self.latched_level] if self.latched_level < 3 else State.CONTROLLED_STOP
                self.reason = "OPERATOR_START"
            else:
                events.append(("START_REJECTED", ",".join(self.ready_issues)))

        # ---------------- state machine ------------------------------------
        if self.state == State.STANDBY:
            pass
        elif self.faults:
            self.state = State.EMERGENCY_STOP
            self.reason = self._fault_reason()
        elif self.state == State.EMERGENCY_STOP:
            # all faults cleared automatically -> resume according to driver level
            self.state = LEVEL_STATE[self.latched_level]
            self.reason = "FAULT_CLEARED"
        elif self.state == State.CONTROLLED_STOP:
            self.reason = self.latched_reason          # latched until reset/ack
        else:
            self.state = LEVEL_STATE[self.latched_level]
            self.reason = self.latched_reason if self.latched_level else "NONE"

        # ---------------- outputs ----------------------------------------
        hard = self.state == State.EMERGENCY_STOP
        v_target = {
            State.STANDBY: 0.0,
            State.NORMAL_CRUISE: c.V_NORMAL,
            State.WARNING: c.V_NORMAL,
            State.SPEED_REDUCTION: c.V_REDUCED,
            State.CONTROLLED_STOP: 0.0,
            State.EMERGENCY_STOP: 0.0,
        }[self.state]

        cap_reason = ""
        d = inp.d_obs_cm
        if v_target > 0 and inp.tof_enabled and d is not None and d < c.D_SLOW_CM:
            frac = max(0.0, min(1.0, (d - c.D_SAFE_CM) / max(1e-3, c.D_SLOW_CM - c.D_SAFE_CM)))
            cap = c.V_CREEP + (c.V_NORMAL - c.V_CREEP) * frac
            if cap < v_target:
                v_target, cap_reason = cap, "OBSTACLE_SLOW_ZONE"

        buzzer, led = INDICATION[self.state]
        if self.state == State.STANDBY and self.ready_issues:
            led = "BLUE_BLINK"
        if self.state != prev:
            events.append(("TRANSITION", f"{prev.value}->{self.state.value}", self.reason))
        return FsmOutput(self.state, prev, v_target, hard, self.reason, buzzer, led, cap_reason, events)

    # ------------------------------------------------------------------ #
    def _update_driver(self, inp: FsmInputs, dt: float, events: list) -> None:
        c = self.cfg
        drv = inp.driver
        link_ok = drv is not None and inp.driver_age_s <= c.HEARTBEAT_TIMEOUT
        self.link_ok = link_ok
        if not link_ok:
            # link loss is handled as a vehicle fault; freeze driver timers
            self.driver_valid = False
            return

        conf_ok = drv.face and drv.c_driver >= c.TH_CONF_MIN
        self.driver_valid = conf_ok
        self.low_conf_timer.update(not conf_ok, dt)

        distracted = conf_ok and abs(drv.yaw_deg) > c.YAW_DISTRACT_DEG
        self.distract_timer.update(distracted, dt)

        # Eye closure: only trust EAR when the face is visible and roughly frontal
        if conf_ok and not distracted:
            self.eyes_closed = drv.ear < drv.ear_th
            if self.eyes_closed:
                self.closed_time += dt
                self.open_time = 0.0
            else:
                self.open_time += dt
                if self.open_time >= c.T_OPEN_TOLERANCE:
                    self.closed_time = 0.0
            self.perclos.update(self.eyes_closed, dt)
        else:
            self.eyes_closed = False

        # Yawn: MAR above threshold for at least T_YAWN_MIN counts as one yawn
        yawning = conf_ok and drv.mar > c.TH_MAR
        self.yawn_timer.update(yawning, dt)
        if yawning and self.yawn_timer.elapsed >= c.T_YAWN_MIN and not self.yawn_counted:
            self.yawn_counted = True
            self.yawn_events.append(self.t)
            events.append(("YAWN", f"count={len(self.yawn_events)}"))
        if not yawning:
            self.yawn_counted = False
        while self.yawn_events and self.t - self.yawn_events[0] > c.YAWN_WINDOW_S:
            self.yawn_events.popleft()

        # a normal blink (< T_BLINK_MAX) does not interrupt the recovery timer
        alert = conf_ok and self.closed_time < c.T_BLINK_MAX and not yawning and not distracted
        self.alert_time = self.alert_time + dt if alert else 0.0

        # ---- instantaneous driver level --------------------------------
        cands = [(0, "NONE")]
        if self.closed_time >= c.T_STOP:
            cands.append((3, "PERSISTENT_DROWSINESS"))
        if self.low_conf_timer.elapsed >= c.T_DRIVER_LOST:
            cands.append((3, "DRIVER_NOT_MONITORABLE"))
        if self.closed_time >= c.T_SLOW:
            cands.append((2, "SUSTAINED_EYE_CLOSURE"))
        if len(self.yawn_events) >= c.YAWN_COUNT_SLOW:
            cands.append((2, "REPEATED_YAWNING"))
            self.yawn_events.clear()           # consumed; escalation stays latched
        if self.low_conf_timer.elapsed >= c.T_CONF_DEBOUNCE:
            cands.append((2, "LOW_DRIVER_CONFIDENCE"))
        if self.closed_time >= c.T_WARN:
            cands.append((1, "EYE_CLOSURE"))
        if yawning and self.yawn_counted:
            cands.append((1, "YAWN"))
        if self.perclos.value >= c.PERCLOS_WARN:
            cands.append((1, "HIGH_PERCLOS"))
        if self.distract_timer.elapsed >= c.T_DISTRACT:
            cands.append((1, "DRIVER_DISTRACTED"))
        level, reason = max(cands, key=lambda x: x[0])
        self.driver_level, self.driver_reason = level, reason

        # ---- latch with hysteresis ------------------------------------
        # Parked (STANDBY): nothing to stop, so no latch - the live driver level is what
        # blocks Start (DRIVER_NOT_VISIBLE etc.). Prevents a stale latch that Reset can't clear.
        if self.state == State.STANDBY:
            if self.latched_level and level == 0:
                events.append(("RECOVERED", f"from_level={self.latched_level}"))
            self.latched_level, self.latched_reason = level, (reason if level else "NONE")
            return
        if level >= self.latched_level and level > 0:
            if level > self.latched_level or self.latched_reason == "NONE":
                self.latched_reason = reason
            self.latched_level = level
        elif self.latched_level in (1, 2) and level == 0 and self.alert_time >= c.T_RECOVER:
            events.append(("RECOVERED", f"from_level={self.latched_level}"))
            self.latched_level, self.latched_reason = 0, "NONE"

    def _update_faults(self, inp: FsmInputs, dt: float, events: list, driving: bool) -> None:
        c = self.cfg

        def raise_(name):
            if name not in self.faults:
                self.faults[name] = self.t
                events.append(("FAULT", name))

        def clear(name):
            if name in self.faults:
                del self.faults[name]
                events.append(("FAULT_CLEARED", name))

        # heartbeat / link
        self.link_ok_timer.update(self.link_ok, dt)
        if driving and not self.link_ok:
            raise_("HEARTBEAT_LOSS")
        elif "HEARTBEAT_LOSS" in self.faults and c.auto_resume_link and self.link_ok_timer.elapsed >= c.T_LINK_RESTORE:
            clear("HEARTBEAT_LOSS")

        # lane confidence (debounced)
        self.lane_low_timer.update(inp.c_lane < c.TH_CONF_MIN, dt)
        if driving and self.lane_low_timer.elapsed >= c.T_LANE_LOSS:
            raise_("LANE_LOST")

        # obstacle / ToF
        if inp.tof_enabled:
            d = inp.d_obs_cm
            self.tof_bad_timer.update(d is None, dt)
            if driving and self.tof_bad_timer.elapsed >= c.T_TOF_FAULT:
                raise_("TOF_FAULT")
            if d is not None and d < c.D_SAFE_CM and driving:
                raise_("OBSTACLE")
            clear_ok = d is not None and d >= c.D_SAFE_CM + c.D_CLEAR_HYST_CM
            self.obstacle_clear_timer.update(clear_ok, dt)
            if "OBSTACLE" in self.faults and c.auto_resume_obstacle and \
                    self.obstacle_clear_timer.elapsed >= c.T_OBSTACLE_CLEAR:
                clear("OBSTACLE")

    def _update_ready(self, inp: FsmInputs) -> None:
        c = self.cfg
        issues = []
        if not self.link_ok:
            issues.append("NO_DRIVER_LINK")
        elif c.require_driver_for_start and not self.driver_valid:
            issues.append("DRIVER_NOT_VISIBLE")
        if inp.c_lane < c.TH_CONF_MIN:
            issues.append("LANE_NOT_VISIBLE")
        if inp.tof_enabled and (inp.d_obs_cm is None or inp.d_obs_cm < c.D_SAFE_CM + c.D_CLEAR_HYST_CM):
            issues.append("PATH_BLOCKED" if inp.d_obs_cm is not None else "TOF_INVALID")
        if self.faults:
            issues.append("ACTIVE_FAULTS")
        if self.latched_level >= 3:
            issues.append("DRIVER_STOP_LATCHED")
        self.ready_issues = issues

    def _try_reset(self, inp: FsmInputs, events: list, operator: bool) -> None:
        c = self.cfg
        # Clear latching faults whose cause is gone
        for f in list(self.faults):
            gone = {
                "MANUAL_ESTOP": operator,
                "LANE_LOST": inp.c_lane >= c.TH_CONF_MIN,
                "TOF_FAULT": inp.d_obs_cm is not None,
                "OBSTACLE": inp.d_obs_cm is not None and inp.d_obs_cm >= c.D_SAFE_CM + c.D_CLEAR_HYST_CM,
                "HEARTBEAT_LOSS": self.link_ok,
            }.get(f, False)
            if gone:
                del self.faults[f]
                events.append(("FAULT_CLEARED", f))
        # The drowsiness latch only releases when the driver is demonstrably alert
        # (a high PERCLOS alone does not block the reset - it is history, cleared below)
        if self.latched_level > 0 and self.driver_reason in ("NONE", "HIGH_PERCLOS") and \
                self.alert_time >= c.T_RECOVER:
            self.latched_level, self.latched_reason = 0, "NONE"
            self.closed_time = 0.0
            self.perclos.reset()
        if not self.faults and self.latched_level < 3:
            self.lane_low_timer.reset()
            self.tof_bad_timer.reset()
            self.state = State.STANDBY
            self.reason = "RESET"
            events.append(("RESET", "operator" if operator else "driver_ack"))
        else:
            events.append(("RESET_REJECTED", ",".join(sorted(self.faults)) or "DRIVER_NOT_ALERT"))

    def _fault_reason(self) -> str:
        order = ["MANUAL_ESTOP", "OBSTACLE", "HEARTBEAT_LOSS", "TOF_FAULT", "LANE_LOST"]
        for f in order:
            if f in self.faults:
                return f
        return next(iter(self.faults))
