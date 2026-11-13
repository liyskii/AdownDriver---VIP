"""Unit tests of the supervisory safety FSM (pure logic, deterministic)."""
import pytest

from adowndriver.common.protocol import DriverPacket
from adowndriver.config import FsmConfig
from adowndriver.vehicle.fsm import FsmInputs, SafetyFSM, State

DT = 1 / 30


def drv(ear=0.30, mar=0.10, c=0.95, face=True, yaw=0.0, th=0.21):
    return DriverPacket(face=face, ear=ear, mar=mar, c_driver=c, yaw_deg=yaw, ear_th=th)


def step(fsm, seconds, driver=None, c_lane=0.95, d=100.0, age=0.01, cmds=(), tof=True):
    out = None
    n = max(1, round(seconds / DT))
    for i in range(n):
        out = fsm.update(FsmInputs(DT, driver if driver is not None else drv(), age, c_lane, d, tof,
                                   cmds if i == 0 else ()))
    return out


@pytest.fixture
def fsm():
    f = SafetyFSM(FsmConfig())
    step(f, 0.1)
    out = step(f, DT, cmds=("start",))
    assert out.state == State.NORMAL_CRUISE
    return f


def test_powers_up_in_standby():
    f = SafetyFSM()
    assert step(f, 0.5).state == State.STANDBY


def test_start_rejected_without_driver_link():
    f = SafetyFSM()
    out = step(f, DT, age=float("inf"), cmds=("start",))
    assert out.state == State.STANDBY
    assert "NO_DRIVER_LINK" in f.ready_issues


def test_start_rejected_when_path_blocked():
    f = SafetyFSM()
    assert step(f, DT, d=15.0, cmds=("start",)).state == State.STANDBY


@pytest.mark.parametrize("blink", [0.1, 0.2, 0.3, 0.39])
def test_blinks_never_escalate(fsm, blink):
    for _ in range(15):
        step(fsm, blink, drv(ear=0.12))
        assert step(fsm, 2.5).state == State.NORMAL_CRUISE


def test_repeated_blinks_do_not_accumulate(fsm):
    # revision-1 bug: t_drowsy summed blinks until 1.5 s of open eyes - this must stay NORMAL
    states = set()
    for _ in range(30):
        states.add(step(fsm, 0.3, drv(ear=0.12)).state)
        states.add(step(fsm, 1.7).state)
    assert states == {State.NORMAL_CRUISE}


def test_graduated_escalation_timing(fsm):
    seen = {}
    t = 0.0
    while t < 4.5:
        out = step(fsm, DT, drv(ear=0.12))
        t += DT
        seen.setdefault(out.state, t)
    assert seen[State.WARNING] == pytest.approx(1.2, abs=0.05)
    assert seen[State.SPEED_REDUCTION] == pytest.approx(2.5, abs=0.05)
    assert seen[State.CONTROLLED_STOP] == pytest.approx(4.0, abs=0.05)


def test_noisy_frame_inside_closure_does_not_split_episode(fsm):
    step(fsm, 1.0, drv(ear=0.12))
    step(fsm, 0.1, drv(ear=0.25))       # one noisy "open" reading shorter than T_OPEN_TOLERANCE
    assert step(fsm, 0.3, drv(ear=0.12)).state == State.WARNING


def test_recovery_requires_continuous_alertness(fsm):
    step(fsm, 1.5, drv(ear=0.12))
    assert fsm.state == State.WARNING
    assert step(fsm, 1.0).state == State.WARNING
    assert step(fsm, 0.8).state == State.NORMAL_CRUISE


def test_controlled_stop_is_latched_until_reset(fsm):
    step(fsm, 4.2, drv(ear=0.12))
    assert step(fsm, 5.0).state == State.CONTROLLED_STOP
    out = step(fsm, DT, cmds=("reset",))
    assert out.state == State.STANDBY
    assert step(fsm, DT, cmds=("start",)).state == State.NORMAL_CRUISE


def test_reset_rejected_while_driver_still_asleep(fsm):
    step(fsm, 4.2, drv(ear=0.12))
    out = step(fsm, DT, drv(ear=0.12), cmds=("reset",))
    assert out.state == State.CONTROLLED_STOP


def test_single_yawn_warns_talking_does_not(fsm):
    for _ in range(5):   # talking: MAR spikes shorter than T_YAWN_MIN
        assert step(fsm, 0.5, drv(mar=0.7)).state == State.NORMAL_CRUISE
        step(fsm, 0.5, drv(mar=0.3))
    assert step(fsm, 1.0, drv(mar=0.75)).state == State.WARNING


def test_three_yawns_slow_down(fsm):
    for _ in range(2):
        step(fsm, 1.5, drv(mar=0.8))
        step(fsm, 3.0)
    out = step(fsm, 1.0, drv(mar=0.8))
    assert out.state == State.SPEED_REDUCTION and out.reason == "REPEATED_YAWNING"


def test_low_driver_confidence_debounced_then_stop(fsm):
    assert step(fsm, 0.3, drv(face=False, c=0.0)).state == State.NORMAL_CRUISE
    assert step(fsm, 0.4, drv(face=False, c=0.0)).state == State.SPEED_REDUCTION
    assert step(fsm, 4.5, drv(face=False, c=0.0)).state == State.CONTROLLED_STOP
    assert fsm.reason == "DRIVER_NOT_MONITORABLE"


def test_distraction_warns_only(fsm):
    out = step(fsm, 2.5, drv(yaw=45))
    assert out.state == State.WARNING and out.reason == "DRIVER_DISTRACTED"


def test_closed_eyes_ignored_while_head_turned(fsm):
    step(fsm, 3.0, drv(ear=0.12, yaw=50))
    assert fsm.closed_time == 0.0


def test_obstacle_emergency_and_auto_resume(fsm):
    out = step(fsm, DT, d=15.0)
    assert out.state == State.EMERGENCY_STOP and out.hard_stop and out.reason == "OBSTACLE"
    assert step(fsm, 0.5, d=30.0).state == State.EMERGENCY_STOP
    assert step(fsm, 0.6, d=30.0).state == State.NORMAL_CRUISE


def test_slow_zone_caps_speed(fsm):
    out = step(fsm, DT, d=30.0)
    assert out.state == State.NORMAL_CRUISE
    assert 0.03 < out.v_target < 0.16 and out.speed_cap_reason == "OBSTACLE_SLOW_ZONE"


def test_heartbeat_timeout(fsm):
    assert step(fsm, DT, age=0.14).state == State.NORMAL_CRUISE
    out = step(fsm, DT, age=0.16)
    assert out.state == State.EMERGENCY_STOP and out.reason == "HEARTBEAT_LOSS"
    assert step(fsm, 1.1).state == State.NORMAL_CRUISE


def test_lane_loss_debounced_and_latched(fsm):
    assert step(fsm, 0.2, c_lane=0.3).state == State.NORMAL_CRUISE
    assert step(fsm, 0.2, c_lane=0.3).state == State.EMERGENCY_STOP
    assert step(fsm, 2.0, c_lane=0.95).state == State.EMERGENCY_STOP   # latched
    assert step(fsm, DT, c_lane=0.95, cmds=("reset",)).state == State.STANDBY


def test_tof_fault(fsm):
    out = step(fsm, 0.3, d=None)
    assert out.state == State.EMERGENCY_STOP and out.reason == "TOF_FAULT"


def test_tof_disabled_never_faults(fsm):
    assert step(fsm, 1.0, d=None, tof=False).state == State.NORMAL_CRUISE


def test_manual_estop_needs_reset(fsm):
    assert step(fsm, DT, cmds=("estop",)).state == State.EMERGENCY_STOP
    assert step(fsm, 2.0).state == State.EMERGENCY_STOP
    assert step(fsm, DT, cmds=("reset",)).state == State.STANDBY


def test_emergency_has_priority_over_drowsiness(fsm):
    step(fsm, 3.0, drv(ear=0.12))
    assert step(fsm, DT, drv(ear=0.12), d=10.0).state == State.EMERGENCY_STOP


def test_operator_stop_soft(fsm):
    out = step(fsm, DT, cmds=("stop",))
    assert out.state == State.STANDBY and not out.hard_stop
