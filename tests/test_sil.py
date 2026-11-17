"""End-to-end software-in-the-loop tests: the real VehicleCore driven by the simulator.

Runs every scenario of the 16-scenario validation matrix (same checks as
``python -m adowndriver validate``) and probes the eye-closure ladder
(T_WARN 1.2 s -> T_SLOW 2.5 s -> T_STOP 4.0 s) just below and above each step.
"""
import pytest

from adowndriver.sim.runner import run_scenario
from adowndriver.sim.scenarios import BY_ID, SCENARIOS, Scenario
from adowndriver.sim.synthetic_driver import Segment

DRIVER_STATES = ("WARNING", "SPEED_REDUCTION", "CONTROLLED_STOP")


def test_matrix_has_16_unique_scenarios():
    ids = [s.id for s in SCENARIOS]
    assert ids == [f"S{i:02d}" for i in range(1, 17)]
    assert len(BY_ID) == 16


@pytest.mark.parametrize("sid", [s.id for s in SCENARIOS])
def test_scenario(sid):
    r = run_scenario(BY_ID[sid])
    assert r.passed, [c for c in r.checks if not c[1]]


def _closure(seconds: float, tail: float = 12.0) -> Scenario:
    """Alert driver, eyes closed once for ``seconds`` starting at t = 8 s, then alert again."""
    t1 = 8.0 + seconds
    return Scenario(f"closure_{seconds}", f"Eyes closed {seconds} s", t1 + tail,
                    [Segment(0, 8, "alert", "alert"), Segment(8, t1, "closed", "drowsy"),
                     Segment(t1, t1 + tail, "alert", "alert")])


@pytest.mark.parametrize("seconds,highest", [
    # below T_WARN: no reaction at all. Kept well under 1.2 s because the synthetic
    # driver may fire a pending natural blink (<= 0.3 s) the moment the eyes reopen.
    (0.8, "NORMAL_CRUISE"),
    (1.8, "WARNING"),            # between T_WARN and T_SLOW
    (3.2, "SPEED_REDUCTION"),    # between T_SLOW and T_STOP
    (4.5, "CONTROLLED_STOP"),    # past T_STOP: full staged stop
])
def test_eye_closure_ladder(seconds, highest):
    r = run_scenario(_closure(seconds))
    seen = [s for s in DRIVER_STATES if s in r.states_seen]
    assert (seen[-1] if seen else "NORMAL_CRUISE") == highest, r.states_seen
    assert "EMERGENCY_STOP" not in r.states_seen
    if highest == "CONTROLLED_STOP":
        # a controlled stop latches: robot stays stopped until an operator presses Reset
        assert r.final_state == "CONTROLLED_STOP"
        assert r.trace[-1]["v"] == 0.0
        # and it climbed the ladder in order, never skipping a step
        order = [s for s in r.states_seen if s in DRIVER_STATES]
        assert order == list(DRIVER_STATES)
    else:
        # shorter closures de-escalate once the driver is alert again
        assert r.final_state == "NORMAL_CRUISE"
