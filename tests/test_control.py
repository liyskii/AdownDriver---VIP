from adowndriver.config import FsmConfig, SteeringConfig
from adowndriver.vehicle.control import LaneKeepingController, VelocityProfiler


def test_controlled_stop_ramp_matches_a_decel():
    cfg = FsmConfig()
    p = VelocityProfiler(cfg)
    p.v = cfg.V_REDUCED
    t = 0.0
    while p.update(0.0, 0.01) > 0:
        t += 0.01
    assert abs(t - cfg.V_REDUCED / cfg.A_DECEL) < 0.03


def test_hard_stop_is_immediate():
    p = VelocityProfiler(FsmConfig())
    p.v = 0.16
    assert p.update(0.16, 0.01, hard_stop=True) == 0.0


def test_steering_sign_and_no_spin_at_zero_speed():
    c = LaneKeepingController(SteeringConfig(), 0.16)
    assert c.compute(0.05, 0.0, 0.16, 0.033) < 0          # lane centre to the right -> turn right (omega < 0)
    assert c.compute(-0.05, 0.0, 0.16, 0.033) > 0
    assert c.compute(0.05, 0.0, 0.0, 0.033) == 0.0
    assert c.compute(0.0, 0.0, 0.16, 0.033, curvature=-1.5) > 0.2   # left bend feed-forward
