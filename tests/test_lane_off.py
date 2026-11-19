import numpy as np

from adowndriver.config import load_config
from adowndriver.vehicle.core import StepInput, VehicleCore


def _run(lane_keeping: bool):
    cfg = load_config()
    cfg.vehicle.lane_keeping = lane_keeping
    core = VehicleCore(cfg)
    blank = np.zeros((cfg.camera.height, cfg.camera.width, 3), np.uint8)   # no tape visible at all
    for i in range(10):
        core.step(StepInput(dt=1 / 30, t_ms=i * 33, frame=blank, d_obs_cm=150.0))
    return core.last_lane


def test_lane_keeping_off_reports_full_confidence_and_straight_steering():
    lane = _run(False)
    assert lane.c_lane == 1.0 and lane.mode == "OFF" and lane.e_axle_m == 0.0


def test_lane_keeping_on_sees_no_lane_on_blank_frame():
    assert _run(True).c_lane < 0.65
