import math

import numpy as np
import pytest

from adowndriver.config import AppConfig
from adowndriver.sim.world import SimWorld
from adowndriver.vehicle.lane import LaneDetector


@pytest.fixture(scope="module")
def setup():
    cfg = AppConfig()
    return cfg, SimWorld(cfg.camera, cfg.lane)


def place(world, s, offset=0.0, dth=0.0):
    x, y, th = world.track.pose_at(s)
    world.robot.x, world.robot.y, world.robot.th = x - math.sin(th) * offset, y + math.cos(th) * offset, th + dth


@pytest.mark.parametrize("offset", [-0.06, -0.03, 0.0, 0.03, 0.06])
def test_lateral_offset_on_straight(setup, offset):
    cfg, w = setup
    place(w, 0.8, offset)
    r = LaneDetector(cfg.lane, cfg.camera).process(w.render())
    assert r.mode == "BOTH" and r.c_lane > 0.9
    assert r.e_axle_m == pytest.approx(offset, abs=0.012)   # robot left of centre -> lane centre to the right (+)


def test_lane_width_estimate(setup):
    cfg, w = setup
    place(w, 1.0)
    r = LaneDetector(cfg.lane, cfg.camera).process(w.render())
    assert r.width_m == pytest.approx(cfg.lane.lane_width_m, abs=0.02)


def test_curve_feedforward_sign(setup):
    cfg, w = setup
    place(w, 2.8)   # left-hand (CCW) arc of radius 0.65 m
    r = LaneDetector(cfg.lane, cfg.camera).process(w.render())
    assert r.c_lane >= 0.65
    assert r.curvature == pytest.approx(-1 / 0.65, rel=0.2)


def test_no_tape_gives_zero_confidence(setup):
    cfg, _ = setup
    black = np.full((240, 320, 3), 25, np.uint8)
    assert LaneDetector(cfg.lane, cfg.camera).process(black).c_lane == 0.0


def test_dim_light_still_detected(setup):
    cfg, w = setup
    place(w, 0.8)
    w.brightness = 0.45
    r = LaneDetector(cfg.lane, cfg.camera).process(w.render())
    w.brightness = 1.0
    assert r.c_lane > 0.8


def test_glare_blob_suppressed(setup):
    cfg, w = setup
    place(w, 0.8)
    w.glare = (210, 150, 22)
    r = LaneDetector(cfg.lane, cfg.camera).process(w.render())
    w.glare = None
    assert r.mode == "BOTH" and abs(r.e_axle_m) < 0.015
