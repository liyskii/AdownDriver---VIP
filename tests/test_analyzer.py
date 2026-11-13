import numpy as np
import pytest

from adowndriver.config import DriverMonitorConfig
from adowndriver.driver_monitor.analyzer import (Calibrator, FaceObservation, compute_ear, compute_mar,
                                                 driver_confidence)


def eye_points(opening):
    pts = [(0, 0)] * 478
    # p1(33) left corner, p4(133) right corner, upper 160/158, lower 144/153
    pts = list(pts)
    pts[33], pts[133] = (0, 0), (30, 0)
    pts[160], pts[158] = (10, -opening), (20, -opening)
    pts[144], pts[153] = (10, opening), (20, opening)
    return pts


@pytest.mark.parametrize("opening,expected", [(4.5, 0.30), (1.5, 0.10)])
def test_ear(opening, expected):
    assert compute_ear(eye_points(opening)) == pytest.approx(expected, abs=1e-6)


def test_mar_closed_mouth_is_zero():
    pts = [(0.0, 0.0)] * 478
    pts = list(pts)
    pts[78], pts[308] = (0, 0), (40, 0)
    assert compute_mar(pts) == 0.0


def test_confidence_factors():
    cfg = DriverMonitorConfig()
    good = FaceObservation(True, face_width_px=160, luminance=120)
    assert driver_confidence(good, cfg) == 1.0
    assert driver_confidence(FaceObservation(False), cfg) == 0.0
    assert driver_confidence(FaceObservation(True, face_width_px=50, luminance=120), cfg) < 0.65
    assert driver_confidence(FaceObservation(True, face_width_px=160, luminance=15), cfg) < 0.65
    turned = FaceObservation(True, face_width_px=160, luminance=120, yaw_deg=45)
    assert 0.65 <= driver_confidence(turned, cfg) < 1.0     # distraction is a warning, not "driver lost"


def test_calibration_threshold():
    cfg = DriverMonitorConfig(calibration_seconds=0.0)
    cal = Calibrator(cfg, "t")
    rng = np.random.default_rng(0)
    for i in range(100):
        ear = 0.10 if i % 25 == 0 else 0.32 + rng.normal(0, 0.005)  # a few blinks in the sample
        cal.add(FaceObservation(True, ear_raw=ear, face_width_px=160, luminance=120), 0.95)
    prof = cal.result()
    assert prof.ear_open == pytest.approx(0.32, abs=0.01)
    assert prof.ear_th == pytest.approx(0.72 * 0.32, abs=0.01)
