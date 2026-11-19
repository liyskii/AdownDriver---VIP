import math

import numpy as np

from adowndriver.vehicle.io import front_distance_cm

INC = 2 * math.pi / 400


def scan(default=2.0):
    return np.full(400, default, dtype=np.float32)


def test_clear_path_returns_distance_minus_offset():
    assert abs(front_distance_cm(scan(2.0), 0.0, INC, 20, 7, 300) - 193.0) < 0.5


def test_obstacle_ahead_wraps_around_zero_angle():
    r = scan(2.0)
    r[-3:] = 0.25            # just left of 0 rad (angles near 2*pi)
    r[:3] = 0.25             # just right of 0 rad
    assert abs(front_distance_cm(r, 0.0, INC, 20, 7, 300) - 18.0) < 0.5


def test_obstacle_behind_is_ignored():
    r = scan(2.0)
    r[195:205] = 0.15        # ~180 deg
    assert front_distance_cm(r, 0.0, INC, 20, 7, 300) > 150


def test_zero_returns_mean_no_echo_and_single_noise_beam_ignored():
    r = scan(0.0)            # nothing reflects -> clear path
    assert front_distance_cm(r, 0.0, INC, 20, 7, 300) == 300
    r = scan(2.0)
    r[0] = 0.12              # one noisy beam must not stop the robot
    assert front_distance_cm(r, 0.0, INC, 20, 7, 300) > 150
