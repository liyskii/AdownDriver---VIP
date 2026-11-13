"""A few end-to-end software-in-the-loop scenarios (full set: python -m adowndriver validate)."""
import pytest

from adowndriver.sim.runner import run_scenario
from adowndriver.sim.scenarios import BY_ID


@pytest.mark.parametrize("sid", ["S02", "S08", "S14", "S15"])
def test_scenario(sid):
    r = run_scenario(BY_ID[sid])
    assert r.passed, [c for c in r.checks if not c[1]]
