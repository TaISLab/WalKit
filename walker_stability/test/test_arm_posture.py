import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from walker_stability import arm_posture as ap  # noqa: E402


def test_the_distance_is_positive_when_the_user_is_behind_the_handles():
    assert ap.handle_distance(-0.405, -0.75) == pytest.approx(0.345)
    assert ap.handle_distance(-0.405, -0.3) == pytest.approx(-0.105)          # leaning over the handles


def test_the_reach_is_the_distance_minus_one_constant():
    assert ap.reach(0.345) == pytest.approx(0.345 - ap.ARM_REACH_OFFSET_M)
    assert ap.reach(0.2, offset=0.1) == pytest.approx(0.1)
    assert ap.reach(ap.ARM_REACH_OFFSET_M) == pytest.approx(0.0)
