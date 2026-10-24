import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from walker_loads import gait_quality as gq  # noqa: E402
from walker_loads.gait_quality import LegStance, Onset  # noqa: E402


def walk(n=12, step_s=0.55, width=0.15, double=0.1, left_step_s=None):
    """A regular walk: the legs alternate every step_s; a leg is in stance for step_s + double."""
    left_step_s = left_step_s or step_s
    left, right, sl, sr = [], [], [], []
    t = 0.0
    for i in range(n):
        if i % 2 == 0:
            left.append(Onset(t, width / 2)); sl.append((t, t + 2 * step_s + double))
            t += left_step_s
        else:
            right.append(Onset(t, -width / 2)); sr.append((t, t + 2 * step_s + double))
            t += step_s
    return left, right, sl, sr


def test_a_regular_walk_has_the_step_time_and_width_that_went_in_and_no_asymmetry():
    left, right, sl, sr = walk(12, 0.55, 0.15)
    q = gq.quality(left, right, sl, sr, 0.0, 6.6)
    assert q.step_time == pytest.approx(0.55) and q.step_width == pytest.approx(0.15)
    assert q.step_time_si == pytest.approx(0.0, abs=1e-9) and q.step_time_cv == pytest.approx(0.0, abs=1e-6)
    assert q.steps == 12 and q.window_s == pytest.approx(6.6)


def test_the_symmetry_index_sees_one_leg_taking_longer_steps():
    left, right, sl, sr = walk(16, 0.5, left_step_s=0.6)
    q = gq.quality(left, right, sl, sr, 0.0, 9.0)
    # left onsets follow the right's after 0.5 s, right's follow the left's after 0.6 s: steps of 0.5 (left) and 0.6 (right)
    assert q.step_time_si == pytest.approx(abs(0.5 - 0.6) / 0.55 * 100, rel=1e-6)
    assert q.step_time_cv > 0


def test_double_support_is_the_overlap_over_the_window():
    assert gq.double_support([(0.0, 2.0)], [(1.0, 3.0)], 0.0, 4.0) == pytest.approx(0.25)
    assert gq.double_support([(0.0, 2.0)], [(1.0, 3.0)], 1.0, 2.0) == pytest.approx(1.0)
    assert gq.double_support([(0.0, None)], [(1.0, None)], 0.0, 4.0) == pytest.approx(0.75)      # both still down
    assert gq.double_support([(0.0, 1.0)], [(2.0, 3.0)], 0.0, 4.0) == 0.0
    assert math.isnan(gq.double_support([], [], 1.0, 1.0))


def test_only_the_window_counts():
    left, right, sl, sr = walk(20, 0.5)
    whole = gq.quality(left, right, sl, sr, 0.0, 10.0)
    last = gq.quality(left, right, sl, sr, 5.0, 10.0)
    assert last.steps < whole.steps and last.window_s == pytest.approx(5.0)


def test_nothing_is_invented_when_there_is_too_little():
    q = gq.quality([Onset(0.0, 0.1)], [], [(0.0, None)], [], 0.0, 1.0)
    assert q.steps == 1 and math.isnan(q.step_time) and math.isnan(q.step_width) and math.isnan(q.step_time_si) and math.isnan(q.step_time_cv)


def test_steps_that_are_too_short_or_too_far_apart_are_not_steps():
    left = [Onset(0.0, 0.1), Onset(5.0, 0.1)]
    right = [Onset(0.1, -0.1), Onset(2.6, -0.1)]            # 0.1 s after the left (too short), then 2.5 s after it (too long)
    tl, tr = gq.step_times(left, right)
    assert tl == [] and tr == []


def test_a_leg_registers_an_onset_and_its_interval_and_a_flicker_is_the_same_stance():
    leg = LegStance(min_interval_s=0.35)
    series = [(0.00, 0.0), (0.10, 0.4), (0.15, 0.0), (0.20, 0.6), (0.30, 20.0), (0.90, 0.0), (1.00, 0.0), (1.50, 10.0), (2.20, 0.0)]
    onsets = [t for t, load in series if leg.update(load, t, 0.1)]
    assert onsets == [0.10, 1.50]
    assert leg.stance() == [(0.10, 0.90), (1.50, 2.20)]                       # the flicker did not cut the first stance
    assert [o.t for o in leg.onsets] == [0.10, 1.50]


def test_a_stance_still_open_has_no_end():
    leg = LegStance()
    leg.update(5.0, 1.0, 0.0)
    assert leg.stance() == [(1.0, None)]
