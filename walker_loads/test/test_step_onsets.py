import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from walker_loads.step_onsets import StepOnsets  # noqa: E402

MS = 1_000_000


def feed(onsets, series):
    """series [(t_ms, load)] -> the t_ms at which a step was registered."""
    return [t for t, load in series if onsets.update(load, t * MS)]


def test_a_clean_load_is_one_step_per_rise():
    assert feed(StepOnsets(), [(0, 0.0), (50, 3.0), (100, 20.0), (900, 0.0), (1000, 0.0), (1100, 15.0), (1200, 30.0)]) == [50, 1100]


def test_a_flicker_inside_the_refractory_period_is_one_step():
    flicker = [(0, 0.0), (50, 0.4), (100, 0.0), (150, 0.6), (200, 0.0), (250, 2.0), (300, 18.0), (900, 0.0)]
    onsets = StepOnsets(350 * MS)
    assert feed(onsets, flicker) == [50]
    assert onsets.ignored == 2


def test_without_a_refractory_every_rise_counts():
    flicker = [(0, 0.0), (50, 0.4), (100, 0.0), (150, 0.6), (200, 0.0), (250, 2.0)]
    assert feed(StepOnsets(0), flicker) == [50, 150, 250]


def test_the_period_runs_from_the_registered_step_not_from_the_last_transition():
    # rises at 0, 200, 400: the one at 400 is 400 ms after the registered step, so it counts even though the flicker
    # kept going in between
    series = [(0, 1.0), (100, 0.0), (200, 1.0), (300, 0.0), (400, 1.0)]
    assert feed(StepOnsets(350 * MS), series) == [0, 400]


def test_a_load_that_stays_positive_is_not_a_new_step():
    assert feed(StepOnsets(), [(0, 5.0), (500, 6.0), (1000, 7.0)]) == [0]


def test_the_first_load_is_a_step_even_at_stamp_zero():
    assert feed(StepOnsets(), [(0, 5.0)]) == [0]
