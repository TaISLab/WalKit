import math
from pathlib import Path

import numpy as np
import pytest

from walker_stability import bos

XLSX = Path(__file__).resolve().parent.parent / 'config' / 'Correlationes.xlsx'


@pytest.fixture(scope='module')
def tables():
    return bos.AnthropometricTables(XLSX)


def brute_force(row, height_m, handlebar_m):
    """The search as it was first written (loops), in radians."""
    shoulder, arm, forearm = bos.arm_geometry_mm(row, height_m * 1000.0)
    target = shoulder - handlebar_m * 1000.0
    reach = []
    for alpha in np.arange(0, 90, 0.1):
        for beta in np.arange(alpha, 180, 0.1):
            ext = beta - alpha
            if 20 <= ext <= 30 and abs(forearm * math.sin(math.radians(alpha)) + arm * math.sin(math.radians(beta)) - target) < 10:
                reach.append(forearm * math.cos(math.radians(alpha)) + arm * math.cos(math.radians(beta)))
    return (min(reach) / 1000, max(reach) / 1000) if reach else None


def test_age_brackets(tables):
    assert tables.row('Masculino', 52)['Min_age'] == 50
    assert tables.row('Masculino', 79)['Min_age'] == 75
    assert tables.row('Masculino', 95)['Min_age'] == 80          # open last bracket
    assert tables.row('Femenino', 20)['Min_age'] == 50           # younger than the tables: the nearest bracket
    assert tables.row('Femenino', 20) is not tables.row('Masculino', 20)


@pytest.mark.parametrize('gender,age,height,handlebar', [('Femenino', 20, 1.63, 0.82), ('Masculino', 44, 1.75, 0.88),
                                                         ('Masculino', 70, 1.70, 0.86)])
def test_vectorized_search_matches_the_loops(tables, gender, age, height, handlebar):
    row = tables.row(gender, age)
    fast = bos.bos_x_range(row, height, handlebar)
    slow = brute_force(row, height, handlebar)
    assert fast == pytest.approx(slow, abs=2e-3)


def test_range_is_a_few_centimetres_around_arm_reach(tables):
    lo, hi = bos.bos_x_range(tables.row('Femenino', 20), 1.63, 0.82)
    assert 0.25 < lo < hi < 0.35 and hi - lo < 0.1


def test_no_posture_reaches_a_handlebar_far_above_the_shoulder(tables):
    assert bos.bos_x_range(tables.row('Masculino', 60), 1.70, 2.5) is None


def test_centered_score():
    assert bos.centered_score(0.5, 0.0, 1.0) == 1.0
    assert bos.centered_score(0.25, 0.0, 1.0) == pytest.approx(0.5)
    assert bos.centered_score(0.75, 0.0, 1.0) == pytest.approx(0.5)
    assert bos.centered_score(0.0, 0.0, 1.0) == 0.0 and bos.centered_score(1.0, 0.0, 1.0) == 0.0
    assert bos.centered_score(-3.0, 0.0, 1.0) == 0.0 and bos.centered_score(2.0, 0.0, 1.0) == 0.0
    assert bos.centered_score(0.5, 1.0, 1.0) == 0.0
