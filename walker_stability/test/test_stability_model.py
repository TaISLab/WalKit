import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from walker_stability.stability_model import LEFT, RIGHT, StabilityModel  # noqa: E402

HANDLES = {LEFT: (0.0, 0.24), RIGHT: (0.0, -0.24)}


def model(**kw):
    kw.setdefault('foot_heel_m', 0.1)             # a symmetric sole: the centre of pressure is at the tracked point
    kw.setdefault('foot_toe_m', 0.1)
    kw.setdefault('com_moment_kgm', 0.0)
    m = StabilityModel(**kw)
    for side, (x, y) in HANDLES.items():
        m.set_handle(side, x, y, 5.0)
    return m


def test_nothing_to_evaluate_until_the_four_contacts_have_been_seen():
    m = StabilityModel()
    assert m.evaluate(0.0) is None
    m.set_foot(LEFT, -0.6, 0.1, 30.0)
    m.set_foot(RIGHT, -0.6, -0.1, 30.0)
    m.set_handle(LEFT, 0.0, 0.24, 5.0)
    assert not m.ready and m.evaluate(0.0) is None
    m.set_handle(RIGHT, 0.0, -0.24, 5.0)
    assert m.ready and m.evaluate(0.0) is not None


def test_no_load_at_all_is_nothing_to_evaluate():
    m = StabilityModel()
    for side, (x, y) in HANDLES.items():
        m.set_handle(side, x, y, 0.0)
    m.set_foot(LEFT, -0.6, 0.1, 0.0)
    m.set_foot(RIGHT, -0.6, -0.1, 0.0)
    assert m.evaluate(0.0) is None


def test_centre_of_pressure_is_the_load_weighted_barycentre_of_the_four_contacts():
    m = model()
    m.set_foot(LEFT, -0.6, 0.1, 30.0)
    m.set_foot(RIGHT, -0.6, -0.1, 10.0)
    cop = m.evaluate(0.0).cop
    assert cop[0] == pytest.approx((-0.6 * 40.0) / 50.0 + 0.0)
    assert cop[1] == pytest.approx((0.1 * 30.0 - 0.1 * 10.0 + 0.24 * 5.0 - 0.24 * 5.0) / 50.0)


def test_both_feet_down_the_polygon_is_the_feet_and_the_handles_and_the_margin_is_positive():
    m = model(com_tau_s=0.0)
    m.set_foot(LEFT, -0.6, 0.1, 30.0)
    m.set_foot(RIGHT, -0.6, -0.1, 30.0)
    e = m.evaluate(0.0)
    assert e.contacts == 4
    assert e.assessment.area > 0.1
    assert 0.0 < e.assessment.margin < 0.2
    assert 0.0 < e.assessment.sta <= 1.0


def test_a_swing_foot_does_not_count_and_one_foot_stance_still_has_a_polygon_thanks_to_the_handles():
    m = model(com_tau_s=0.0)
    m.set_foot(LEFT, -0.6, 0.1, 55.0)
    m.set_foot(RIGHT, -0.3, -0.1, 0.0)
    e = m.evaluate(0.0)
    assert e.contacts == 3
    assert e.assessment.area > 0.0


def test_the_centre_of_mass_estimate_follows_the_centre_of_pressure_with_the_time_constant():
    m = model(com_tau_s=1.0)
    m.set_foot(LEFT, -0.6, 0.1, 30.0)
    m.set_foot(RIGHT, -0.6, -0.1, 30.0)
    first = m.evaluate(0.0)
    assert first.com == first.cop
    m.set_foot(LEFT, -0.6, 0.1, 60.0)
    m.set_foot(RIGHT, -0.6, -0.1, 0.0)           # the pressure goes to the left foot
    later = m.evaluate(1.0)                       # one time constant after: halfway there
    assert later.cop[1] > 0.05
    assert later.com[1] == pytest.approx(first.com[1] + 0.5 * (later.cop[1] - first.com[1]))
    assert first.com[1] < later.com[1] < later.cop[1]


def test_same_stamps_give_same_numbers_and_a_repeated_stamp_does_not_move_the_filter():
    def run():
        m = model()
        out = []
        for k in range(10):
            m.set_foot(LEFT, -0.6, 0.1, 10.0 * k)
            m.set_foot(RIGHT, -0.6, -0.1, 50.0 - 4.0 * k)
            out.append(m.evaluate(0.05 * k))
        return out
    assert run() == run()
    m = model()
    m.set_foot(LEFT, -0.6, 0.1, 30.0)
    m.set_foot(RIGHT, -0.6, -0.1, 30.0)
    a = m.evaluate(1.0)
    m.set_foot(LEFT, -0.6, 0.1, 60.0)
    b = m.evaluate(1.0)
    assert b.com == a.com


def two_feet(m, load=30.0):
    m.set_foot(LEFT, -0.6, 0.1, load)
    m.set_foot(RIGHT, -0.6, -0.1, load)


def test_a_loaded_foot_pushes_at_the_centre_of_its_sole_not_at_the_tracked_point():
    plain, forward = model(), model(foot_heel_m=0.08, foot_toe_m=0.18)
    two_feet(plain)
    two_feet(forward)
    a, b = plain.evaluate(0.0).cop, forward.evaluate(0.0).cop
    shift = 0.05 * 60.0 / 70.0                       # 5 cm for the feet's share (60 of 70 kg) of the pressure
    assert b[0] - a[0] == pytest.approx(shift) and b[1] == pytest.approx(a[1])


def test_the_moment_correction_moves_the_centre_of_mass_forward_by_moment_over_weight():
    m = model(com_moment_kgm=6.0, com_tau_s=0.0)
    two_feet(m)
    without = m.evaluate(0.0)
    assert without.com == without.cop                 # no weight known: no correction
    m.set_weight(60)
    with_weight = m.evaluate(0.1)
    assert with_weight.com[0] - with_weight.cop[0] == pytest.approx(0.1)
    assert with_weight.com[1] == pytest.approx(with_weight.cop[1])
    assert with_weight.assessment.margin != without.assessment.margin
    m.set_weight(0)                                   # an invalid weight switches it off again
    assert m.evaluate(0.2).com == m.evaluate(0.2).cop


def test_the_polygon_is_the_hull_of_the_loaded_soles_and_both_handles():
    m = model(com_tau_s=0.0)
    m.set_foot(LEFT, -0.6, 0.1, 55.0)
    m.set_foot(RIGHT, -0.3, -0.1, 0.0)                # a swing foot is not in it
    polygon = m.evaluate(0.0).polygon
    assert len(polygon) >= 3
    xs = [x for x, _ in polygon]
    assert min(xs) == pytest.approx(-0.7) and max(xs) == pytest.approx(0.0)       # from the left sole (heel 0.1) to the handles
    assert (0.0, 0.24) in polygon and (0.0, -0.24) in polygon
    assert -0.3 not in xs
