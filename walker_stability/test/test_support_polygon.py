import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from walker_stability import support_polygon as sp  # noqa: E402

SQUARE = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]


def test_hull_drops_inner_and_collinear_points_and_is_counter_clockwise():
    hull = sp.convex_hull(SQUARE + [(0.5, 0.5), (0.5, 0.0), (0.0, 0.0)])
    assert sorted(hull) == sorted(SQUARE)
    assert sp.area(hull) == pytest.approx(1.0)          # positive: counter-clockwise


def test_hull_of_few_or_collinear_points():
    assert sp.convex_hull([]) == []
    assert sp.convex_hull([(1.0, 2.0), (1.0, 2.0)]) == [(1.0, 2.0)]
    assert len(sp.convex_hull([(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)])) == 2
    assert sp.area(sp.convex_hull([(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)])) == 0.0


def test_margin_is_the_distance_to_the_nearest_edge():
    hull = sp.convex_hull(SQUARE)
    assert sp.margin((0.5, 0.5), hull) == pytest.approx(0.5)
    assert sp.margin((0.2, 0.5), hull) == pytest.approx(0.2)
    assert sp.margin((0.5, 0.9), hull) == pytest.approx(0.1)
    assert sp.margin((1.0, 0.5), hull) == pytest.approx(0.0)


def test_margin_is_negative_outside_and_measures_the_distance_to_the_polygon():
    hull = sp.convex_hull(SQUARE)
    assert sp.margin((1.5, 0.5), hull) == pytest.approx(-0.5)
    assert sp.margin((2.0, 2.0), hull) == pytest.approx(-math.sqrt(2.0))      # to the corner, not to an edge's line


def test_margin_in_a_triangle_is_the_inradius_at_its_incentre():
    hull = sp.convex_hull([(0.0, 0.0), (4.0, 0.0), (0.0, 3.0)])             # 3-4-5: inradius (3+4-5)/2 = 1
    assert sp.margin((1.0, 1.0), hull) == pytest.approx(1.0)


def test_a_polygon_without_area_has_nothing_to_stand_in():
    segment = sp.convex_hull([(0.0, 0.0), (2.0, 0.0)])
    assert sp.margin((1.0, 0.0), segment) == pytest.approx(0.0)
    assert sp.margin((1.0, 0.5), segment) == pytest.approx(-0.5)
    point = sp.convex_hull([(0.0, 0.0)])
    assert sp.margin((3.0, 4.0), point) == pytest.approx(-5.0)
    assert sp.margin((0.0, 0.0), []) is None


def test_centre_of_pressure_is_the_load_weighted_barycentre():
    contacts = [sp.Contact(sp.FOOT, 0.0, 0.0, 30.0), sp.Contact(sp.FOOT, 1.0, 0.0, 10.0),
                sp.Contact(sp.HANDLE, 0.0, 1.0, 10.0), sp.Contact(sp.HANDLE, 1.0, 1.0, 10.0)]
    cx, cy = sp.centre_of_pressure(contacts)
    assert (cx, cy) == pytest.approx((1 / 3, 1 / 3))
    assert sp.centre_of_pressure([sp.Contact(sp.FOOT, 1.0, 1.0, 0.0)]) is None
    assert sp.centre_of_pressure([]) is None


def test_the_centre_of_pressure_is_inside_the_polygon_of_the_contacts():
    contacts = [sp.Contact(sp.FOOT, 0.0, 0.0, 20.0), sp.Contact(sp.FOOT, 0.3, 0.3, 40.0),
                sp.Contact(sp.HANDLE, 0.5, -0.2, 5.0), sp.Contact(sp.HANDLE, 0.5, 0.4, 8.0)]
    cop = sp.centre_of_pressure(contacts)
    assert sp.margin(cop, sp.convex_hull([(c.x, c.y) for c in contacts])) > 0.0


def test_footprint_is_the_sole_around_the_tracked_point_pointing_forward():
    corners = sp.footprint(1.0, 2.0, heel_m=0.1, toe_m=0.2, half_width_m=0.05)
    hull = sp.convex_hull(corners)
    assert sp.area(hull) == pytest.approx(0.3 * 0.1)
    assert min(c[0] for c in corners) == pytest.approx(0.9) and max(c[0] for c in corners) == pytest.approx(1.2)
    assert sp.margin((1.0, 2.0), hull) == pytest.approx(0.05)          # lateral margin of the tracked point on its own foot


def test_index_is_the_clipped_margin_over_the_reference():
    assert sp.index(0.05, 0.10) == pytest.approx(0.5)
    assert sp.index(0.30, 0.10) == 1.0
    assert sp.index(-0.2, 0.10) == 0.0
    assert sp.index(None, 0.10) == 0.0


def test_assess_ties_it_together():
    a = sp.assess(SQUARE, (0.5, 0.5), reference_m=1.0)
    assert (a.area, a.margin, a.sta) == (pytest.approx(1.0), pytest.approx(0.5), pytest.approx(0.5))
    assert sp.assess([], (0.5, 0.5), reference_m=1.0) == (0.0, 0.0, 0.0)
    assert sp.assess(SQUARE, None, reference_m=1.0) == (pytest.approx(1.0), 0.0, 0.0)
