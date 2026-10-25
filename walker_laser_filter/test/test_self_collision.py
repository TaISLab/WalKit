import os
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from walker_laser_filter import self_collision as sc  # noqa: E402

N_BEAMS = 360
ANGLES = sc.beam_angles(-np.pi, 2 * np.pi / N_BEAMS, N_BEAMS)
RANGE_MIN, RANGE_MAX = 0.15, 12.0
POST = range(100, 110)                    # the walker's post: ten beams at 0.5 m


def scans(n=300, seed=1, edge=()):
    """A room 3 m away, a post that always returns 0.5 m (1 cm of noise, a scan in ten it is missed), and a leg that walks about.
    `edge`: further beams on which the post also shows in this recording, a little further away."""
    rng = np.random.default_rng(seed)
    out = np.full((n, N_BEAMS), 3.0) + rng.normal(0, 0.01, (n, N_BEAMS))
    for i in list(POST) + list(edge):
        out[:, i] = 0.5 + (0.02 if i in edge else 0.0) + rng.normal(0, 0.005, n)
        out[rng.random(n) < 0.1, i] = np.inf
    for k in range(n):                    # a leg 0.4 to 0.9 m away, on beams that change from scan to scan
        centre = rng.integers(0, N_BEAMS - 8)
        out[k, centre:centre + 6] = rng.uniform(0.4, 0.9)
    return out


def found(recordings, **kw):
    """boxes_from_beams of the structure of each recording, as find_self_collision_boxes.py does."""
    per = [sc.structure_beams(r, RANGE_MIN, RANGE_MAX) for r in recordings]
    return sc.boxes_from_beams(np.array([p[0] for p in per]), np.array([p[1] for p in per]), ANGLES, **kw)


def test_the_post_is_found_and_nothing_else():
    boxes = found([scans(seed=k) for k in range(4)])
    assert len(boxes) == 1 and boxes[0]['recordings'] == (4, 4)
    assert boxes[0]['beams'] == (POST[0], POST[-1]) and boxes[0]['persistence'] > 0.8
    # the box holds the post's points (the reference is the nearest returns, a little under the 0.5 m) with the margin, and is
    # not the size of the room or of the leg's range of movement
    x, y = 0.5 * np.cos(ANGLES[list(POST)]), 0.5 * np.sin(ANGLES[list(POST)])
    assert boxes[0]['min_x'] == pytest.approx(x.min() - sc.MARGIN_M, abs=0.01)
    assert boxes[0]['max_y'] == pytest.approx(y.max() + sc.MARGIN_M, abs=0.01)
    assert boxes[0]['max_x'] - boxes[0]['min_x'] < 0.2 and boxes[0]['max_y'] - boxes[0]['min_y'] < 0.2


def test_a_static_thing_further_than_max_dist_is_not_the_walker():
    ref, pers = sc.structure_beams(scans(), RANGE_MIN, RANGE_MAX)
    assert np.isnan(ref[0]) and pers[0] == 0.0               # the wall at 3 m, which never moves either
    ref, pers = sc.structure_beams(scans(), RANGE_MIN, RANGE_MAX, max_dist=5.0)
    (room,) = sc.boxes_from_beams(ref, pers, ANGLES)         # ... and it would be, the whole room, if we looked that far
    assert room['max_x'] > 2.5 and room['beams'] == (0, N_BEAMS - 1)


def test_a_leg_that_stays_put_for_a_part_of_the_scans_is_not_the_walker():
    recordings = [scans(seed=k) for k in range(4)]
    recordings[0][:, 200:206] = 0.45                         # a user who did not move at all in one recording
    assert [b['beams'] for b in found(recordings)] == [(POST[0], POST[-1])]        # ... is not in three of them
    for r in recordings[1:3]:
        r[:, 200:206] = 0.45
    assert [b['beams'] for b in found(recordings, min_recordings=4)] == [(POST[0], POST[-1])]


def test_the_edge_beams_of_the_post_that_show_in_some_sessions_are_in_the_box_and_the_rare_ones_are_not():
    edge_often, edge_rarely = (110, 111), (96, 97)
    recordings = [scans(seed=k, edge=edge_often + (edge_rarely if k < 2 else ()) if k < 3 else ()) for k in range(7)]
    # pooled, an edge beam that shows in three sessions of seven would return its range in too few of the scans
    _, pooled = sc.structure_beams(np.concatenate(recordings), RANGE_MIN, RANGE_MAX)
    assert pooled[110] < sc.MIN_PERSISTENCE <= pooled[105]
    (box,) = found(recordings)
    assert box['beams'] == (POST[0], 111) and box['recordings'] == (3, 7)          # the beams of 2 of 7 are left out


def test_a_patch_of_too_few_beams_is_noise_and_two_patches_stay_apart():
    ref = np.full(N_BEAMS, 0.5)                              # one recording, as plain arrays
    pers = np.zeros(N_BEAMS)
    pers[[10, 11]] = 1.0                                     # two beams: noise
    pers[50:56] = 1.0
    pers[60:66] = 1.0                                        # four beams in between: another patch
    pers[70:72] = 1.0
    pers[73:76] = 1.0                                        # one beam in between: the same patch
    assert [b['beams'] for b in sc.boxes_from_beams(ref, pers, ANGLES)] == [(50, 55), (60, 65), (70, 75)]


def test_boxes_remove_what_is_inside_and_invert_keeps_only_it():
    box = [{'min_x': 0.0, 'max_x': 1.0, 'min_y': -0.5, 'max_y': 0.5}]
    angles = np.array([0.0, np.pi])                          # one beam ahead (x > 0), one behind
    ranges = np.array([0.5, 0.5])
    assert np.isnan(sc.apply_boxes(ranges, angles, box)).tolist() == [True, False]
    assert np.isnan(sc.apply_boxes(ranges, angles, box, invert=True)).tolist() == [False, True]
    assert ranges.tolist() == [0.5, 0.5]                     # the input is not touched
    assert sc.apply_boxes(np.array([[0.5, 0.5]] * 3), angles, box).shape == (3, 2)


def test_readings_that_are_not_returns_are_left_as_they_are():
    box = [{'min_x': 0.0, 'max_x': 1.0, 'min_y': -0.5, 'max_y': 0.5}]
    angles = np.zeros(5)
    ranges = np.array([np.inf, np.nan, 0.05, 20.0, 12.0])    # no return, no return, below range_min, beyond range_max, AT range_max
    out = sc.apply_boxes(ranges, angles, box, invert=True, range_min=0.15, range_max=12.0)
    assert out[0] == np.inf and out[2] == 0.05 and out[3] == 20.0 and out[4] == 12.0 and np.isnan(out[1])


def test_the_yaml_is_a_filter_chain_with_one_box_filter_per_box():
    boxes = found([scans(seed=k) for k in range(3)])
    text = sc.box_filter_yaml(boxes, ['a comment', ''])
    assert text.startswith('# a comment\n#\n/**:')
    filters = yaml.safe_load(text)['/**']['ros__parameters']
    assert list(filters) == ['filter1']
    f = filters['filter1']
    assert f['type'] == 'laser_filters/LaserScanBoxFilter' and f['params']['invert'] is False
    assert f['params']['box_frame'] == 'laser' and f['params']['min_x'] == pytest.approx(boxes[0]['min_x'], abs=5e-4)
    assert yaml.safe_load(sc.box_filter_yaml([]))['/**']['ros__parameters'] == {}
