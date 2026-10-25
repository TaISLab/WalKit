"""Where the lidar sees the walker itself, found from recorded scans, and the box filters that remove it.

The lidar is fixed to the walker, so the walker's own structure (frame, posts) returns the same range on the same beam in every
scan, whoever is using it and wherever it is. Everything else that is close (the user's legs, a wall, a person) moves with
respect to the walker. A beam therefore "sees the structure" when, across the scans of one recording, it keeps returning the
same short range. `structure_beams` measures that for one recording, and `boxes_from_beams` takes what several recordings (bags,
sessions) found and turns the beams into the rectangles `laser_filters/LaserScanBoxFilter` takes. It is done recording by
recording and then put together, not on all the scans at once, because the edge beams of a post flip between the post and the
background by a hair's breadth and do so differently from one session to the next: pooled, they never reach the persistence
they have inside each session, and their returns would be left in the scan as small clusters that a step detector takes for
legs. `box_filter_yaml` writes the boxes as a parameter file for `scan_to_scan_filter_chain`.

Pure numpy: no ROS. `apply_boxes` is a reference implementation of what the official box filter does to a scan (a point inside
the box is removed, or with `invert` a point outside it), to check the boxes against scans without running the filter.
"""
import warnings

import numpy as np

# Defaults of find_self_collision_boxes.py. Measured on the 47 definitive bags (24314 scans, four users): the same four patches
# for every persistence between 0.4 and 0.8 and for each user's bags on their own.
MAX_DIST_M = 1.0          # the structure is closer than this; a static wall further away is not the walker
PERCENTILE = 10.0         # the nearest return a beam keeps giving: the structure is the nearest thing there is
TOLERANCE_M = 0.03        # a return within this of the reference is "the same return" (the structure's own spread is 1-2 cm)
MIN_PERSISTENCE = 0.5     # fraction of the scans of a recording in which a beam returns its reference range
MIN_RECORDINGS = 3        # recordings in which a beam has to be structure: a person does not stand still for half of three of them
MARGIN_M = 0.03           # added around the beams found: the tolerance, and the arc a beam covers
MAX_GAP_BEAMS = 2         # beams between two structure beams that still belong to the same patch
MIN_BEAMS = 3             # a patch of fewer beams is noise


def beam_angles(angle_min, angle_increment, n_beams):
    return angle_min + angle_increment * np.arange(n_beams)


def valid_ranges(ranges, range_min, range_max):
    """Mask of the readings that are a return: finite, at least range_min and below range_max (a reading AT range_max is how the
    driver says "nothing there", and laser_geometry, which the box filter projects the scan with, leaves it out)."""
    return np.isfinite(ranges) & (ranges >= range_min) & (ranges < range_max)


def structure_beams(ranges, range_min, range_max, max_dist=MAX_DIST_M, percentile=PERCENTILE, tolerance=TOLERANCE_M):
    """(reference, persistence) per beam, from `ranges`, an array (scans, beams) on a fixed beam grid.

    reference: the `percentile` of the beam's returns, NaN if there are too few of them or it is further than `max_dist`.
    persistence: the fraction of ALL the scans in which the beam returned within `tolerance` of the reference.
    Meant for the scans of one recording; `boxes_from_beams` puts several recordings together."""
    ranges = np.asarray(ranges, dtype=float)
    n_scans, n_beams = ranges.shape
    valid = valid_ranges(ranges, range_min, range_max)
    reference = np.full(n_beams, np.nan)
    persistence = np.zeros(n_beams)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)           # a beam that never returned is an all-NaN column
        nearest = np.nanpercentile(np.where(valid, ranges, np.nan), percentile, axis=0)
    for i in range(n_beams):
        if valid[:, i].sum() < min(50, n_scans) or not nearest[i] <= max_dist:
            continue
        reference[i] = nearest[i]
        persistence[i] = np.mean(valid[:, i] & (np.abs(ranges[:, i] - nearest[i]) <= tolerance))
    return reference, persistence


def boxes_from_beams(references, persistences, angles, min_persistence=MIN_PERSISTENCE, min_recordings=MIN_RECORDINGS,
                     margin=MARGIN_M, max_gap=MAX_GAP_BEAMS, min_beams=MIN_BEAMS):
    """One box per patch of neighbouring structure beams, from what `structure_beams` found in each recording
    (`references`, `persistences`: arrays (recordings, beams); a single recording may be given as plain (beams,) arrays).

    A beam is structure in a recording when its persistence there is at least `min_persistence`, and a beam of the walker when it
    is so in at least `min_recordings` of them (all of them, if there are fewer). A patch is a run of such beams with up to
    `max_gap` others between them, of at least `min_beams`. Its box holds the points (beam angle, reference range) of every
    recording in which they are structure, plus `margin` on every side.
    [{'min_x', 'max_x', 'min_y', 'max_y', 'beams': (first, last), 'persistence', 'recordings': (fewest, most)}]"""
    references = np.atleast_2d(references)
    persistences = np.atleast_2d(persistences)
    member = persistences >= min_persistence
    support = member.sum(axis=0)
    idx = np.where(support >= min(min_recordings, len(references)))[0]
    boxes = []
    if len(idx) == 0:
        return boxes
    for run in np.split(idx, np.where(np.diff(idx) > max_gap + 1)[0] + 1):
        if len(run) < min_beams:
            continue
        x, y = [], []
        for i in run:
            r = references[member[:, i], i]
            x += list(r * np.cos(angles[i]))
            y += list(r * np.sin(angles[i]))
        boxes.append({'min_x': float(min(x) - margin), 'max_x': float(max(x) + margin),
                      'min_y': float(min(y) - margin), 'max_y': float(max(y) + margin),
                      'beams': (int(run[0]), int(run[-1])), 'persistence': float(persistences[:, run][member[:, run]].mean()),
                      'recordings': (int(support[run].min()), int(support[run].max()))})
    return sorted(boxes, key=lambda b: (b['min_x'], b['min_y']))


def apply_boxes(ranges, angles, boxes, invert=False, range_min=0.0, range_max=np.inf):
    """What the official box filter does to scans (`ranges`: (beams,) or (scans, beams), the box frame being the laser's): a
    return inside any box becomes NaN, or with `invert` a return outside every box (the boxes are then one active area).
    Only returns are touched: a reading that is not one (NaN, inf, outside [range_min, range_max]) stays as it is."""
    ranges = np.array(ranges, dtype=float)
    valid = valid_ranges(ranges, range_min, range_max)
    with np.errstate(invalid='ignore'):                      # float32 and strict inequalities, like laser_geometry and inBox()
        x = (ranges * np.cos(angles)).astype(np.float32)
        y = (ranges * np.sin(angles)).astype(np.float32)
    inside = np.zeros(ranges.shape, dtype=bool)
    for b in boxes:
        inside |= ((x > np.float32(b['min_x'])) & (x < np.float32(b['max_x'])) & (y > np.float32(b['min_y'])) & (y < np.float32(b['max_y'])))
    ranges[valid & (inside != invert)] = np.nan
    return ranges


def box_filter_yaml(boxes, comments=(), name='self', z=1.0):
    """The parameter file of a `scan_to_scan_filter_chain` with one `LaserScanBoxFilter` per box, which removes what is inside it."""
    lines = ['# ' + c if c else '#' for c in comments]
    lines += ['/**:', '  ros__parameters:']
    if not boxes:
        lines.append('    {}')
    for i, b in enumerate(boxes, 1):
        lines += [f'    filter{i}:',
                  f'      name: {name}_{i}',
                  '      type: laser_filters/LaserScanBoxFilter',
                  '      params:',
                  '        box_frame: laser',
                  f'        min_x: {b["min_x"]:.3f}', f'        max_x: {b["max_x"]:.3f}',
                  f'        min_y: {b["min_y"]:.3f}', f'        max_y: {b["max_y"]:.3f}',
                  f'        min_z: {-z:.1f}', f'        max_z: {z:.1f}',
                  '        invert: false']
    return '\n'.join(lines) + '\n'
