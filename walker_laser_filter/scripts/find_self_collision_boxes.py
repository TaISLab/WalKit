#!/usr/bin/env python3
"""Finds from recorded scans where the lidar sees the walker itself and writes the box filters that remove it.

    ros2 run walker_laser_filter find_self_collision_boxes.py                              # the definitive bags, report only
    ros2 run walker_laser_filter find_self_collision_boxes.py ~/bagsFolder/definitive --out config/self_filter.yaml
    ros2 run walker_laser_filter find_self_collision_boxes.py CA_test01 MF_test07 --min-persistence 0.6

The walker's structure returns the same short range on the same beam in every scan, whoever uses it; the rest of what is close
moves. See walker_laser_filter/self_collision.py for the criterion and what each option does. The result is the parameter file of
`self_filter.launch.py` (config/self_filter.yaml); the comment lines at its top say which data and options it came from, so it can
be reproduced. Reading the bags is walker_replay's (bagio).
"""
import argparse
import datetime
import sys
from pathlib import Path

import numpy as np
from walker_replay import bagio

from walker_laser_filter import self_collision as sc


def bag_folders(paths):
    """The bags named, or all the bags found under a folder."""
    out = []
    for p in paths:
        p = Path(p).expanduser()
        out += [p] if (p / 'metadata.yaml').is_file() else bagio.find_bags(p)
    if not out:
        sys.exit('no rosbag2 folder found in: ' + ', '.join(map(str, paths)))
    return out


def load_scans(bags, topic):
    """([ranges (scans, beams) of each bag], angles, range_min, range_max, frame). Only the scans on the beam grid of the first one
    are used: the beam index must mean the same direction in every scan."""
    groups, grid, skipped = [], None, 0
    for bag in bags:
        rows = []
        for _, _, m in bagio.iter_messages(bag, [topic]):
            this = (m.header.frame_id, len(m.ranges), round(m.angle_min, 5), round(m.angle_increment, 7), m.range_min, m.range_max)
            if grid is None:
                grid = this
            if this != grid:
                skipped += 1
                continue
            rows.append(np.asarray(m.ranges, dtype=np.float32))
        if rows:
            groups.append(np.stack(rows))
    if not groups:
        sys.exit(f'no {topic} scans in the bags')
    if skipped:
        print(f'WARNING: {skipped} scans are not on the beam grid of the first one {grid} and were left out', file=sys.stderr)
    frame, n_beams, angle_min, increment, range_min, range_max = grid
    return groups, sc.beam_angles(angle_min, increment, n_beams), range_min, range_max, frame


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='*', default=['~/bagsFolder/definitive'],
                    help='rosbag2 folders, or folders with rosbag2 folders in them (default: ~/bagsFolder/definitive)')
    ap.add_argument('--topic', default='/scan', help='the raw scan (default /scan)')
    ap.add_argument('--out', help='write the filter parameter file here (default: only print the report)')
    ap.add_argument('--max-dist', type=float, default=sc.MAX_DIST_M, help='the structure is closer than this, m (%(default)s)')
    ap.add_argument('--percentile', type=float, default=sc.PERCENTILE, help='nearest-return percentile that is the reference (%(default)s)')
    ap.add_argument('--tolerance', type=float, default=sc.TOLERANCE_M, help='same return if within this, m (%(default)s)')
    ap.add_argument('--min-persistence', type=float, default=sc.MIN_PERSISTENCE,
                    help='fraction of the scans of a bag that return it (%(default)s)')
    ap.add_argument('--min-bags', type=int, default=sc.MIN_RECORDINGS, help='bags in which a beam has to be structure (%(default)s)')
    ap.add_argument('--margin', type=float, default=sc.MARGIN_M, help='added around each box, m (%(default)s)')
    ap.add_argument('--max-gap', type=int, default=sc.MAX_GAP_BEAMS, help='beams between two of a patch (%(default)s)')
    ap.add_argument('--min-beams', type=int, default=sc.MIN_BEAMS, help='beams of the smallest patch (%(default)s)')
    args = ap.parse_args(argv)

    bags = bag_folders(args.bags)
    groups, angles, range_min, range_max, frame = load_scans(bags, args.topic)
    found = [sc.structure_beams(g, range_min, range_max, args.max_dist, args.percentile, args.tolerance) for g in groups]
    references, persistences = np.array([f[0] for f in found]), np.array([f[1] for f in found])
    boxes = sc.boxes_from_beams(references, persistences, angles, args.min_persistence, args.min_bags, args.margin,
                                args.max_gap, args.min_beams)

    n_scans = sum(len(g) for g in groups)
    near = removed = 0
    for g in groups:
        close = sc.valid_ranges(g, range_min, range_max) & (g < 1.5)
        near += close.sum()
        removed += (np.isnan(sc.apply_boxes(g, angles, boxes, range_min=range_min, range_max=range_max)) & close).sum()
    print(f'{len(groups)} bags, {n_scans} scans of {len(angles)} beams in frame [{frame}]; {len(boxes)} patch(es) of the walker in view')
    for i, b in enumerate(boxes, 1):
        a, z = b['beams']
        print(f'  box {i}: beams {a}-{z} ({np.degrees(angles[a]):+.0f} to {np.degrees(angles[z]):+.0f} deg), '
              f'structure in {b["recordings"][0]}-{b["recordings"][1]} bags, persistence {b["persistence"]:.2f}  '
              f'x [{b["min_x"]:+.3f}, {b["max_x"]:+.3f}]  y [{b["min_y"]:+.3f}, {b["max_y"]:+.3f}]')
    print(f'  they remove {removed / max(near, 1):.1%} of the returns closer than 1.5 m')

    if args.out:
        comments = [
            'Where the lidar sees the walker itself: box filters for scan_to_scan_filter_chain (self_filter.launch.py).',
            'GENERATED by find_self_collision_boxes.py, do not edit by hand: run it again.',
            f'{datetime.date.today()}: {len(bags)} bags ({bags[0].name} ... {bags[-1].name}), {n_scans} scans of {args.topic}.',
            f'--max-dist {args.max_dist} --percentile {args.percentile} --tolerance {args.tolerance} '
            f'--min-persistence {args.min_persistence} --min-bags {args.min_bags} --margin {args.margin} '
            f'--max-gap {args.max_gap} --min-beams {args.min_beams}',
            '']
        for i, b in enumerate(boxes, 1):
            comments.append(f'box {i}: beams {b["beams"][0]}-{b["beams"][1]}, structure in {b["recordings"][0]}-{b["recordings"][1]} bags '
                            f'(its range returned in {b["persistence"]:.0%} of the scans of each)')
        Path(args.out).expanduser().write_text(sc.box_filter_yaml(boxes, comments), encoding='utf-8')
        print('written', args.out)


if __name__ == '__main__':
    main()
