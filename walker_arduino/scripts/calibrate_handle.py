#!/usr/bin/env python3
"""Bench calibration of a handle: raw reading at known masses -> the table walker_loads/config/handle_calib.yaml needs.

The table maps the handle's reading (what /left_handle or /right_handle publish) to kilograms; `partial_loads` interpolates it with a
spline and CLAMPS readings beyond its last point to that point's weight, so a table that ends at 20 kg reports 20 kg for any load above
(in the definitive bags the left handle was above the table's end in up to 22 % of the samples of some MF and LA tests). Extend the table
to the loads the users really put on the handles (50 kg).

Interactive (ROS sourced, handle node running):

    python3 calibrate_handle.py --topic /left_handle --side left --masses 0 1 2 4 6 8 10 15 20 30 40 50

For each mass it asks you to hang or press that mass on the handle AT THE GRIP POSITION (not at the sensor), averages the reading for
--seconds, and prints the table. Do it once with the load straight down and, to check the cross sensitivity, once pushing forward;
and do the 0 kg point with nobody touching the walker (the zero is taken at power-up, see firmware/Handle-fw/README.md).
Without ROS: --csv FILE with "mass_kg,reading" rows (as many readings per mass as you like).

Output: the two lines for handle_calib.yaml (weight_points and <side>_handle_points) and a linearity report.
"""
import argparse
import csv
import statistics
import sys
import time


def summarize(readings, skip_fraction=0.2):
    """Robust value of one mass: the median of the readings after dropping the first `skip_fraction` (the load settling), and the
    spread (the median absolute deviation, scaled to a standard deviation)."""
    kept = readings[int(len(readings) * skip_fraction):]
    if len(kept) < 3:
        raise ValueError('Too few readings (%d) to trust' % len(readings))
    median = statistics.median(kept)
    return median, 1.4826 * statistics.median(abs(v - median) for v in kept)


def build_table(readings_by_mass):
    """{mass_kg: [readings]} -> (masses, values, spreads), sorted by mass. Raises ValueError if the table is not increasing: a handle
    whose reading does not grow with the load cannot be calibrated, and the spline would be garbage."""
    masses = sorted(readings_by_mass)
    values, spreads = [], []
    for m in masses:
        v, s = summarize(readings_by_mass[m])
        values.append(v)
        spreads.append(s)
    for (m0, v0), (m1, v1) in zip(zip(masses, values), zip(masses[1:], values[1:])):
        if v1 <= v0:
            raise ValueError('The reading does not grow from %g kg (%.2f) to %g kg (%.2f)' % (m0, v0, m1, v1))
    return masses, values, spreads


def linearity(masses, values):
    """(slope in reading per kg, offset at 0 kg, largest deviation from the straight line in kg)."""
    n = len(masses)
    mean_m, mean_v = sum(masses) / n, sum(values) / n
    slope = sum((m - mean_m) * (v - mean_v) for m, v in zip(masses, values)) / sum((m - mean_m) ** 2 for m in masses)
    offset = mean_v - slope * mean_m
    return slope, offset, max(abs((v - offset) / slope - m) for m, v in zip(masses, values))


def yaml_lines(side, masses, values):
    fmt = lambda xs: '[' + ', '.join('%.4g' % x for x in xs) + ']'  # noqa: E731
    return ['weight_points:       %s # measurement points in kg' % fmt(masses),
            '%s_handle_points: %s # handle measurement' % (side, fmt(values))]


def read_csv(path):
    out = {}
    with open(path, newline='', encoding='utf-8') as f:
        for row in csv.reader(f):
            if row and not row[0].lstrip().startswith('#'):
                try:
                    out.setdefault(float(row[0]), []).append(float(row[1]))
                except (ValueError, IndexError):
                    continue          # a header
    return out


def collect(topic, masses, seconds):
    import rclpy
    from walker_msgs.msg import ForceStamped
    rclpy.init()
    node = rclpy.create_node('calibrate_handle')
    last = []
    node.create_subscription(ForceStamped, topic, lambda m: last.append(float(m.force)), 100)
    out = {}
    try:
        for mass in masses:
            input('Put %g kg on the handle at the grip position and press Enter ' % mass)
            del last[:]
            t0 = time.time()
            while time.time() - t0 < seconds:
                rclpy.spin_once(node, timeout_sec=0.1)
            out[mass] = list(last)
            print('  %g kg: %d readings' % (mass, len(last)))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--side', choices=('left', 'right'), required=True)
    ap.add_argument('--topic', help='the handle topic (default /<side>_handle)')
    ap.add_argument('--masses', type=float, nargs='+', default=[0, 1, 2, 4, 6, 8, 10, 15, 20, 30, 40, 50])
    ap.add_argument('--seconds', type=float, default=6.0, help='how long to average each mass')
    ap.add_argument('--csv', help='mass_kg,reading rows instead of measuring')
    a = ap.parse_args(argv)
    data = read_csv(a.csv) if a.csv else collect(a.topic or '/%s_handle' % a.side, a.masses, a.seconds)
    try:
        masses, values, spreads = build_table(data)
    except ValueError as e:
        print('Cannot build the table: %s' % e, file=sys.stderr)
        return 1
    for m, v, s in zip(masses, values, spreads):
        print('%6.1f kg  reading %9.2f  (spread %.2f = %.2f kg)' % (m, v, s, s / max((values[-1] - values[0]) / (masses[-1] - masses[0]), 1e-9)))
    slope, offset, worst = linearity(masses, values)
    print('\nLinear fit: %.2f reading/kg, %.2f at 0 kg; the table is within %.2f kg of the line%s' % (
        slope, offset, worst, '' if worst < 0.5 else '  <- NOT linear: check the load point and the mounting'))
    print('\n' + '\n'.join(yaml_lines(a.side, masses, values)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
