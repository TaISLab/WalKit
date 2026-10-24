import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import calibrate_handle as ch  # noqa: E402


def readings(value, n=20, noise=0.0):
    return [value + noise * ((-1) ** i) for i in range(n)]


def test_the_first_readings_are_dropped_and_the_median_ignores_outliers():
    value, spread = ch.summarize([500.0] * 4 + [100.0] * 15 + [9999.0])
    assert value == pytest.approx(100.0) and spread < 1.0


def test_too_few_readings_are_refused():
    with pytest.raises(ValueError):
        ch.summarize([1.0, 2.0])


def test_the_table_is_sorted_by_mass_and_must_grow():
    masses, values, _ = ch.build_table({10.0: readings(334.6), 0.0: readings(2.7), 5.0: readings(190.0)})
    assert masses == [0.0, 5.0, 10.0] and values == [2.7, 190.0, 334.6]
    with pytest.raises(ValueError):
        ch.build_table({0.0: readings(2.7), 5.0: readings(190.0), 10.0: readings(150.0)})


def test_linearity_finds_the_slope_and_flags_a_kink():
    masses = [0.0, 10.0, 20.0, 30.0, 40.0]
    slope, offset, worst = ch.linearity(masses, [3.0 + 33.4 * m for m in masses])
    assert slope == pytest.approx(33.4) and offset == pytest.approx(3.0) and worst == pytest.approx(0.0, abs=1e-9)
    assert ch.linearity(masses, [3.0 + 33.4 * m for m in masses[:3]] + [900.0, 1000.0])[2] > 1.0


def test_the_yaml_lines_have_the_keys_partial_loads_reads():
    lines = ch.yaml_lines('left', [0.0, 1.0], [2.7, 46.7])
    assert lines[0].startswith('weight_points:') and lines[1].startswith('left_handle_points:') and '[2.7, 46.7]' in lines[1]


def test_csv_with_a_header_and_comments(tmp_path):
    f = tmp_path / 'c.csv'
    f.write_text('mass_kg,reading\n# a comment\n0,2.7\n0,2.8\n5,190.0\n')
    assert ch.read_csv(str(f)) == {0.0: [2.7, 2.8], 5.0: [190.0]}
