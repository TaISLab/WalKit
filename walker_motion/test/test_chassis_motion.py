import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from walker_motion import chassis_motion as cm  # noqa: E402

FS = 50.0


def walk(seconds=20.0, stride=1.1, ap_amp=0.5, ml_amp=0.3, noise=0.0):
    """AP at the step frequency (two per stride), ML at the stride frequency, plus a little of the other harmonics."""
    n = int(seconds * FS)
    ap = [ap_amp * math.sin(2 * math.pi * 2 * i / FS / stride) + 0.1 * ap_amp * math.sin(2 * math.pi * 3 * i / FS / stride) for i in range(n)]
    ml = [ml_amp * math.sin(2 * math.pi * i / FS / stride) for i in range(n)]
    return ap, ml


def test_rms_is_the_standard_deviation_and_ignores_the_mean():
    assert cm.rms([5.0, 5.0, 5.0]) == pytest.approx(0.0)
    assert cm.rms([1.0, -1.0] * 10) == pytest.approx(1.0)
    assert cm.rms([4.0 + math.sin(i) for i in range(1000)]) == pytest.approx(math.sqrt(0.5), rel=0.02)
    assert math.isnan(cm.rms([]))


def test_the_stride_time_comes_out_of_the_signals():
    ap, ml = walk(20.0, 1.1)
    assert cm.stride_time(ap, ml, FS) == pytest.approx(1.1, abs=2 / FS)
    ap, ml = walk(20.0, 1.4)
    assert cm.stride_time(ap, ml, FS) == pytest.approx(1.4, abs=2 / FS)


def test_no_stride_time_without_a_periodic_signal_or_enough_data():
    import random
    rng = random.Random(1)
    noise = [rng.gauss(0, 1) for _ in range(1000)]
    assert math.isnan(cm.stride_time(noise, noise[::-1], FS))
    assert math.isnan(cm.stride_time([0.0] * 1000, [0.0] * 1000, FS))
    ap, ml = walk(2.0)
    assert math.isnan(cm.stride_time(ap, ml, FS))


def test_harmonic_ratio_is_high_for_a_smooth_periodic_gait_and_low_for_a_stumbling_one():
    ap, ml = walk(20.0, 1.1)
    smooth = cm.harmonic_ratio(ap, FS, 1.1, even_over_odd=True)
    assert smooth > 3.0                                          # AP is made of even harmonics of the stride
    assert cm.harmonic_ratio(ml, FS, 1.1, even_over_odd=False) > 3.0
    stumbling = [a + (0.4 if (i // 55) % 2 else 0.0) * math.sin(2 * math.pi * i / FS / 1.1) for i, a in enumerate(ap)]
    assert cm.harmonic_ratio(stumbling, FS, 1.1, even_over_odd=True) < smooth
    assert math.isnan(cm.harmonic_ratio(ap, FS, math.nan))


def test_regularity_is_one_for_a_periodic_signal():
    ap, _ = walk(20.0, 1.1)
    step, stride = cm.regularity(ap, FS, 1.1)
    assert step > 0.9 and stride > 0.9
    assert all(math.isnan(v) for v in cm.regularity([0.0] * 500, FS, 1.1))


def test_yaw_rate_and_the_fraction_turning():
    rms, turning = cm.yaw_stats([0.0] * 50 + [30.0] * 50)
    assert rms == pytest.approx(math.sqrt(450.0)) and turning == pytest.approx(0.5)
    assert all(math.isnan(v) for v in cm.yaw_stats([]))


def test_a_window_gives_every_metric_and_the_window_length():
    ap, ml = walk(20.0, 1.1)
    m = cm.chassis_metrics(ap, ml, [2.0] * len(ap), FS)
    assert m.window_s == pytest.approx(20.0) and m.stride_time == pytest.approx(1.1, abs=0.05)
    assert m.rms_ap > m.rms_ml > 0 and m.hr_ap > 1.0 and m.yaw_rate_rms == pytest.approx(2.0) and m.turning_fraction == 0.0
    short = cm.chassis_metrics([0.0] * 3, [0.0] * 3, [0.0] * 3, FS)
    assert math.isnan(short.rms_ap) and math.isnan(short.stride_time)


def test_resampling_makes_an_irregular_series_uniform():
    t, v = cm.resample_uniform([0.0, 0.1, 0.5, 1.0], [0.0, 1.0, 5.0, 10.0], 10.0)
    assert t[:3] == pytest.approx([0.0, 0.1, 0.2]) and v[:3] == pytest.approx([0.0, 1.0, 2.0]) and len(t) == 10


def test_rotating_the_imu_into_the_walker_frame():
    rotate, rotation_matrix = cm.rotate, cm.rotation_matrix
    half = math.sqrt(0.5)
    r = rotation_matrix((0.0, 0.0, half, half))                       # +90 degrees about z
    assert rotate(r, (1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0), abs=1e-12)
    assert rotate(rotation_matrix((0.0, 0.0, 0.0, 1.0)), (1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0)
