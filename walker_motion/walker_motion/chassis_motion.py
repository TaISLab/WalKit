"""Motion of the walker's chassis from its acceleration: RMS, harmonic ratio, regularity, and how much it turns. Pure python (no ROS).

The walker computes it from its IMU (scripts/chassis_motion.py) and the mocap from the rigid body of the walker (mocap_motion), and both
use these functions, so they differ only in the signals they feed in. (It replaces mocap_eval/imu_metrics.py of walker_mocap_eval, whose
formulas it keeps; the stride time that the harmonic ratio needs is now found in the signals themselves, not taken from the gait events
of another module, so the metric needs nothing but the IMU.)

The IMU is on the chassis, not on the user's trunk, where these indices are normally measured: the values are not comparable with the norms
of the literature (trunk), only the walker against the mocap with the same definition. The vertical axis is left out: it carries gravity and
the chassis hardly moves along it.

Over a window of the accelerations along the walker's x (AP) and y (ML), uniformly sampled at fs:
  rms          standard deviation of the signal (mean removed)
  stride time  the lag of the highest autocorrelation peak of AP + ML between 0.8 and 2.4 s (needs a peak above 0.1)
  harmonic ratio  Moe-Nilssen 1998: amplitudes at the first 10 multiples of 1/stride; AP even/odd, ML odd/even
  step / stride regularity  autocorrelation of AP near stride/2 and near stride (Moe-Nilssen & Helbostad 2004)
  yaw rate     RMS of the angular velocity about the vertical and the fraction of the window above 15 deg/s
"""
import math
from typing import NamedTuple

STRIDE_RANGE_S = (0.8, 2.4)
MIN_STRIDE_PEAK = 0.1
TURNING_DEG_S = 15.0
MIN_SAMPLES = 8


class Metrics(NamedTuple):
    window_s: float
    rms_ap: float
    rms_ml: float
    hr_ap: float
    hr_ml: float
    step_regularity: float
    stride_regularity: float
    stride_time: float
    yaw_rate_rms: float
    turning_fraction: float


def _mean(values):
    return sum(values) / len(values)


def rms(values):
    """Standard deviation of the values (mean removed); NaN for none."""
    vals = [v for v in values if not math.isnan(v)]
    if not vals:
        return math.nan
    m = _mean(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / len(vals))


def resample_uniform(t_s, values, fs):
    """(t, values) of irregular samples -> (t, values) on a uniform grid of fs Hz by linear interpolation."""
    if len(t_s) < 2:
        return [], []
    t0, t1 = t_s[0], t_s[-1]
    n = max(2, int((t1 - t0) * fs))
    out_t, out_v, j = [], [], 0
    for i in range(n):
        t = t0 + i / fs
        while j < len(t_s) - 2 and t_s[j + 1] < t:
            j += 1
        ta, tb = t_s[j], t_s[j + 1]
        frac = 0.0 if tb <= ta else (t - ta) / (tb - ta)
        out_t.append(t)
        out_v.append(values[j] + frac * (values[j + 1] - values[j]))
    return out_t, out_v


def autocorrelation(values, max_lag):
    """Normalised autocorrelation (1 at lag 0) at lags 0..max_lag; None if the signal has no variance."""
    n = len(values)
    m = _mean(values)
    sig = [v - m for v in values]
    c0 = sum(s * s for s in sig)
    if c0 < 1e-12:
        return None
    return [sum(sig[i] * sig[i + lag] for i in range(n - lag)) / c0 for lag in range(min(max_lag, n - 1) + 1)]


def stride_time(ap, ml, fs, search_s=STRIDE_RANGE_S):
    """Stride time (s) from the autocorrelation of AP + ML: the highest local maximum between search_s[0] and search_s[1], NaN if none
    rises above MIN_STRIDE_PEAK."""
    lo, hi = int(round(search_s[0] * fs)), int(round(search_s[1] * fs))
    if len(ap) < 2 * hi:
        return math.nan
    a, b = autocorrelation(ap, hi + 1), autocorrelation(ml, hi + 1)
    if a is None and b is None:
        return math.nan
    acf = [(a[i] if a else 0.0) + (b[i] if b else 0.0) for i in range(hi + 2)]
    scale = 2.0 if (a and b) else 1.0
    best, best_lag = MIN_STRIDE_PEAK * scale, None
    for lag in range(max(lo, 1), hi + 1):
        if acf[lag] > acf[lag - 1] and acf[lag] >= acf[lag + 1] and acf[lag] > best:
            best, best_lag = acf[lag], lag
    return best_lag / fs if best_lag else math.nan


def harmonic_ratio(values, fs, stride_s, n_harmonics=10, even_over_odd=True):
    """Harmonic ratio (Moe-Nilssen 1998): direct DFT at the first n_harmonics multiples of 1/stride_s; even over odd harmonics (AP,
    vertical) or odd over even (ML, whose period is the step). NaN without a stride time or with too few samples."""
    if stride_s is None or math.isnan(stride_s) or stride_s <= 0 or len(values) < MIN_SAMPLES or fs <= 0:
        return math.nan
    m = _mean(values)
    sig = [v - m for v in values]
    amps = []
    for h in range(1, n_harmonics + 1):
        f = h / stride_s
        if f >= fs / 2:
            amps.append(0.0)
            continue
        w = 2 * math.pi * f / fs
        re = sum(s * math.cos(w * i) for i, s in enumerate(sig))
        im = sum(s * math.sin(w * i) for i, s in enumerate(sig))
        amps.append(math.hypot(re, im))
    odd, even = sum(amps[0::2]), sum(amps[1::2])
    if even_over_odd:
        return even / odd if odd > 1e-9 else math.nan
    return odd / even if even > 1e-9 else math.nan


def regularity(values, fs, stride_s, search_frac=0.3):
    """(step regularity, stride regularity): the autocorrelation maximum within +/-search_frac of the lags stride/2 and stride."""
    if stride_s is None or math.isnan(stride_s) or stride_s <= 0:
        return math.nan, math.nan
    acf = autocorrelation(values, int(round(stride_s * fs * (1 + search_frac))) + 2)
    if acf is None:
        return math.nan, math.nan

    def near(expected_s):
        lag0 = int(round(expected_s * fs))
        window = max(1, int(lag0 * search_frac))
        lags = range(max(1, lag0 - window), min(len(acf) - 1, lag0 + window) + 1)
        return max((acf[lag] for lag in lags), default=math.nan)

    return near(stride_s / 2.0), near(stride_s)


def yaw_stats(yaw_rate_deg_s):
    """(RMS of the yaw rate, fraction of samples faster than TURNING_DEG_S), both NaN for none."""
    if not yaw_rate_deg_s:
        return math.nan, math.nan
    return (math.sqrt(sum(w * w for w in yaw_rate_deg_s) / len(yaw_rate_deg_s)),
            sum(abs(w) > TURNING_DEG_S for w in yaw_rate_deg_s) / len(yaw_rate_deg_s))


def chassis_metrics(ap, ml, yaw_rate_deg_s, fs):
    """Metrics of one window from the AP and ML accelerations (m/s2) and the yaw rate (deg/s) sampled uniformly at fs."""
    window_s = len(ap) / fs
    if len(ap) < MIN_SAMPLES or len(ml) != len(ap):
        return Metrics(window_s, *([math.nan] * 9))
    stride = stride_time(ap, ml, fs)
    step_reg, stride_reg = regularity(ap, fs, stride)
    yaw_rms, turning = yaw_stats(yaw_rate_deg_s)
    return Metrics(window_s, rms(ap), rms(ml), harmonic_ratio(ap, fs, stride, even_over_odd=True),
                   harmonic_ratio(ml, fs, stride, even_over_odd=False), step_reg, stride_reg, stride, yaw_rms, turning)


def rotation_matrix(q):
    """3x3 rotation matrix of the unit quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def rotate(r, v):
    return tuple(sum(r[i][j] * v[j] for j in range(3)) for i in range(3))
