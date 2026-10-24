"""Gait quality from the stance of each leg: step width, double support, and the symmetry and variability of the step time.
Pure python (no ROS): the walker computes it from its leg loads (scripts/gait_quality.py) and the mocap from the kinematics of
the ankles (mocap_loads), and the two use these functions, so they differ only in how they find the stance, never in what they
do with it.

The input is, for each leg, its *stance onsets* (instant and position in the walker's frame, the moment the foot arrives) and its
*stance intervals* (onset and end). Over a window [t0, t1]:

  step time    from the onset of the opposite leg to this leg's, when that is between 0.2 and 2 s; assigned to this leg
  step time SI |mean_left - mean_right| / (0.5 (mean_left + mean_right)) * 100 (Robinson et al. 1987); needs 2 steps per leg
  step time CV std / mean * 100 of all the step times; needs 4 steps
  step width   mean |y_this - y_other|, y being the lateral position in the walker's frame at the onset, y_other that of the
               opposite leg's onset nearest in time (within 1 s)
  double support the time with both legs in stance over the window

A noisy stance detection shows here more than anywhere: a spurious onset cuts a step in two. The stance of the walker has a refractory
period per leg for that (step_onsets.py); the mocap's has one too.
"""
import math
from typing import NamedTuple

from walker_loads.step_onsets import StepOnsets

MIN_STEP_S, MAX_STEP_S = 0.2, 2.0
MAX_WIDTH_PAIR_S = 1.0


class Onset(NamedTuple):
    t: float        # s
    y: float        # m, lateral position in the walker's frame


class Quality(NamedTuple):
    window_s: float
    steps: int
    step_width: float
    double_support: float
    step_time: float
    step_time_si: float
    step_time_cv: float


def _mean(values):
    return sum(values) / len(values) if values else math.nan


def _std(values):
    m = _mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / len(values))


def step_times(left, right):
    """([left step times], [right step times]) from the onsets of both legs (lists of Onset in time order)."""
    merged = sorted([(o.t, 'l') for o in left] + [(o.t, 'r') for o in right])
    out = {'l': [], 'r': []}
    for (t0, leg0), (t1, leg1) in zip(merged, merged[1:]):
        if leg0 != leg1 and MIN_STEP_S < t1 - t0 < MAX_STEP_S:
            out[leg1].append(t1 - t0)
    return out['l'], out['r']


def step_widths(left, right):
    """Lateral gap at every onset to the nearest onset of the opposite leg."""
    widths = []
    for own, other in ((left, right), (right, left)):
        for o in own:
            if other:
                near = min(other, key=lambda q: abs(q.t - o.t))
                if abs(near.t - o.t) <= MAX_WIDTH_PAIR_S:
                    widths.append(abs(o.y - near.y))
    return widths


def double_support(stance_left, stance_right, t0, t1):
    """Fraction of [t0, t1] with both legs in stance; intervals are (on, off) with off None while the leg is still down."""
    if t1 <= t0:
        return math.nan

    def clipped(intervals):
        return [(max(a, t0), min(t1 if b is None else b, t1)) for a, b in intervals if (t1 if b is None else b) > t0 and a < t1]

    overlap = sum(max(0.0, min(b, d) - max(a, c)) for a, b in clipped(stance_left) for c, d in clipped(stance_right))
    return overlap / (t1 - t0)


def symmetry_index(a, b):
    if math.isnan(a) or math.isnan(b) or a + b <= 0:
        return math.nan
    return abs(a - b) / (0.5 * (a + b)) * 100.0


def quality(left, right, stance_left, stance_right, t0, t1):
    """The Quality of the window [t0, t1] (s), from the onsets and stance intervals of both legs (whole history is fine:
    what falls outside the window is left out)."""
    inside = lambda onsets: [o for o in onsets if t0 <= o.t <= t1]  # noqa: E731
    left, right = inside(left), inside(right)
    tl, tr = step_times(left, right)
    all_steps = tl + tr
    return Quality(
        window_s=t1 - t0, steps=len(left) + len(right), step_width=_mean(step_widths(left, right)),
        double_support=double_support(stance_left, stance_right, t0, t1), step_time=_mean(all_steps),
        step_time_si=symmetry_index(_mean(tl), _mean(tr)) if len(tl) >= 2 and len(tr) >= 2 else math.nan,
        step_time_cv=_std(all_steps) / _mean(all_steps) * 100.0 if len(all_steps) >= 4 else math.nan)


class LegStance:
    """One leg's stance from its load: onsets (with the refractory period of StepOnsets) and stance intervals. A load that comes
    back inside the refractory period is the same stance (the load flickered), so the interval it closed is reopened."""

    def __init__(self, min_interval_s=0.8):
        self._onsets = StepOnsets(int(min_interval_s * 1e9))
        self.min_interval_s = min_interval_s
        self.onsets = []          # [Onset]
        self.intervals = []       # [[on, off or None]]
        self._loaded = False

    def update(self, load, t_s, y):
        """Feed the leg's loads in time order (t_s seconds, y the lateral position of the foot in the walker's frame);
        True when this load is a new stance onset."""
        was_loaded = self._loaded
        self._loaded = load > 0
        registered = self._onsets.update(load, int(round(t_s * 1e9)))
        if registered:
            self.onsets.append(Onset(t_s, y))
            self.intervals.append([t_s, None])
        elif self._loaded and not was_loaded and self.intervals and self.intervals[-1][1] is not None:
            self.intervals[-1][1] = None            # flicker inside the refractory period: still the same stance
        elif not self._loaded and was_loaded and self.intervals and self.intervals[-1][1] is None:
            self.intervals[-1][1] = t_s
        return registered

    def stance(self):
        return [(a, b) for a, b in self.intervals]
