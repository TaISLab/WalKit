"""Raw handle reading -> kg, from the calibration table walker_loads uses (config/handle_calib.yaml: the raw readings of each
handle at known weights). For the arrows of a handle that has no `walker_loads` chain behind it (a bag viewed on its own): the
chain itself (`partial_loads`) interpolates that table with a spline; this is the piecewise-linear version, enough for a picture
(a fraction of a kilogram of difference). Beyond the table's ends the line of the last segment goes on, like a spline would.
Pure python (PyYAML).
"""
import bisect

import yaml


class HandleCalibration:

    def __init__(self, weight_points, left_points, right_points):
        self._weights = [float(w) for w in weight_points]
        self._raw = {'left': [float(v) for v in left_points], 'right': [float(v) for v in right_points]}
        for side, raw in self._raw.items():
            if len(raw) != len(self._weights) or len(raw) < 2 or any(b <= a for a, b in zip(raw, raw[1:])):
                raise ValueError('The %s handle calibration needs as many increasing raw points as weights (at least 2)' % side)

    @classmethod
    def from_yaml(cls, path):
        with open(path, encoding='utf-8') as f:
            params = yaml.safe_load(f)['partial_loads']['ros__parameters']
        return cls(params['weight_points'], params['left_handle_points'], params['right_handle_points'])

    def kg(self, side, raw_value):
        raw = self._raw[side]
        i = min(max(bisect.bisect_right(raw, raw_value) - 1, 0), len(raw) - 2)
        t = (raw_value - raw[i]) / (raw[i + 1] - raw[i])
        return self._weights[i] + t * (self._weights[i + 1] - self._weights[i])
