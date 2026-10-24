"""Stability of the user from what the walker measures: where its four contacts are and how much each carries.
Pure python (no ROS); the node (scripts/user_stability.py) only feeds it, in data-time order.

What it computes at an instant, in the frame of the step positions:

  * the centre of pressure: the barycentre of the two feet and the two handles weighted by their loads (what
    walker_centroid_support publishes), with each loaded foot pushing at the centre of its sole, not at the point
    where the legs are tracked. It is where the contacts push, not where the user's mass is: while walking it
    swings from foot to foot, two to three times wider than the centre of mass does;
  * an estimate of the centre of mass: the centre of pressure low-passed with time constant `com_tau_s` (the centre of
    mass is, for an inverted pendulum, what the centre of pressure averages to), plus `com_moment_kgm / weight`
    metres forward. Against the mocap's De Leva centre of mass, over the 47 definitive bags, the filter takes the
    error of the raw centre of pressure (RMS without bias) from 8.9 to 5.5 cm and reproduces its spread; and the
    centre of pressure lies 12 cm behind it on average, a bias that is, to within 2 cm, 8.75 kg*m / weight (4 cm of it
    is the sole centre, the rest is `com_moment_kgm`): a moment that does not depend on the user, whose origin is
    not identified (see the README);
  * the support polygon: the convex hull of the soles of the feet that carry at least `min_foot_load_kg` and of both
    handles, which count as support wherever the hands are even when they carry little (the user walks with them
    on the handlebar; a hand that rests there is a contact the user can lean on);
  * the margin of the centre-of-mass estimate in that polygon (support_polygon.margin) and the 0..1 index
    made from it.

Every number depends only on the data and its stamps: the filter advances with the stamps of the ticks it is
asked for, not with the clock.
"""
from typing import NamedTuple

from walker_stability import support_polygon as sp

LEFT, RIGHT = 'left', 'right'


class Evaluation(NamedTuple):
    cop: tuple                  # (x, y) centre of pressure
    com: tuple                  # (x, y) estimate of the centre of mass (cop low-passed)
    contacts: int               # contacts in the polygon: feet carrying load, plus the two handles
    assessment: sp.Assessment   # the polygon's area, the margin of `com` in it and the index
    polygon: list               # the support polygon: (x, y) vertices of the hull, counter-clockwise


class StabilityModel:

    def __init__(self, min_foot_load_kg=5.0, com_tau_s=0.5, margin_ref_m=0.10, foot_heel_m=0.08, foot_toe_m=0.18,
                 foot_half_width_m=0.05, com_moment_kgm=6.0):
        self.min_foot_load_kg = min_foot_load_kg
        self.com_tau_s = com_tau_s
        self.margin_ref_m = margin_ref_m
        self.sole = (foot_heel_m, foot_toe_m, foot_half_width_m)
        self.foot_cop_dx = (foot_toe_m - foot_heel_m) / 2.0           # a loaded foot pushes at the centre of its sole
        self.com_moment_kgm = com_moment_kgm
        self.weight_kg = None
        self._feet = {}
        self._handles = {}
        self._com = None
        self._t = None

    def set_foot(self, side, x, y, load_kg):
        self._feet[side] = sp.Contact(sp.FOOT, x, y, load_kg)

    def set_handle(self, side, x, y, load_kg):
        self._handles[side] = sp.Contact(sp.HANDLE, x, y, load_kg)

    def set_weight(self, weight_kg):
        """The user's weight, for the moment correction of the centre of mass (none without it)."""
        self.weight_kg = weight_kg if weight_kg and weight_kg > 0 else None

    @property
    def ready(self):
        """All four contacts have been seen at least once."""
        return len(self._feet) == 2 and len(self._handles) == 2

    def evaluate(self, t_s):
        """The state as of data time `t_s` (seconds, not earlier than the previous call), or None while there is
        nothing to evaluate: a contact not seen yet, or no load at all."""
        if not self.ready:
            return None
        pushing = [c._replace(x=c.x + self.foot_cop_dx) for c in self._feet.values()] + list(self._handles.values())
        cop = sp.centre_of_pressure(pushing)
        if cop is None:
            return None
        if self._com is None or self.com_tau_s <= 0.0:
            self._com = cop
        elif t_s > self._t:
            a = (t_s - self._t) / (self.com_tau_s + (t_s - self._t))
            self._com = (self._com[0] + a * (cop[0] - self._com[0]), self._com[1] + a * (cop[1] - self._com[1]))
        self._t = t_s if self._t is None else max(self._t, t_s)
        com = self._com if self.weight_kg is None else (self._com[0] + self.com_moment_kgm / self.weight_kg, self._com[1])
        points = [p for c in self._feet.values() if c.load >= self.min_foot_load_kg
                  for p in sp.footprint(c.x, c.y, *self.sole)]
        points += [(c.x, c.y) for c in self._handles.values()]
        stood_on = sum(c.load >= self.min_foot_load_kg for c in self._feet.values())
        return Evaluation(cop, com, stood_on + len(self._handles), sp.assess(points, com, self.margin_ref_m), sp.convex_hull(points))
