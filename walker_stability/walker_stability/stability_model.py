"""Stability of the user from what the walker measures: where its four contacts are and how much each carries.
Pure python (no ROS); the node (scripts/user_stability.py) only feeds it, in data-time order.

What it computes at an instant, in the frame of the step positions:

  * the centre of pressure: the barycentre of the two feet and the two handles weighted by their loads (what
    walker_centroid_support publishes). It is where the contacts push, not where the user's mass is: while walking it
    swings from foot to foot, two to three times wider than the centre of mass does;
  * an estimate of the centre of mass, the centre of pressure low-passed with time constant `com_tau_s` (the centre of
    mass is, for an inverted pendulum, what the centre of pressure averages to). Against the mocap's De Leva centre
    of mass it halves the error of the raw centre of pressure and reproduces its spread (see mocap_stability);
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


class StabilityModel:

    def __init__(self, min_foot_load_kg=5.0, com_tau_s=0.5, margin_ref_m=0.10, foot_heel_m=0.08, foot_toe_m=0.18,
                 foot_half_width_m=0.05):
        self.min_foot_load_kg = min_foot_load_kg
        self.com_tau_s = com_tau_s
        self.margin_ref_m = margin_ref_m
        self.sole = (foot_heel_m, foot_toe_m, foot_half_width_m)
        self._feet = {}
        self._handles = {}
        self._com = None
        self._t = None

    def set_foot(self, side, x, y, load_kg):
        self._feet[side] = sp.Contact(sp.FOOT, x, y, load_kg)

    def set_handle(self, side, x, y, load_kg):
        self._handles[side] = sp.Contact(sp.HANDLE, x, y, load_kg)

    @property
    def ready(self):
        """All four contacts have been seen at least once."""
        return len(self._feet) == 2 and len(self._handles) == 2

    def evaluate(self, t_s):
        """The state as of data time `t_s` (seconds, not earlier than the previous call), or None while there is
        nothing to evaluate: a contact not seen yet, or no load at all."""
        if not self.ready:
            return None
        contacts = list(self._feet.values()) + list(self._handles.values())
        cop = sp.centre_of_pressure(contacts)
        if cop is None:
            return None
        if self._com is None or self.com_tau_s <= 0.0:
            self._com = cop
        elif t_s > self._t:
            a = (t_s - self._t) / (self.com_tau_s + (t_s - self._t))
            self._com = (self._com[0] + a * (cop[0] - self._com[0]), self._com[1] + a * (cop[1] - self._com[1]))
        self._t = t_s if self._t is None else max(self._t, t_s)
        points = [p for c in self._feet.values() if c.load >= self.min_foot_load_kg
                  for p in sp.footprint(c.x, c.y, *self.sole)]
        points += [(c.x, c.y) for c in self._handles.values()]
        stood_on = sum(c.load >= self.min_foot_load_kg for c in self._feet.values())
        return Evaluation(cop, self._com, stood_on + len(self._handles), sp.assess(points, self._com, self.margin_ref_m))
