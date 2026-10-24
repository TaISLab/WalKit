"""Step loads of one leg: the instants at which its load goes from 0 to above 0, with a refractory period.

Near 0 the load of a leg flickers (0, 0.4, 0, 0.6 kg in 100 ms) and every 0 -> >0 transition would count as a step.
A leg does not step twice in a third of a second (at 100-120 steps/min each leg steps about once a second), so after
a step is registered the leg's next transitions are ignored for `min_interval_ns`. The period runs from the last
*registered* step, not from the last transition, so a long flicker cannot keep extending it. The two legs have one
instance each: the other leg's step may follow closely (double support).
"""


class StepOnsets:

    def __init__(self, min_interval_ns=350_000_000):
        self.min_interval_ns = min_interval_ns
        self._loaded = False
        self._last_ns = None
        self.ignored = 0          # transitions inside the refractory period

    def update(self, load, stamp_ns):
        """Feed the leg's loads in stamp order; True when this one is a step load (a registered 0 -> >0 transition)."""
        if load <= 0:
            self._loaded = False
            return False
        was_loaded, self._loaded = self._loaded, True
        if was_loaded:
            return False
        if self._last_ns is not None and stamp_ns - self._last_ns < self.min_interval_ns:
            self.ignored += 1
            return False
        self._last_ns = stamp_ns
        return True
