"""Hands the messages of several streams to a handler in data-time order (header stamp), not in the
order they happened to be delivered.

ROS delivers each topic in order, but gives no ordering between topics: which of two messages of
different topics a node sees first depends on DDS, thread scheduling and, replaying a bag, on
--rate. A node that reacts to messages as they arrive (or to a timer) therefore computes something
different on every run. This class makes the result depend only on the messages' stamps: every
pushed message is buffered and released, sorted by (stamp, stream index, arrival order within the
stream), once no stream can still deliver anything older -- i.e. when the "watermark", the smallest
of the streams' last stamps, has passed it (the usual event-time notion of stream processors). The
price is latency: a message waits for the slowest stream, one of its periods at most.

A stream whose stamps can repeat (several messages with the same stamp, e.g. a state republished as
other inputs change) is declared in `repeating`: more messages with its last stamp may still come, so
that stamp only counts as complete once the stream has moved on to a later one. Otherwise the order
in which the repeated messages interleave with other streams' messages of the same stamp would depend
on arrival order.

A stream that only feeds an accumulator (a running sum does not care about the order it is added in)
can be declared in `late_ok`: what it delivers after the watermark has passed it is not dropped but
handed to the handler at once (counted in `out_of_order`), so a source whose stamps lag behind the
others' by more than max_wait_ns (a different clock) still gets all its data counted, only its
position in the sequence is approximate.

A stream that stops delivering would hold everything back forever, so the watermark is never
allowed to lag the newest stamp seen by more than max_wait_ns: after that the stalled stream is
given up on, and whatever it delivers later, if older than what was already released, is dropped
(and counted in `late`). With max_wait_ns <= 0 the wait is unbounded.

Same semantics as walker_loads/stamp_sequencer.h (used by partial_loads).
"""
import heapq
import itertools


def stamp_ns(stamp):
    """builtin_interfaces/Time -> integer nanoseconds."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class StampSequencer:

    def __init__(self, n_streams, handler, max_wait_ns=1_000_000_000, max_buffer=100_000, repeating=(), late_ok=()):
        """handler(stream, stamp_ns, payload) is called in data-time order."""
        self._repeating = [s in repeating for s in range(n_streams)]
        self._late_ok = frozenset(late_ok)
        self._last = [None] * n_streams
        self._max_seen = None
        self._released = None
        self._heap = []
        self._seq = itertools.count()
        self._max_wait_ns = max_wait_ns
        self._max_buffer = max_buffer
        self._handler = handler
        self.late = 0
        self.out_of_order = 0
        self.overflow = 0

    @property
    def pending(self):
        return len(self._heap)

    def push(self, stream, stamp, payload):
        """Buffers a message; returns False if it was dropped as late (older than what was already
        released). Nothing reaches the handler until release() is called."""
        if self._released is not None and stamp < self._released:
            if stream in self._late_ok:
                self.out_of_order += 1
                self._handler(stream, stamp, payload)
                return True
            self.late += 1
            return False
        last = self._last[stream]
        self._last[stream] = stamp if last is None else max(last, stamp)
        self._max_seen = stamp if self._max_seen is None else max(self._max_seen, stamp)
        heapq.heappush(self._heap, (stamp, stream, next(self._seq), payload))
        if self._max_buffer > 0 and len(self._heap) > self._max_buffer:
            # the oldest by stamp goes
            heapq.heappop(self._heap)
            self.overflow += 1
        return True

    def release(self):
        """Hands the handler everything the watermark has passed, oldest first."""
        if not self._heap:
            return
        if any(last is None for last in self._last):
            wm = None
        else:
            wm = min(last - 1 if rep else last for last, rep in zip(self._last, self._repeating))
        if self._max_wait_ns > 0:
            forced = self._max_seen - self._max_wait_ns
            wm = forced if wm is None else max(wm, forced)
        if wm is None:
            return
        while self._heap and self._heap[0][0] <= wm:
            stamp, stream, _seq, payload = heapq.heappop(self._heap)
            self._released = stamp
            self._handler(stream, stamp, payload)


class Ticks:
    """Fixed-period grid on data time: what a timer is when time is the stamps of the data, not the clock.

    Feed it the stamps in order; `until(stamp)` yields the grid times that fall before `stamp`, i.e.
    the instants at which, by then, everything up to them has been seen. The grid starts one period
    after the first stamp. A node that publishes a state sampled every `period` seconds does so, in
    data time, with: `for tick in ticks.until(stamp): publish(state)` before it applies the event."""

    def __init__(self, period_ns):
        self._period = period_ns
        self._next = None

    def until(self, stamp):
        if self._next is None:
            self._next = stamp + self._period
        while self._next < stamp:
            tick = self._next
            self._next += self._period
            yield tick
