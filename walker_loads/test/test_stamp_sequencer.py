import random

from walker_loads.stamp_sequencer import StampSequencer, Ticks, Watermark


def run(arrivals, n_streams=2, max_wait_ns=0, max_buffer=1000, repeating=(), late_ok=()):
    out = []
    seq = StampSequencer(n_streams, lambda s, t, p: out.append((s, t, p)), max_wait_ns, max_buffer, repeating, late_ok)
    for stream, stamp, tag in arrivals:
        seq.push(stream, stamp, tag)
        seq.release()
    return out, seq


def test_released_in_stamp_order_whatever_the_arrival_order():
    msgs = [(0, 10, 'a10'), (0, 30, 'a30'), (1, 20, 'b20'), (1, 40, 'b40')]
    ref = None
    for _ in range(20):
        order = msgs[:]
        random.shuffle(order)
        # keep each stream's own order, as ROS does
        per_stream = {s: [m for m in msgs if m[0] == s] for s in (0, 1)}
        mixed, idx = [], {0: 0, 1: 0}
        for m in order:
            mixed.append(per_stream[m[0]][idx[m[0]]])
            idx[m[0]] += 1
        out, seq = run(mixed)
        tags = [p for _, _, p in out]
        # b40 stays: the watermark is min(last stamps) = 30
        assert tags == ['a10', 'b20', 'a30']
        assert seq.pending == 1
        ref = ref or tags
        assert tags == ref


def test_ties_go_by_stream_then_arrival():
    out, _ = run([(2, 5, 'c'), (1, 5, 'b1'), (1, 5, 'b2'), (0, 5, 'a')], n_streams=3)
    assert [p for _, _, p in out] == ['a', 'b1', 'b2', 'c']


def test_nothing_is_released_until_every_stream_has_spoken():
    out, seq = run([(0, 10, 'a')])
    assert out == []
    seq.push(1, 10, 'b')
    seq.release()
    assert [p for _, _, p in out] == ['a', 'b']


def test_silent_stream_is_given_up_on_after_max_wait():
    out, seq = run([(1, 0, 'b0'), (0, 50, 'a50'), (0, 90, 'a90')], max_wait_ns=100)
    assert [p for _, _, p in out] == ['b0']
    seq.push(0, 150, 'a150')
    seq.release()
    assert [p for _, _, p in out] == ['b0', 'a50']
    assert not seq.push(1, 20, 'late')
    assert seq.late == 1
    assert seq.push(1, 60, 'ok')


def test_overflow_drops_the_oldest():
    _, seq = run([(0, 1, 'a1'), (0, 2, 'a2'), (0, 3, 'a3'), (0, 4, 'a4')], max_buffer=3)
    assert seq.pending == 3
    assert seq.overflow == 1


def test_repeating_stream_stamp_is_complete_only_once_the_stream_moves_on():
    # stream 0 republishes stamp 10 twice; stream 1 has already reached 10. Whatever the arrival order,
    # both a10 messages are processed before b10
    for arrivals in ([(0, 10, 'a1'), (1, 10, 'b'), (0, 10, 'a2'), (0, 20, 'a3')],
                     [(0, 10, 'a1'), (0, 10, 'a2'), (1, 10, 'b'), (0, 20, 'a3')]):
        out, _ = run(arrivals, repeating=(0,))
        assert [p for _, _, p in out] == ['a1', 'a2', 'b']


def test_ticks_fall_on_a_fixed_grid_whatever_the_event_spacing():
    ticks = Ticks(100)
    got = []
    for stamp in (0, 30, 250, 260, 600):
        got.append(list(ticks.until(stamp)))
    # grid: 100, 200, 300, ...; a tick equal to the stamp waits for it, a gap emits every tick in it
    assert got == [[], [], [100, 200], [], [300, 400, 500]]


def test_late_ok_stream_is_handed_over_at_once_instead_of_dropped():
    out, seq = run([(1, 0, 'b0'), (0, 50, 'a50'), (0, 150, 'a150')], max_wait_ns=100, late_ok=(1,))
    assert [p for _, _, p in out] == ['b0', 'a50']
    assert seq.push(1, 20, 'late-but-fine')
    assert [p for _, _, p in out] == ['b0', 'a50', 'late-but-fine']
    assert seq.out_of_order == 1 and seq.late == 0


def test_watermark_is_none_until_every_stream_has_delivered_then_the_oldest_newest_stamp():
    wm = Watermark(3)
    assert [wm.update(0, 50), wm.update(1, 70)] == [None, None]
    assert wm.update(2, 20) == 20
    assert wm.update(0, 60) == 20              # a faster stream running on does not move it
    assert wm.update(2, 65) == 60
    assert wm.update(1, 40) == 60              # a stamp older than the stream's newest changes nothing
    assert wm.update(2, 90) == 60


def test_watermark_gives_up_on_a_silent_stream_after_max_wait():
    wm = Watermark(2, max_wait_ns=100)
    wm.update(0, 10)
    assert wm.update(1, 10) == 10
    assert [wm.update(0, t) for t in (50, 100, 150)] == [10, 10, 50]       # stream 1 stays at 10: after 100 it is let go
    assert wm.update(1, 20) == 50                                        # and what it delivers late does not take it back
    unbounded = Watermark(2)
    unbounded.update(0, 10)
    unbounded.update(1, 10)
    assert unbounded.update(0, 10_000) == 10
