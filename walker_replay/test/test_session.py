import time

from walker_replay import session


def test_stop_reaches_the_children_of_the_process():
    proc = session.start("bash -c 'sleep 60 & wait'", setups=())
    assert session.wait_alive(proc, 0.5)
    start = time.time()
    session.stop(proc, timeout=3)
    assert proc.poll() is not None
    assert time.time() - start < 6


def test_stop_of_a_finished_process_is_harmless():
    proc = session.start('true', setups=())
    proc.wait()
    session.stop(proc)
    session.stop(None)
