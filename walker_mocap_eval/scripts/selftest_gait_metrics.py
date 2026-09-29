#!/usr/bin/env python3
"""Auto-test con datos sinteticos (sin bags, sin ROS) para gait_metrics.py y
contact_events.py: no reemplaza la validacion contra bags reales, pero
garantiza que el puerto del algoritmo de GaitMonitorSp a funciones puras es
correcto antes de fiarse de el sobre datos de mocap.

Uso:
    python3 selftest_gait_metrics.py
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mocap_eval import contact_events, gait_metrics  # noqa: E402


def approx(a, b, tol=1e-6):
    return math.isfinite(a) and math.isfinite(b) and abs(a - b) <= tol


def test_gait_metrics_matches_hand_computation():
    # Marcha alterna sencilla en linea recta, paso de 0.6m cada 0.5s:
    # L en x=0, R en x=0.6, L en x=1.2, R en x=1.8, L en x=2.4, R en x=3.0,
    # L en x=3.6 -- 4 apoyos izquierdos y 3 derechos (minimo >2 en ambos
    # pies, condicion de guarda de GaitMonitorSp.timer_callback()).
    ns = int(1e9)
    events = [
        (0 * ns, "left", (0.0, 0.0)),
        (int(0.5 * ns), "right", (0.6, 0.0)),
        (int(1.0 * ns), "left", (1.2, 0.0)),
        (int(1.5 * ns), "right", (1.8, 0.0)),
        (int(2.0 * ns), "left", (2.4, 0.0)),
        (int(2.5 * ns), "right", (3.0, 0.0)),
        (int(3.0 * ns), "left", (3.6, 0.0)),
    ]
    left_log, right_log = gait_metrics.process_events(events)

    assert len(left_log.events) == 4, left_log.events
    assert len(right_log.events) == 3, right_log.events

    # stride length/time: 1.2m cada 1.0s, en ambos pies.
    assert all(approx(v, 1.2) for v in left_log.stride_lengths), left_log.stride_lengths
    assert all(approx(v, 1.0) for v in left_log.stride_times), left_log.stride_times
    assert all(approx(v, 1.2) for v in right_log.stride_lengths), right_log.stride_lengths
    assert all(approx(v, 1.0) for v in right_log.stride_times), right_log.stride_times

    # step length/time: siempre 0.6m / 0.5s entre pie contrario y el actual.
    assert all(approx(v, 0.6) for v in left_log.step_lengths + right_log.step_lengths)
    assert all(approx(v, 0.5) for v in left_log.step_times + right_log.step_times)

    distance_m = 3.6  # el andador avanza en linea recta lo mismo que el ultimo evento
    summary = gait_metrics.summarize(left_log, right_log, distance_m)
    assert summary is not None

    assert approx(summary["left_sdt"], 1.0)
    assert approx(summary["left_sdl"], 1.2)
    assert approx(summary["right_sdt"], 1.0)
    assert approx(summary["right_sdl"], 1.2)
    assert approx(summary["left_spt"], 0.5)
    assert approx(summary["left_spl"], 0.6)
    assert summary["nos"] == 7
    assert approx(summary["tr_s"], 3.0)
    assert approx(summary["cad"], 60.0 * 7 / 3.0)
    assert approx(summary["wv"], 3.6 / 3.0)
    assert approx(summary["avg_step_speed_all"], 0.6 / 0.5)

    print("test_gait_metrics_matches_hand_computation: OK")


def test_summarize_returns_none_when_not_enough_steps():
    ns = int(1e9)
    events = [
        (0 * ns, "left", (0.0, 0.0)),
        (int(0.5 * ns), "right", (0.6, 0.0)),
    ]
    left_log, right_log = gait_metrics.process_events(events)
    assert gait_metrics.summarize(left_log, right_log, 1.0) is None
    print("test_summarize_returns_none_when_not_enough_steps: OK")


def test_detect_stance_events_picks_speed_minima():
    # Trayectoria sintetica en X: quieto en x=0 (apoyo), sube rapido a x=1
    # (balanceo), quieto en x=1 (apoyo). Muestreo cada 0.05s.
    ns = int(1e9)
    track = []
    t = 0.0
    # apoyo en x=0 durante 0.4s
    while t <= 0.4 + 1e-9:
        track.append((int(t * ns), (0.0, 0.0, 0.0)))
        t += 0.05
    # balanceo lineal de x=0 a x=1 en 0.2s (rapido)
    t0 = t
    while t <= t0 + 0.2 + 1e-9:
        frac = (t - t0) / 0.2
        track.append((int(t * ns), (frac * 1.0, 0.0, 0.0)))
        t += 0.02
    # apoyo en x=1 durante 0.4s
    t1 = t
    while t <= t1 + 0.4 + 1e-9:
        track.append((int(t * ns), (1.0, 0.0, 0.0)))
        t += 0.05

    events = contact_events.detect_stance_events(track, speed_threshold=0.25, min_gap_s=0.1)
    assert len(events) == 2, events
    (_, p0), (_, p1) = events
    assert approx(p0[0], 0.0, tol=1e-3), p0
    assert approx(p1[0], 1.0, tol=1e-3), p1
    print("test_detect_stance_events_picks_speed_minima: OK")


if __name__ == "__main__":
    test_gait_metrics_matches_hand_computation()
    test_summarize_returns_none_when_not_enough_steps()
    test_detect_stance_events_picks_speed_minima()
    print("\nTodos los selftests OK")
