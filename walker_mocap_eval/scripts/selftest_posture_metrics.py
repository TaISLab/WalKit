#!/usr/bin/env python3
"""Auto-test con datos sinteticos (sin bags, sin ROS salvo bag_io.indexed_
nearest/nearest, que no tocan disco) para posture_metrics.py -- mismo
espiritu que selftest_gait_metrics.py: valida la geometria antes de fiarse
de ella sobre datos de mocap reales.

Uso (necesita el entorno ROS sourceado, por el import de bag_io dentro de
step_width_m/lateral_margin_at_event):
    source /opt/ros/jazzy/setup.bash
    python3 selftest_posture_metrics.py
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mocap_eval import posture_metrics  # noqa: E402


def approx(a, b, tol=1e-6):
    return math.isfinite(a) and math.isfinite(b) and abs(a - b) <= tol


def test_trunk_lean_upright_is_zero():
    total, sag, fro = posture_metrics.trunk_lean_at_frame(
        l_shoulder=(0.0, 0.2, 1.4), r_shoulder=(0.0, -0.2, 1.4),
        l_hip=(0.0, 0.15, 0.9), r_hip=(0.0, -0.15, 0.9))
    assert approx(total, 0.0) and approx(sag, 0.0) and approx(fro, 0.0), (total, sag, fro)
    print("test_trunk_lean_upright_is_zero: OK")


def test_trunk_lean_forward_30deg():
    lean_rad = math.radians(30)
    h = 0.5
    dx, dz = h * math.sin(lean_rad), h * math.cos(lean_rad)
    total, sag, fro = posture_metrics.trunk_lean_at_frame(
        l_shoulder=(dx, 0.2, 0.9 + dz), r_shoulder=(dx, -0.2, 0.9 + dz),
        l_hip=(0.0, 0.15, 0.9), r_hip=(0.0, -0.15, 0.9))
    assert approx(total, 30.0, tol=1e-3), total
    assert approx(sag, 30.0, tol=1e-3), sag
    assert approx(fro, 0.0, tol=1e-3), fro
    print("test_trunk_lean_forward_30deg: OK")


def test_double_support_fraction():
    ns = int(1e9)
    # solape [1,2] de un total [0,3] -> 1/3
    dsp = posture_metrics.double_support_fraction([(0, 2 * ns)], [(1 * ns, 3 * ns)])
    assert approx(dsp, 1.0 / 3.0), dsp
    # sin solape
    dsp0 = posture_metrics.double_support_fraction([(0, 1 * ns)], [(2 * ns, 3 * ns)])
    assert approx(dsp0, 0.0), dsp0
    print("test_double_support_fraction: OK")


def test_step_width_yaw_invariant():
    ns = int(1e9)
    # yaw=0: lateral = eje Y directamente
    walker_track = [(0, (0.0, 0.0, 0.0))]
    widths = posture_metrics.step_width_m(
        [(0, (1.0, 0.2, 0.0))], [(0, (1.0, -0.2, 0.0))], walker_track, int(0.1 * ns), int(0.1 * ns))
    assert all(approx(w, 0.4) for _, w in widths), widths

    # yaw=90deg: el eje lateral del andador es ahora -X del mundo
    walker_track2 = [(0, (0.0, 0.0, math.pi / 2))]
    widths2 = posture_metrics.step_width_m(
        [(0, (0.3, 1.0, 0.0))], [(0, (-0.3, 1.0, 0.0))], walker_track2, int(0.1 * ns), int(0.1 * ns))
    assert all(approx(w, 0.6) for _, w in widths2), widths2
    print("test_step_width_yaw_invariant: OK")


def test_lateral_margin_centered_and_edge():
    ns = int(1e9)
    walker_track = [(0, (0.0, 0.0, 0.0))]
    la_track = [(0, (1.0, 0.2, 0.0))]
    ra_track = [(0, (1.0, -0.2, 0.0))]

    hip_centered = [(0, (1.0, 0.0, 0.9))]
    m = posture_metrics.lateral_margin_at_event(
        0, hip_centered, la_track, ra_track, walker_track, int(0.1 * ns), int(0.1 * ns))
    assert approx(m, 0.5), m

    hip_at_left_ankle = [(0, (1.0, 0.2, 0.9))]
    m_edge = posture_metrics.lateral_margin_at_event(
        0, hip_at_left_ankle, la_track, ra_track, walker_track, int(0.1 * ns), int(0.1 * ns))
    assert approx(m_edge, 1.0), m_edge
    print("test_lateral_margin_centered_and_edge: OK")


if __name__ == "__main__":
    test_trunk_lean_upright_is_zero()
    test_trunk_lean_forward_30deg()
    test_double_support_fraction()
    test_step_width_yaw_invariant()
    test_lateral_margin_centered_and_edge()
    print("\nTodos los selftests OK")
