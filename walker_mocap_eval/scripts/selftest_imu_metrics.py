#!/usr/bin/env python3
"""Auto-test con señales sinteticas (sin bags) para imu_metrics.py --
valida RMS, Harmonic Ratio, regularidad por autocorrelacion y la
aceleracion AP/ML estimada por mocap antes de fiarse de ellas sobre datos
reales.

Uso:
    python3 selftest_imu_metrics.py
"""
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mocap_eval import imu_metrics  # noqa: E402


def approx(a, b, tol=1e-3):
    return math.isfinite(a) and math.isfinite(b) and abs(a - b) <= tol


def test_rms_sine():
    fs, f, amp, dur = 100.0, 2.0, 3.0, 10.0
    sig = [amp * math.sin(2 * math.pi * f * i / fs) for i in range(int(fs * dur))]
    assert approx(imu_metrics.rms(sig), amp / math.sqrt(2), tol=0.02), imu_metrics.rms(sig)
    print("test_rms_sine: OK")


def test_harmonic_ratio_pure_harmonics():
    fs, dur, stride_s = 100.0, 20.0, 1.0
    f0 = 1.0 / stride_s
    n = int(fs * dur)
    # pizca de ruido de banda ancha (no un tono puro, para no caer por
    # casualidad en un numero entero de ciclos dentro de la ventana y
    # quedar igualmente "ortogonal" a los bins de armonicos) para que los
    # bins "vacios" no den exactamente 0 (division por ~0 -> NaN por diseño)
    rng = random.Random(42)
    noise = [1e-4 * rng.gauss(0, 1) for _ in range(n)]
    # señal con solo el 2º armonico (par) -> HR even_over_odd debe ser grande
    sig_even = [math.sin(2 * math.pi * (2 * f0) * i / fs) + noise[i] for i in range(n)]
    hr_eo = imu_metrics.harmonic_ratio(sig_even, fs, stride_s, kind="even_over_odd")
    assert hr_eo > 20, hr_eo
    # señal con solo el 1er armonico (impar) -> HR even_over_odd debe ser pequeño
    sig_odd = [math.sin(2 * math.pi * f0 * i / fs) + noise[i] for i in range(n)]
    hr_eo2 = imu_metrics.harmonic_ratio(sig_odd, fs, stride_s, kind="even_over_odd")
    assert hr_eo2 < 0.05, hr_eo2
    # la misma señal impar con kind="odd_over_even" (ML) debe ser grande
    hr_oe = imu_metrics.harmonic_ratio(sig_odd, fs, stride_s, kind="odd_over_even")
    assert hr_oe > 20, hr_oe
    print("test_harmonic_ratio_pure_harmonics: OK")


def test_autocorr_regularity_periodic_signal():
    # dur grande frente al periodo: la normalizacion de ac_at_lag() usa N
    # muestras en el denominador pero solo N-lag en el numerador (sesgo
    # deliberado del metodo, ver imu_metrics.py), así que para un lag no
    # despreciable frente a N el valor teorico maximo es (N-lag)/N < 1, no
    # exactamente 1 -- con dur>>periodo ese sesgo se vuelve despreciable.
    fs, dur, period = 100.0, 100.0, 0.8
    n = int(fs * dur)
    sig = [math.sin(2 * math.pi * i / fs / period) for i in range(n)]
    ad1, ad2 = imu_metrics.autocorr_regularity(sig, fs, step_time_s=period, stride_time_s=2 * period)
    assert approx(ad1, 1.0, tol=0.02), ad1
    assert approx(ad2, 1.0, tol=0.02), ad2
    print("test_autocorr_regularity_periodic_signal: OK")


def test_autocorr_regularity_nan_without_reference():
    ad1, ad2 = imu_metrics.autocorr_regularity([1.0] * 20, 10.0, step_time_s=None, stride_time_s=float("nan"))
    assert math.isnan(ad1) and math.isnan(ad2)
    print("test_autocorr_regularity_nan_without_reference: OK")


def test_walker_accel_zero_for_constant_velocity():
    ns = int(1e9)
    track = [(i * int(0.01 * ns), (i * 0.01, 0.0, 0.0)) for i in range(200)]  # 1 m/s recto, yaw=0
    accel = imu_metrics.walker_accel_ap_ml(track)
    assert len(accel) > 100
    for _, ap, ml in accel[5:-5]:
        assert approx(ap, 0.0, tol=1e-6), ap
        assert approx(ml, 0.0, tol=1e-6), ml
    print("test_walker_accel_zero_for_constant_velocity: OK")


def test_walker_accel_constant_forward_accel():
    ns = int(1e9)
    dt = 0.01
    a_true = 0.5
    yaw = math.pi / 4  # 45 grados: comprobar que la proyeccion AP/ML es correcta
    xs, ys, t = [], [], 0.0
    x = y = v = 0.0
    for i in range(300):
        xs.append(x); ys.append(y)
        t += dt
        v += a_true * dt
        x += v * dt * math.cos(yaw)
        y += v * dt * math.sin(yaw)
    track = [(int(i * dt * ns), (xs[i], ys[i], yaw)) for i in range(len(xs))]
    accel = imu_metrics.walker_accel_ap_ml(track)
    mid = accel[len(accel) // 2]
    assert approx(mid[1], a_true, tol=0.05), mid
    assert approx(mid[2], 0.0, tol=0.05), mid
    print("test_walker_accel_constant_forward_accel: OK")


if __name__ == "__main__":
    test_rms_sine()
    test_harmonic_ratio_pure_harmonics()
    test_autocorr_regularity_periodic_signal()
    test_autocorr_regularity_nan_without_reference()
    test_walker_accel_zero_for_constant_velocity()
    test_walker_accel_constant_forward_accel()
    print("\nTodos los selftests OK")
