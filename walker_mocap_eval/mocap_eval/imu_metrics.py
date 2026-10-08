#!/usr/bin/env python3
"""Metricas de estabilidad derivadas de señales de aceleracion -- punto 1 de
la ampliacion del informe: Harmonic Ratio (HR), RMS de aceleracion y
regularidad de paso/zancada por autocorrelacion, sobre dos fuentes:

  - IMU real del andador (/bwt901cl/imu o /imu_filtered, sensor_msgs/Imu,
    grabado en todos los bags -- no hace falta reproducir nada).
  - Equivalente estimado por mocap: doble derivada numerica de la posicion
    del rigid body del andador, proyectada a los ejes antero-posterior
    (AP, direccion de avance) y medio-lateral (ML) del propio andador segun
    su yaw en cada instante.

Metodologia estandar de la literatura de marcha con IMU de tronco (ver
referencias en el informe): Moe-Nilssen 1998 (Harmonic Ratio), Moe-Nilssen
& Helbostad 2004 (regularidad de paso/zancada por autocorrelacion). Aqui el
IMU esta montado en el chasis del andador, no en el tronco del usuario --
se documenta como aproximacion: el chasis oscila con el patron de marcha
transmitido a traves del propio andador, no es una medida directa del
tronco. No hay componente vertical mocap fiable (el rigid body del andador
apenas se desplaza en z); la V de mocap se devuelve como NaN.
"""
import math
import statistics


def rms(values):
    vals = [v for v in values if not (isinstance(v, float) and math.isnan(v))]
    if not vals:
        return float("nan")
    mean = statistics.mean(vals)
    return math.sqrt(statistics.mean((v - mean) ** 2 for v in vals))


def resample_uniform(t_s, values, fs):
    """Interpola (t_s, values), de muestreo posiblemente irregular, a una
    rejilla uniforme a fs Hz (interpolacion lineal) -- necesario para que
    harmonic_ratio()/autocorr_regularity() trabajen con un periodo de
    muestreo constante."""
    if len(t_s) < 2:
        return [], []
    t0, t1 = t_s[0], t_s[-1]
    n = max(2, int((t1 - t0) * fs))
    out_t = [t0 + i / fs for i in range(n)]
    out_v = []
    j = 0
    for t in out_t:
        while j < len(t_s) - 2 and t_s[j + 1] < t:
            j += 1
        t_a, t_b = t_s[j], t_s[j + 1]
        v_a, v_b = values[j], values[j + 1]
        frac = 0.0 if t_b <= t_a else (t - t_a) / (t_b - t_a)
        out_v.append(v_a + frac * (v_b - v_a))
    return out_t, out_v


def harmonic_ratio(values, fs, stride_time_s, n_harmonics=10, kind="even_over_odd"):
    """Harmonic Ratio (Moe-Nilssen 1998): suma de las amplitudes espectrales
    en los n_harmonics primeros multiplos de la frecuencia fundamental de
    zancada f0=1/stride_time_s (DFT directa en esas frecuencias concretas,
    mas robusto que buscar picos en una FFT completa sobre señales cortas).
    kind="even_over_odd" (vertical/antero-posterior: HR=pares/impares) o
    "odd_over_even" (medio-lateral: HR=impares/pares, por su periodicidad
    de paso en vez de zancada)."""
    if (stride_time_s is None or math.isnan(stride_time_s) or stride_time_s <= 0
            or len(values) < 8 or fs <= 0):
        return float("nan")
    mean = statistics.mean(values)
    sig = [v - mean for v in values]
    f0 = 1.0 / stride_time_s
    amps = []
    for h in range(1, n_harmonics + 1):
        f = h * f0
        if f >= fs / 2:
            amps.append(0.0)
            continue
        w = 2 * math.pi * f / fs
        re = sum(s * math.cos(-w * i) for i, s in enumerate(sig))
        im = sum(s * math.sin(-w * i) for i, s in enumerate(sig))
        amps.append(math.hypot(re, im))
    odd = sum(amps[0::2])   # armonicos 1,3,5,7,9
    even = sum(amps[1::2])  # armonicos 2,4,6,8,10
    if kind == "even_over_odd":
        return even / odd if odd > 1e-9 else float("nan")
    return odd / even if even > 1e-9 else float("nan")


def autocorr_regularity(values, fs, step_time_s, stride_time_s, search_frac=0.3):
    """Coeficientes de autocorrelacion normalizada en los lags
    correspondientes al tiempo de paso (Ad1, regularidad de paso) y de
    zancada (Ad2, regularidad de zancada) -- metodo de Moe-Nilssen &
    Helbostad (2004). Busca el maximo local en una ventana
    +/-search_frac*lag_esperado alrededor del lag esperado, en vez del lag
    exacto (rara vez coincide por la variabilidad natural de la marcha).
    Devuelve (Ad1, Ad2); NaN si no hay suficientes muestras o el tiempo de
    referencia no es valido."""
    if len(values) < 8 or fs <= 0:
        return float("nan"), float("nan")
    mean = statistics.mean(values)
    sig = [v - mean for v in values]
    n = len(sig)
    c0 = sum(s * s for s in sig)
    if c0 < 1e-12:
        return float("nan"), float("nan")

    def ac_at_lag(lag):
        if lag <= 0 or lag >= n:
            return float("nan")
        return sum(sig[i] * sig[i + lag] for i in range(n - lag)) / c0

    def best_near(expected_s):
        if expected_s is None or math.isnan(expected_s) or expected_s <= 0:
            return float("nan")
        lag0 = int(round(expected_s * fs))
        window = max(1, int(lag0 * search_frac))
        best, found = float("-inf"), False
        for lag in range(max(1, lag0 - window), min(n - 1, lag0 + window) + 1):
            v = ac_at_lag(lag)
            if not math.isnan(v) and v > best:
                best, found = v, True
        return best if found else float("nan")

    return best_near(step_time_s), best_near(stride_time_s)


def _moving_average(values, window):
    if window <= 1 or len(values) <= window:
        return list(values)
    half = window // 2
    n = len(values)
    out = []
    for i in range(n):
        lo, hi = max(0, i - half), min(n, i + half + 1)
        out.append(sum(values[lo:hi]) / (hi - lo))
    return out


def walker_accel_ap_ml(rb_track_xy_yaw, fs=60.0, smooth_window=9):
    """A partir de [(t_ns, (x,y,yaw))] del rigid body del andador (frame
    map), estima la aceleracion horizontal por segunda derivada numerica y
    la proyecta a los ejes antero-posterior (AP) y medio-lateral (ML) del
    propio andador usando su yaw en cada instante -- equivalente mocap de
    lo que mediria un IMU montado en el chasis con el eje x hacia delante.

    Mocap da timestamps IRREGULARES (medido en datos reales: mediana de dt
    ~8.3 ms a ~120 Hz nominal, pero con valores puntuales desde ~0.06 ms
    hasta ~80 ms por duplicados/huecos de tracking); derivar dos veces con
    diferencias finitas sobre esos dt reales divide por un dt casi nulo en
    esos puntos y dispara la aceleracion a decenas de m/s^2 sin relacion
    con el movimiento real (confirmado empiricamente). Por eso la posicion
    se remuestrea PRIMERO a una rejilla uniforme a `fs` Hz
    (resample_uniform(), interpolacion lineal -- de paso filtra los
    duplicados/huecos), se suaviza con una media movil de `smooth_window`
    muestras, y solo entonces se deriva dos veces con dt CONSTANTE
    (formula de 2a derivada centrada de 3 puntos, numericamente estable).
    El yaw se remuestrea via seno/coseno para evitar discontinuidades en
    +/-pi.

    Devuelve [(t_s, ap, ml)] con t_s relativo al primer instante."""
    if len(rb_track_xy_yaw) < max(3, smooth_window):
        return []
    t0_ns = rb_track_xy_yaw[0][0]
    ts = [(t - t0_ns) / 1e9 for t, _ in rb_track_xy_yaw]
    xs_raw = [p[0] for _, p in rb_track_xy_yaw]
    ys_raw = [p[1] for _, p in rb_track_xy_yaw]
    cos_raw = [math.cos(p[2]) for _, p in rb_track_xy_yaw]
    sin_raw = [math.sin(p[2]) for _, p in rb_track_xy_yaw]

    rt, xs = resample_uniform(ts, xs_raw, fs)
    _, ys = resample_uniform(ts, ys_raw, fs)
    _, cosy = resample_uniform(ts, cos_raw, fs)
    _, siny = resample_uniform(ts, sin_raw, fs)
    if len(rt) < max(3, smooth_window):
        return []

    xs = _moving_average(xs, smooth_window)
    ys = _moving_average(ys, smooth_window)
    dt = 1.0 / fs
    out = []
    for i in range(1, len(rt) - 1):
        ax = (xs[i + 1] - 2 * xs[i] + xs[i - 1]) / (dt * dt)
        ay = (ys[i + 1] - 2 * ys[i] + ys[i - 1]) / (dt * dt)
        yaw = math.atan2(siny[i], cosy[i])
        ap = ax * math.cos(yaw) + ay * math.sin(yaw)
        ml = -ax * math.sin(yaw) + ay * math.cos(yaw)
        out.append((rt[i], ap, ml))
    return out
