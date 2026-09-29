#!/usr/bin/env python3
"""Estima valores razonables de amplitud/frecuencia iniciales (kalman_model_a0,
kalman_model_f0) para el EKF de TrackLeg, a partir del movimiento REAL de
tobillo (ground truth de mocap, sin pasar por ningun detector).

Por que hace falta: el modelo es pos = d + a*sin(2*pi*f*t+p) por eje, y hoy
a0=f0=p0=0.001 (practicamente cero) para las cuatro variantes -- con
amplitud ~0 el jacobiano de medida respecto a la frecuencia (dpos/df =
2*pi*a*cos(p)) tambien es ~0, asi que la frecuencia (y la fase) son casi
inobservables: el EKF nunca aprende un valor real, y en la practica solo el
termino "d" (offset) seguia la pierna, como un filtro de posicion plano, no
el modelo de marcha periodica que el diseño pretendia. Arrancar con una
amplitud/frecuencia ya en el orden de magnitud correcto deberia des-
bloquear esa observabilidad.

Metodo por bag: para el eje x (frame laser, aproximadamente la direccion
de avance) de cada tobillo, quita la tendencia lenta (resta una media
movil) y mide:
  - amplitud: std de la señal destendida * sqrt(2) (una senoidal de
    amplitud a tiene std = a/sqrt(2))
  - frecuencia: pico dominante del espectro de Fourier de esa señal

Uso:
    python3 estimate_gait_params.py --bags-dir $DATASETS/labeled_bags --limit 15
"""
import argparse
import sys
from pathlib import Path

import numpy as np

from mocap_bag_io import FOOT_KEYS, find_labeled_bags, load_bag, load_calib, markers_by_name, project_map_to_laser, rigid_body_pose_xy_yaw

MIN_SAMPLES = 200
DETREND_WINDOW_S = 2.0  # ventana de la media movil (mas lenta que un ciclo de marcha)


def ankle_x_series(mk_msgs, rb_msgs, calib, key):
    rb_t = np.array([m[0] for m in rb_msgs], dtype=np.float64)
    t, x = [], []
    for ts, msg in mk_msgs:
        feet = markers_by_name(msg, FOOT_KEYS)
        if key not in feet:
            continue
        idx = np.argmin(np.abs(rb_t - ts))
        if abs(rb_t[idx] - ts) / 1e9 > 0.05:
            continue
        rb_x, rb_y, rb_yaw = rigid_body_pose_xy_yaw(rb_msgs[idx][1])
        lx, ly = project_map_to_laser(feet[key][0], feet[key][1], rb_x, rb_y, rb_yaw, calib)
        t.append(ts / 1e9)
        x.append(lx)
    return np.array(t), np.array(x)


def detrend_and_estimate(t, x):
    if len(t) < MIN_SAMPLES:
        return None
    dt = np.median(np.diff(t))
    win = max(3, int(round(DETREND_WINDOW_S / dt)))
    kernel = np.ones(win) / win
    trend = np.convolve(x, kernel, mode="same")
    resid = x - trend
    # recorta los bordes, afectados por el borde de la convolucion
    edge = win
    resid = resid[edge:-edge] if len(resid) > 2 * edge else resid
    if len(resid) < MIN_SAMPLES:
        return None

    amplitude = float(np.std(resid) * np.sqrt(2))

    # frecuencia dominante via FFT (senal re-muestreada a paso uniforme)
    t_u = np.linspace(t[edge], t[-edge - 1] if len(t) > 2 * edge else t[-1], len(resid))
    fs = 1.0 / np.median(np.diff(t_u)) if len(t_u) > 1 else 0
    if fs <= 0 or len(resid) < 16:
        return None
    freqs = np.fft.rfftfreq(len(resid), d=1.0 / fs)
    spec = np.abs(np.fft.rfft(resid - np.mean(resid)))
    # descarta 0Hz y frecuencias por encima de 3Hz (ruido/no fisiologico)
    valid = (freqs > 0.1) & (freqs < 3.0)
    if not np.any(valid):
        return None
    dominant_freq = float(freqs[valid][np.argmax(spec[valid])])

    return amplitude, dominant_freq


def process_bag(bag_dir, calib):
    try:
        data = load_bag(bag_dir, topics=("/labeled_markers", "/rigid_bodies"), require_all=True)
    except ValueError as e:
        print(f"  [!] {bag_dir.name}: {e}", file=sys.stderr)
        return None
    mk, rb = data["/labeled_markers"], data["/rigid_bodies"]
    if len(mk) < MIN_SAMPLES:
        return None

    results = []
    for key in ("l_ankle", "r_ankle"):
        t, x = ankle_x_series(mk, rb, calib, key)
        r = detrend_and_estimate(t, x)
        if r:
            results.append(r)
    if not results:
        return None
    amps = [r[0] for r in results]
    freqs = [r[1] for r in results]
    return float(np.mean(amps)), float(np.mean(freqs))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags-dir", required=True)
    ap.add_argument("--calib", default=str(Path(__file__).resolve().parent.parent / "config" / "mocap_to_base_footprint.yaml"))
    ap.add_argument("--limit", type=int, default=None)
    args = ap.parse_args()

    calib = load_calib(args.calib)
    bag_dirs = find_labeled_bags(args.bags_dir)
    if args.limit:
        bag_dirs = bag_dirs[: args.limit]
    print(f"{len(bag_dirs)} bags a analizar\n")

    all_amps, all_freqs = [], []
    for bag_dir in bag_dirs:
        r = process_bag(bag_dir, calib)
        if r is None:
            print(f"  [-] {bag_dir.name}: descartado")
            continue
        amp, freq = r
        print(f"  [{bag_dir.name}] amplitud={amp:.3f}m  frecuencia={freq:.3f}Hz")
        all_amps.append(amp)
        all_freqs.append(freq)

    if not all_amps:
        sys.exit("Sin datos suficientes")

    print(f"\n=== {len(all_amps)} bags ===")
    print(f"amplitud: media={np.mean(all_amps):.3f}m  mediana={np.median(all_amps):.3f}m  "
          f"p25={np.percentile(all_amps,25):.3f}  p75={np.percentile(all_amps,75):.3f}")
    print(f"frecuencia: media={np.mean(all_freqs):.3f}Hz  mediana={np.median(all_freqs):.3f}Hz  "
          f"p25={np.percentile(all_freqs,25):.3f}  p75={np.percentile(all_freqs,75):.3f}")


if __name__ == "__main__":
    main()
