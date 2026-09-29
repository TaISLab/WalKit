#!/usr/bin/env python3
"""Valida, con ground truth de mocap puro (sin pasar por ningun detector),
si la relacion mecanica entre las dos piernas es lo bastante estable como
para merecer un EKF conjunto en legs_tracker.cpp (en vez de dos TrackLeg
independientes + heuristicas de asociacion, que es lo que hay hoy).

Dos cosas a comprobar, las dos necesarias para que un modelo acoplado
tenga sentido:

  1. Relacion de fase entre piernas: durante marcha alternante, cuando una
     pierna va rapido (swing) la otra deberia ir despacio (stance) --
     deberia verse como correlacion NEGATIVA entre |v_l_ankle(t)| y
     |v_r_ankle(t)|. Se mide a lag=0 (mismo timestamp exacto, ambas series
     vienen del mismo mensaje de mocap, sin incertidumbre de sincronizacion)
     y al mejor lag en +-1.5s (por si el desfase real no es exactamente 0).

  2. Ancho de zancada (separacion lateral pie-pie): si esta razonablemente
     acotado (poca dispersion relativa a la media), es una restriccion util
     para el filtro. Tambien se mide la fraccion de frames "cruzados" (el
     pie que normalmente esta mas a la izquierda pasa a estar mas a la
     derecha en el frame del laser, usando la calibracion ya validada) --
     mide directamente cuanto ocurre el "cruce de piernas" que motiva la
     pregunta.

Uso:
    python3 validate_leg_coupling.py --bags-dir $DATASETS/labeled_bags
"""
import argparse
import sys
from pathlib import Path

import numpy as np

from mocap_bag_io import (
    FOOT_KEYS, find_labeled_bags, load_bag, load_calib, markers_by_name,
    project_map_to_laser, rigid_body_pose_xy_yaw,
)

LAGS_S = np.arange(-1.5, 1.51, 0.1)
MIN_SAMPLES = 60


def ankle_series(mk_msgs, rb_msgs, calib):
    """[(t_ns, lx, ly, rx, ry)] en frame laser (via calibracion), solo frames
    con ambos tobillos y rigid_body disponibles, emparejando por cercania de
    tiempo (labeled_markers y rigid_bodies vienen juntos, misma tasa)."""
    rb_t = np.array([m[0] for m in rb_msgs], dtype=np.float64)
    out = []
    for t, msg in mk_msgs:
        feet = markers_by_name(msg, FOOT_KEYS)
        if "l_ankle" not in feet or "r_ankle" not in feet:
            continue
        idx = np.argmin(np.abs(rb_t - t))
        if abs(rb_t[idx] - t) / 1e9 > 0.05:
            continue
        rb_x, rb_y, rb_yaw = rigid_body_pose_xy_yaw(rb_msgs[idx][1])
        lx, ly = project_map_to_laser(feet["l_ankle"][0], feet["l_ankle"][1], rb_x, rb_y, rb_yaw, calib)
        rx, ry = project_map_to_laser(feet["r_ankle"][0], feet["r_ankle"][1], rb_x, rb_y, rb_yaw, calib)
        out.append((t, lx, ly, rx, ry))
    return out


def process_bag(bag_dir, calib):
    try:
        data = load_bag(bag_dir, topics=("/labeled_markers", "/rigid_bodies"), require_all=True)
    except ValueError as e:
        print(f"  [!] {bag_dir.name}: {e}", file=sys.stderr)
        return None
    mk, rb = data["/labeled_markers"], data["/rigid_bodies"]
    if len(mk) < MIN_SAMPLES or len(rb) < MIN_SAMPLES:
        return None

    series = ankle_series(mk, rb, calib)
    if len(series) < MIN_SAMPLES:
        return None

    t = np.array([s[0] for s in series], dtype=np.float64)
    lx = np.array([s[1] for s in series]); ly = np.array([s[2] for s in series])
    rx = np.array([s[3] for s in series]); ry = np.array([s[4] for s in series])

    dt = np.diff(t) / 1e9
    ok = dt > 0
    vl = np.hypot(np.diff(lx), np.diff(ly))[ok] / dt[ok]
    vr = np.hypot(np.diff(rx), np.diff(ry))[ok] / dt[ok]
    t_v = t[1:][ok]

    # --- 1. fase: correlacion velocidad izq vs velocidad der ---
    corr0 = float(np.corrcoef(vl, vr)[0, 1]) if np.std(vl) > 1e-6 and np.std(vr) > 1e-6 else float("nan")

    best_lag, best_corr = 0.0, corr0
    for lag_s in LAGS_S:
        shift_n = int(round(lag_s / np.median(dt[ok])))
        if shift_n == 0:
            continue
        if shift_n > 0:
            a, b = vl[shift_n:], vr[:-shift_n]
        else:
            a, b = vl[:shift_n], vr[-shift_n:]
        if len(a) < MIN_SAMPLES or np.std(a) < 1e-6 or np.std(b) < 1e-6:
            continue
        c = float(np.corrcoef(a, b)[0, 1])
        if c < best_corr:  # buscamos la MAS NEGATIVA (antifase)
            best_corr, best_lag = c, float(lag_s)

    # --- 2. ancho de zancada + cruces (frame laser, via calibracion) ---
    step_width = np.hypot(lx - rx, ly - ry)
    lateral_diff = ly - ry  # >0: l mas a la izquierda que r (convencion habitual)
    majority_sign = 1.0 if np.median(lateral_diff) >= 0 else -1.0
    crossed_frac = float(np.mean(np.sign(lateral_diff) != majority_sign))

    return {
        "n_speed": len(vl), "corr0": corr0, "best_lag_s": best_lag, "best_corr": best_corr,
        "step_width_mean": float(np.mean(step_width)), "step_width_std": float(np.std(step_width)),
        "crossed_frac": crossed_frac,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags-dir", required=True)
    ap.add_argument("--calib", default=str(Path(__file__).resolve().parent.parent / "config" / "mocap_to_base_footprint.yaml"))
    args = ap.parse_args()

    calib = load_calib(args.calib)
    bag_dirs = find_labeled_bags(args.bags_dir)
    print(f"{len(bag_dirs)} bags encontrados\n")

    results = {}
    for bag_dir in bag_dirs:
        r = process_bag(bag_dir, calib)
        if r is None:
            print(f"  [-] {bag_dir.name}: descartado (datos insuficientes)")
            continue
        results[bag_dir.name] = r
        print(f"  [{bag_dir.name}] corr0={r['corr0']:+.3f}  mejor_lag={r['best_lag_s']:+.1f}s "
              f"corr={r['best_corr']:+.3f}  ancho_zancada={r['step_width_mean']:.3f}+-{r['step_width_std']:.3f}m  "
              f"cruzado={100*r['crossed_frac']:.1f}%")

    if not results:
        sys.exit("\nNingun bag tenia datos suficientes")

    print(f"\n=== {len(results)}/{len(bag_dirs)} bags usables ===")
    corr0_vals = np.array([r["corr0"] for r in results.values() if not np.isnan(r["corr0"])])
    best_vals = np.array([r["best_corr"] for r in results.values()])
    best_lags = np.array([r["best_lag_s"] for r in results.values()])
    widths_mean = np.array([r["step_width_mean"] for r in results.values()])
    widths_std = np.array([r["step_width_std"] for r in results.values()])
    crossed = np.array([r["crossed_frac"] for r in results.values()])

    print(f"\n-- Fase (velocidad izq vs der) --")
    print(f"corr(lag=0): media={np.mean(corr0_vals):+.3f}  mediana={np.median(corr0_vals):+.3f}  "
          f"negativos en {np.sum(corr0_vals < 0)}/{len(corr0_vals)} bags")
    print(f"mejor corr (antifase, cualquier lag): media={np.mean(best_vals):+.3f}  mediana={np.median(best_vals):+.3f}  "
          f"<-0.3 en {np.sum(best_vals < -0.3)}/{len(best_vals)} bags")
    print(f"lag optimo: media={np.mean(best_lags):+.2f}s  mediana={np.median(best_lags):+.2f}s  std={np.std(best_lags):.2f}s")

    print(f"\n-- Ancho de zancada (separacion lateral pie-pie, frame laser) --")
    print(f"media de medias por bag: {np.mean(widths_mean):.3f} m  "
          f"dispersion tipica dentro de un bag: {np.mean(widths_std):.3f} m "
          f"(cv medio = {np.mean(widths_std/widths_mean):.2f})")

    print(f"\n-- Cruces (pie 'izquierdo' pasa a estar mas a la derecha que el 'derecho') --")
    print(f"fraccion de frames cruzados: media={100*np.mean(crossed):.1f}%  mediana={100*np.median(crossed):.1f}%  "
          f"max={100*np.max(crossed):.1f}%")


if __name__ == "__main__":
    main()
