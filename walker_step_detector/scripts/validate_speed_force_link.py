#!/usr/bin/env python3
"""Valida, con datos reales (sin pasar por el detector de pasos), si existe
una relacion explotable entre la velocidad relativa de los tobillos
(ground truth de mocap: l_ankle vs r_ankle, izquierda/derecha correctas por
construccion) y la asimetria de fuerza en las manetas (/left_handle,
/right_handle -- inequivocas por construccion, cada sensor esta montado en
una maneta fisica concreta).

Por que asi: walker_loads/partial_loads.cpp ya fusiona ambas senales
(speed_diff del detector de pasos, force_diff de las manetas) en un unico
Kalman de dos senos con la MISMA frecuencia y un desfase libre `d` entre
ellas (ver DiffSystemModel.hpp/ForceMeasurementModel.hpp) -- es decir, el
diseno ya asume que la relacion es un desfase de fase, no una correlacion
simple del mismo signo. Aqui se comprueba esa asuncion directamente contra
el ground truth de mocap, sin que los bugs del detector de pasos puedan
contaminar la validacion.

Metodo por bag:
  1. velocidad de tobillo (m/s) por diferencias finitas consecutivas de
     /labeled_markers (l_ankle, r_ankle) -- independiente de cualquier frame,
     solo magnitud del desplazamiento / dt.
  2. speed_diff_gt(t) = |v_l_ankle(t)| - |v_r_ankle(t)|, interpolado a los
     timestamps de /left_handle y /right_handle (que son mucho mas
     esporadicos, ~2-3 Hz tipico).
  3. force_diff(t) = left_handle.force - right_handle.force en esos mismos
     timestamps (emparejando cada muestra de una maneta con la mas cercana
     de la otra, dentro de una tolerancia).
  4. correlacion de Pearson a varios desfases temporales (lag) entre
     speed_diff_gt y force_diff -- si el mejor lag da una correlacion
     fuerte y consistente entre bags, la relacion es explotable para
     detectar/corregir un swap de identidad izquierda/derecha en el
     detector de pasos; si no, no lo es.

Uso:
    python3 validate_speed_force_link.py --bags-dir $DATASETS/labeled_bags
"""
import argparse
import sys
from pathlib import Path

import numpy as np

from mocap_bag_io import find_labeled_bags, load_bag, markers_by_name, FOOT_KEYS

LAGS_S = np.arange(-1.5, 1.51, 0.1)  # +/- 1.5s, cubre un ciclo de marcha tipico
MIN_HANDLE_MSGS = 40
MAX_HANDLE_DT_S = 1.0          # por encima de esto, el sensor no es fiable/continuo
HANDLE_MATCH_TOL_S = 0.3       # tolerancia para emparejar left_handle<->right_handle
GT_MATCH_TOL_S = 0.1           # tolerancia para interpolar velocidad de tobillo


def ankle_speed_series(mk_msgs):
    """[(t_ns, l_speed, r_speed)], velocidad = |desplazamiento|/dt entre
    frames CONSECUTIVOS de /labeled_markers que tengan ambos tobillos."""
    pts = []
    for t, msg in mk_msgs:
        feet = markers_by_name(msg, FOOT_KEYS)
        if "l_ankle" in feet and "r_ankle" in feet:
            pts.append((t, feet["l_ankle"], feet["r_ankle"]))
    out = []
    for (t0, l0, r0), (t1, l1, r1) in zip(pts, pts[1:]):
        dt = (t1 - t0) / 1e9
        if dt <= 0 or dt > 0.5:  # gap demasiado grande, no es un frame consecutivo real
            continue
        lv = np.hypot(l1[0] - l0[0], l1[1] - l0[1]) / dt
        rv = np.hypot(r1[0] - r0[0], r1[1] - r0[1]) / dt
        out.append((t1, lv, rv))
    return out


def interp_at(series_t, series_v, query_t):
    """Interpolacion lineal simple; None si query_t cae fuera del rango con margen."""
    if not len(series_t) or query_t < series_t[0] - GT_MATCH_TOL_S * 1e9 or query_t > series_t[-1] + GT_MATCH_TOL_S * 1e9:
        return None
    return float(np.interp(query_t, series_t, series_v))


def process_bag(bag_dir):
    try:
        data = load_bag(bag_dir, topics=("/left_handle", "/right_handle", "/labeled_markers"), require_all=True)
    except ValueError:
        return None

    lh, rh, mk = data["/left_handle"], data["/right_handle"], data["/labeled_markers"]
    if len(lh) < MIN_HANDLE_MSGS or len(rh) < MIN_HANDLE_MSGS:
        return None

    lh_dt = np.median(np.diff([m[0] for m in lh])) / 1e9
    rh_dt = np.median(np.diff([m[0] for m in rh])) / 1e9
    if lh_dt > MAX_HANDLE_DT_S or rh_dt > MAX_HANDLE_DT_S:
        return None
    # asimetria de tasa entre left/right (visto en vivo: algunos bags tienen
    # sensores con tasas muy distintas entre manetas -- sospechoso, se descarta)
    if max(lh_dt, rh_dt) / max(min(lh_dt, rh_dt), 1e-6) > 5:
        return None

    speeds = ankle_speed_series(mk)
    if len(speeds) < 30:
        return None
    st = np.array([s[0] for s in speeds], dtype=np.float64)
    lv = np.array([s[1] for s in speeds])
    rv = np.array([s[2] for s in speeds])
    speed_diff_t = st
    speed_diff_v = lv - rv

    # empareja left_handle <-> right_handle mas cercano dentro de tolerancia
    rh_t = np.array([m[0] for m in rh], dtype=np.float64)
    rh_f = np.array([m[1].force for m in rh])
    pairs_t, force_diff = [], []
    for t, msg in lh:
        idx = np.argmin(np.abs(rh_t - t))
        if abs(rh_t[idx] - t) / 1e9 <= HANDLE_MATCH_TOL_S:
            pairs_t.append(t)
            force_diff.append(msg.force - rh_f[idx])
    if len(pairs_t) < MIN_HANDLE_MSGS:
        return None
    pairs_t = np.array(pairs_t, dtype=np.float64)
    force_diff = np.array(force_diff)

    # para cada lag candidato: interpola speed_diff_gt en (t_handle - lag) y
    # correla contra force_diff(t_handle)
    best = (0.0, 0.0, 0)  # lag_s, corr, n
    for lag_s in LAGS_S:
        query_t = pairs_t - lag_s * 1e9
        sd = [interp_at(speed_diff_t, speed_diff_v, qt) for qt in query_t]
        valid = np.array([v is not None for v in sd])
        if valid.sum() < 20:
            continue
        sd_v = np.array([v for v in sd if v is not None])
        fd_v = force_diff[valid]
        if np.std(sd_v) < 1e-6 or np.std(fd_v) < 1e-6:
            continue
        corr = float(np.corrcoef(sd_v, fd_v)[0, 1])
        if abs(corr) > abs(best[1]):
            best = (float(lag_s), corr, int(valid.sum()))

    zero_lag_query = pairs_t
    sd0 = np.array([interp_at(speed_diff_t, speed_diff_v, qt) for qt in zero_lag_query])
    valid0 = np.array([v is not None for v in sd0])
    corr0 = float("nan")
    if valid0.sum() >= 20:
        sd0_v = np.array([v for v in sd0 if v is not None])
        fd0_v = force_diff[valid0]
        if np.std(sd0_v) > 1e-6 and np.std(fd0_v) > 1e-6:
            corr0 = float(np.corrcoef(sd0_v, fd0_v)[0, 1])

    return {
        "n_speed_samples": len(speeds), "n_force_pairs": len(pairs_t),
        "corr_lag0": corr0, "best_lag_s": best[0], "best_corr": best[1], "best_n": best[2],
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags-dir", required=True)
    args = ap.parse_args()

    bag_dirs = find_labeled_bags(args.bags_dir)
    print(f"{len(bag_dirs)} bags encontrados\n")

    results = {}
    for bag_dir in bag_dirs:
        r = process_bag(bag_dir)
        if r is None:
            print(f"  [-] {bag_dir.name}: descartado (sensores de maneta insuficientes/irregulares)")
            continue
        results[bag_dir.name] = r
        print(f"  [{bag_dir.name}] corr(lag=0)={r['corr_lag0']:+.3f}  "
              f"mejor: lag={r['best_lag_s']:+.1f}s corr={r['best_corr']:+.3f} (n={r['best_n']})")

    if not results:
        sys.exit("\nNingun bag tenia datos de maneta usables para esta validacion")

    print(f"\n=== {len(results)}/{len(bag_dirs)} bags usables ===")
    corr0_vals = np.array([r["corr_lag0"] for r in results.values() if not np.isnan(r["corr_lag0"])])
    best_vals = np.array([r["best_corr"] for r in results.values()])
    best_lags = np.array([r["best_lag_s"] for r in results.values()])

    print(f"corr(lag=0): media={np.mean(corr0_vals):+.3f}  mediana={np.median(corr0_vals):+.3f}  "
          f"positivos={np.sum(corr0_vals>0)}/{len(corr0_vals)}")
    print(f"mejor corr (cualquier lag): media={np.mean(best_vals):+.3f}  mediana={np.median(best_vals):+.3f}  "
          f"|corr|>0.3 en {np.sum(np.abs(best_vals)>0.3)}/{len(best_vals)} bags  "
          f"positivos={np.sum(best_vals>0)}/{len(best_vals)}")
    print(f"lag optimo: media={np.mean(best_lags):+.2f}s  mediana={np.median(best_lags):+.2f}s  "
          f"std={np.std(best_lags):.2f}s")


if __name__ == "__main__":
    main()
