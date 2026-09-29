#!/usr/bin/env python3
"""Calibra el offset fijo rigid_body[0] (andador, mocap) -> base_footprint.

Por que hace falta: /rigid_bodies[0].pose da la pose del andador en el frame
"map" del mocap (bag_checker ya corrigio su orientacion para que apunte en
el sentido de marcha), pero el punto y ejes que reporta el software de mocap
para ese rigid body no tienen por que coincidir exactamente con el origen
real de base_footprint -- hay un offset de montaje fijo (mismo andador en
todos los bags) sin calibrar todavia.

Metodo (ICP 2D, ya que el laser es un plano): con una estimacion inicial
identidad,
  1. proyecta l_ankle/r_ankle (frame map) a "laser" usando rigid_body.pose +
     calib_actual + base_footprint->laser (fijo, del xacro),
  2. empareja cada proyeccion con el punto laser cartesiano mas cercano
     (gate en metros, se relaja en las primeras iteraciones),
  3. reajusta (dx, dy, dyaw) por minimos cuadrados robustos (soft_l1) sobre
     todos los pares acumulados de todos los bags,
  4. repite hasta que el residuo deja de bajar.

El offset en Z (altura) no es observable con un laser 2D -- un dz no cambia
la proyeccion (x,y). Se ignora aqui a proposito; ver README para la nota
sobre el sesgo esperado por diferencia de altura tobillo/plano-laser.

Uso:
    python3 calibrate_mocap_to_laser.py \
        --bags-dir $DATASETS/labeled_bags \
        --out config/mocap_to_base_footprint.yaml
"""
import argparse
import math
import sys

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

from mocap_bag_io import (
    FOOT_KEYS, find_labeled_bags, load_bag, nearest, rigid_body_pose_xy_yaw,
    markers_by_name, scan_to_cartesian, project_map_to_laser,
)

MAX_DT_NS = int(0.05 * 1e9)  # 50 ms: tolerancia scan<->rigid_bodies/markers


def collect_frames(bag_dirs, max_range=2.5):
    """Por cada bag: lista de (px, py, rb_x, rb_y, rb_yaw, scan_pts_xy)."""
    frames = []
    for bag_dir in bag_dirs:
        try:
            data = load_bag(bag_dir)
        except ValueError as e:
            print(f"  [!] {bag_dir.name}: {e}", file=sys.stderr)
            continue
        scans = data["/scan"]
        rbs = data["/rigid_bodies"]
        mks = data["/labeled_markers"]
        if not scans or not rbs or not mks:
            continue
        n_ok = 0
        for t_scan, scan in scans:
            rb = nearest(rbs, t_scan, MAX_DT_NS)
            mk = nearest(mks, t_scan, MAX_DT_NS)
            if rb is None or mk is None:
                continue
            rb_x, rb_y, rb_yaw = rigid_body_pose_xy_yaw(rb[1])
            feet = markers_by_name(mk[1], FOOT_KEYS)
            if not feet:
                continue
            scan_pts = scan_to_cartesian(scan, max_range=max_range)
            if len(scan_pts) < 3:
                continue
            for name, (px, py, _pz) in feet.items():
                frames.append((px, py, rb_x, rb_y, rb_yaw, scan_pts))
            n_ok += 1
        print(f"  {bag_dir.name}: {n_ok}/{len(scans)} scans con match completo")
    return frames


_KD_CACHE = {}


def frames_kdtree_cache(scan_pts):
    key = id(scan_pts)
    tree = _KD_CACHE.get(key)
    if tree is None:
        tree = cKDTree(np.asarray(scan_pts))
        _KD_CACHE[key] = tree
    return tree


def match_pairs(frames, calib, gate):
    """Paso E: para calib fijo, empareja cada pie con su punto laser mas
    cercano (si cae dentro del gate). Devuelve arrays fijos (no cambian de
    tamano mientras se optimiza calib en el paso M siguiente)."""
    kept = []  # (px, py, rb_x, rb_y, rb_yaw)
    targets = []  # (ax, ay) punto laser emparejado, fijo durante el paso M
    for px, py, rb_x, rb_y, rb_yaw, scan_pts in frames:
        lx, ly = project_map_to_laser(px, py, rb_x, rb_y, rb_yaw, calib)
        tree = frames_kdtree_cache(scan_pts)
        d, idx = tree.query([lx, ly])
        if d <= gate:
            kept.append((px, py, rb_x, rb_y, rb_yaw))
            targets.append(scan_pts[idx])
    return kept, np.asarray(targets)


def residuals_for(kept, targets, calib):
    out = np.empty(2 * len(kept))
    for i, (px, py, rb_x, rb_y, rb_yaw) in enumerate(kept):
        lx, ly = project_map_to_laser(px, py, rb_x, rb_y, rb_yaw, calib)
        out[2 * i] = lx - targets[i][0]
        out[2 * i + 1] = ly - targets[i][1]
    return out


def fit_calib(frames, gate_schedule=(1.0, 0.6, 0.4, 0.3), n_iters_per_gate=2):
    calib = np.zeros(3)  # dx, dy, dyaw
    history = []
    for gate in gate_schedule:
        for _ in range(n_iters_per_gate):
            # Paso E: correspondencia fija para este calib/gate.
            kept, targets = match_pairs(frames, calib, gate)
            n_matched = len(kept)
            if n_matched < 20:
                print(f"  gate={gate}: solo {n_matched} pares, se para aqui")
                break
            errs = residuals_for(kept, targets, calib).reshape(-1, 2)
            rmse = float(np.sqrt((errs ** 2).sum(axis=1).mean()))
            history.append((gate, n_matched, rmse, tuple(calib)))
            print(f"  gate={gate:.2f}  pares={n_matched:5d}  rmse={rmse:.3f} m  "
                  f"calib=(dx={calib[0]:+.3f}, dy={calib[1]:+.3f}, dyaw={math.degrees(calib[2]):+.2f} deg)")

            # Paso M: minimiza sobre ESA correspondencia fija (residuo de
            # longitud constante, lo que least_squares necesita).
            res = least_squares(
                lambda c: residuals_for(kept, targets, c),
                calib, loss="soft_l1", f_scale=0.1,
            )
            calib = res.x
    kept, targets = match_pairs(frames, calib, gate_schedule[-1])
    errs = residuals_for(kept, targets, calib).reshape(-1, 2) if kept else np.empty((0, 2))
    final_rmse = float(np.sqrt((errs ** 2).sum(axis=1).mean())) if len(errs) else float("nan")
    return calib, final_rmse, len(kept), history


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags-dir", required=True)
    ap.add_argument("--out", default=None, help="YAML de salida con la calibracion")
    ap.add_argument("--max-range", type=float, default=2.5, help="recorta el scan a este radio (m) antes del ICP")
    args = ap.parse_args()

    bag_dirs = find_labeled_bags(args.bags_dir)
    if not bag_dirs:
        sys.exit(f"No se encontraron bags (metadata.yaml) bajo {args.bags_dir}")
    print(f"Bags encontrados: {[b.name for b in bag_dirs]}")

    print("\nCargando frames y emparejando timestamps...")
    frames = collect_frames(bag_dirs, max_range=args.max_range)
    print(f"\nTotal frames (pie, scan) utilizables: {len(frames)}")
    if len(frames) < 50:
        sys.exit("Muy pocos frames emparejados, revisa los topics/timestamps de los bags")

    print("\nAjustando calib (dx, dy, dyaw) por ICP 2D...")
    calib, rmse, n_matched, history = fit_calib(frames)

    dx, dy, dyaw = calib
    print("\n=== Resultado ===")
    print(f"dx={dx:+.4f} m  dy={dy:+.4f} m  dyaw={math.degrees(dyaw):+.3f} deg")
    print(f"RMSE final: {rmse:.4f} m sobre {n_matched} pares (gate={history[-1][0] if history else '?'} m)")
    print(f"RMSE inicial (calib=0): {history[0][2]:.4f} m sobre {history[0][1]} pares" if history else "")

    if args.out:
        out = {
            "note": "rigid_body[0] (andador, mocap map frame) -> base_footprint. "
                    "dz no observable con laser 2D, no incluido. "
                    "Ver calibrate_mocap_to_laser.py para el metodo (ICP 2D).",
            "dx": float(dx), "dy": float(dy), "dyaw": float(dyaw),
            "rmse_m": rmse, "n_pairs": n_matched,
            "n_bags": len(bag_dirs), "bags": [b.name for b in bag_dirs],
        }
        with open(args.out, "w") as f:
            yaml.dump(out, f, sort_keys=False)
        print(f"\nGuardado en {args.out}")


if __name__ == "__main__":
    main()
