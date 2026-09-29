#!/usr/bin/env python3
"""Recalcula, a partir del mocap etiquetado, los mismos parametros que
publica walker_loads/gait_monitor_speed.py (nodo `gait_monitor_speed`):
Step/Stride time y length por pierna, Numero de pasos acumulado, distancia
recorrida, cadencia y velocidad media de marcha -- punto 1 del plan de
validacion (recalcular con mocap para poder comparar en el punto 2).

Que hace, por bag:
  1. Lee /labeled_markers (l_ankle, r_ankle) y /rigid_bodies (rigidbodies[0]
     = andador), ver mocap_eval/bag_io.py.
  2. Detecta eventos de apoyo por pie a partir de la cinematica del tobillo
     (mocap_eval/contact_events.py) -- el mocap no mide carga, asi que esto
     sustituye al flanco load>0 de walker_msgs/StepStamped que usa el nodo
     real.
  3. Proyecta cada evento (frame "map") al frame del andador ("laser") con
     la calibracion ya existente (walker_step_detector/config/
     mocap_to_base_footprint.yaml), para que las distancias sean
     comparables a las que ve el nodo real (que trabaja en ese frame).
  4. Corre el mismo algoritmo que GaitMonitorSp (mocap_eval/gait_metrics.py)
     sobre esos eventos, con la distancia recorrida real (arclength del
     rigid body del andador en el mocap) en vez del valor hardcodeado que
     usa hoy gait_monitor_speed.py::getTravelledDist() -- ver README.md de
     este paquete para el detalle de esa discrepancia.

Uso:
    python3 compute_gait_from_mocap.py --bags-dir $DATASETS/labeled_bags \
        --calib ../../walker_step_detector/config/mocap_to_base_footprint.yaml \
        --out /tmp/mocap_gait_results.json

    python3 compute_gait_from_mocap.py --bag $DATASETS/labeled_bags/CA_test01 \
        --calib ../../walker_step_detector/config/mocap_to_base_footprint.yaml
"""
import argparse
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mocap_eval import bag_io, contact_events, gait_metrics  # noqa: E402

DEFAULT_CALIB = (
    Path(__file__).resolve().parent.parent.parent
    / "walker_step_detector" / "config" / "mocap_to_base_footprint.yaml"
)


def compute_bag(bag_dir, calib, speed_threshold, min_gap_s, max_dt_ns):
    """Devuelve el dict de gait_metrics.summarize() para un bag, o None si
    falta ground truth o no hay pasos suficientes."""
    data = bag_io.load_bag(bag_dir, topics=("/labeled_markers", "/rigid_bodies"), require_all=True)
    mk_msgs, rb_msgs = data["/labeled_markers"], data["/rigid_bodies"]
    if not mk_msgs or not rb_msgs:
        return None

    rb_track = bag_io.rigid_body_track_xy(rb_msgs)

    events = []
    for leg, key in (("left", "l_ankle"), ("right", "r_ankle")):
        track = bag_io.marker_track(mk_msgs, key)
        for t_evt, p_evt in contact_events.detect_stance_events(track, speed_threshold, min_gap_s):
            rb = bag_io.nearest(rb_msgs, t_evt, max_dt_ns)
            if rb is None:
                continue
            rb_x, rb_y, rb_yaw = bag_io.rigid_body_pose_xy_yaw(rb[1])
            lx, ly = bag_io.project_map_to_laser(p_evt[0], p_evt[1], rb_x, rb_y, rb_yaw, calib)
            events.append((t_evt, leg, (lx, ly)))

    left_log, right_log = gait_metrics.process_events(events)
    if not left_log.events or not right_log.events:
        return None

    t_start = min(left_log.events[0][0], right_log.events[0][0])
    t_end = max(left_log.events[-1][0], right_log.events[-1][0])
    distance_m = bag_io.arclength_m(bag_io.clip_track(rb_track, t_start, t_end))

    return gait_metrics.summarize(left_log, right_log, distance_m)


def fmt(v, nd=3):
    if v is None:
        return "-"
    if isinstance(v, float) and math.isnan(v):
        return "-"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def print_table(results):
    cols = ["bag", "l_spt", "l_sdt", "l_spl", "l_sdl", "l_nos",
            "r_spt", "r_sdt", "r_spl", "r_sdl", "r_nos",
            "nos", "tr_s", "d_m", "cad", "wv"]
    rows = []
    for bag_name, r in results.items():
        if r is None:
            rows.append([bag_name] + ["-"] * (len(cols) - 1))
            continue
        rows.append([
            bag_name,
            fmt(r["left_spt"]), fmt(r["left_sdt"]), fmt(r["left_spl"]), fmt(r["left_sdl"]), r["left_nos"],
            fmt(r["right_spt"]), fmt(r["right_sdt"]), fmt(r["right_spl"]), fmt(r["right_sdl"]), r["right_nos"],
            r["nos"], fmt(r["tr_s"]), fmt(r["d_m"]), fmt(r["cad"]), fmt(r["wv"]),
        ])
    widths = [max(len(str(row[i])) for row in ([cols] + rows)) for i in range(len(cols))]
    line = "  ".join(str(c).ljust(w) for c, w in zip(cols, widths))
    print(line)
    print("-" * len(line))
    for row in rows:
        print("  ".join(str(c).ljust(w) for c, w in zip(row, widths)))


def print_aggregate(results):
    valid = [r for r in results.values() if r is not None]
    if not valid:
        print("\nSin bags validos para agregar.")
        return
    print(f"\nAgregado sobre {len(valid)}/{len(results)} bags:")
    for key in ("left_spt", "left_sdt", "left_spl", "left_sdl",
                "right_spt", "right_sdt", "right_spl", "right_sdl",
                "cad", "wv"):
        vals = [r[key] for r in valid if not math.isnan(r[key])]
        if len(vals) >= 2:
            print(f"  {key:16s} mean={statistics.mean(vals):.3f}  std={statistics.stdev(vals):.3f}  n={len(vals)}")
        elif vals:
            print(f"  {key:16s} value={vals[0]:.3f}  n=1")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag", help="un unico bag etiquetado")
    src.add_argument("--bags-dir", help="carpeta con varios bags etiquetados (p.ej. $DATASETS/labeled_bags)")
    ap.add_argument("--calib", default=str(DEFAULT_CALIB),
                     help="mocap_to_base_footprint.yaml (por defecto, el de walker_step_detector)")
    ap.add_argument("--speed-threshold", type=float, default=0.25,
                     help="m/s por debajo del cual se considera apoyo (ver contact_events.py)")
    ap.add_argument("--min-gap-s", type=float, default=0.3,
                     help="separacion minima entre dos eventos de apoyo consecutivos del mismo pie")
    ap.add_argument("--max-dt-ms", type=float, default=50.0,
                     help="tolerancia de emparejado evento<->rigid_bodies")
    ap.add_argument("--out", default=None, help="JSON de salida con las metricas por bag")
    args = ap.parse_args()

    calib = bag_io.load_calib(args.calib)
    print(f"Calibracion: dx={calib[0]:+.3f} dy={calib[1]:+.3f} dyaw={math.degrees(calib[2]):+.2f} deg  ({args.calib})")

    bag_dirs = [Path(args.bag)] if args.bag else bag_io.find_labeled_bags(args.bags_dir)
    if not bag_dirs:
        sys.exit(f"No se encontraron bags etiquetados en {args.bags_dir}")

    max_dt_ns = int(args.max_dt_ms * 1e6)
    results = {}
    for bag_dir in bag_dirs:
        try:
            results[bag_dir.name] = compute_bag(bag_dir, calib, args.speed_threshold, args.min_gap_s, max_dt_ns)
        except ValueError as e:
            print(f"[!] {bag_dir}: {e}", file=sys.stderr)
            results[bag_dir.name] = None

    print()
    print_table(results)
    print_aggregate(results)

    if args.out:
        out = {k: v for k, v in results.items()}
        out["_meta"] = {
            "calib": args.calib, "speed_threshold": args.speed_threshold,
            "min_gap_s": args.min_gap_s, "max_dt_ms": args.max_dt_ms,
        }
        with open(args.out, "w") as f:
            json.dump(out, f, indent=2, ensure_ascii=False)
        print(f"\nGuardado en {args.out}")


if __name__ == "__main__":
    main()
