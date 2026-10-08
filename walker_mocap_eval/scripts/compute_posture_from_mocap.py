#!/usr/bin/env python3
"""Calcula, desde el mocap etiquetado, los parametros posturales/de
estabilidad del paso 3 del plan de validacion (ver walker_mocap_eval/
readme.md): inclinacion de tronco, ancho de paso, tiempo de doble apoyo y
margen de estabilidad lateral real -- pensados para, mas adelante, buscar
correlacion con datos del andador (reparto de carga manetas/pies, centroide
de soporte, StabilityStamped) sobre los mismos bags.

A diferencia de compute_gait_from_mocap.py (que recalcula algo que YA
publica un nodo del andador, para comparar 1:1), estos parametros no tienen
hoy un equivalente publicado -- este script solo los calcula desde mocap;
la correlacion con datos del andador es un paso posterior, una vez se
decida que datos del andador cruzar (ver mocap_eval/posture_metrics.py).

Uso:
    python3 compute_posture_from_mocap.py --bag $DATASETS/labeled_bags/CA_test01
    python3 compute_posture_from_mocap.py --bags-dir $DATASETS/labeled_bags \\
        --out /tmp/mocap_posture_results.json
"""
import argparse
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mocap_eval import bag_io, contact_events, posture_metrics  # noqa: E402

LEAN_KEYS = ("l_shoulder", "r_shoulder", "l_hip", "r_hip")


def _mean_std(values):
    vals = [v for v in values if not (isinstance(v, float) and math.isnan(v))]
    if not vals:
        return float("nan"), float("nan")
    if len(vals) == 1:
        return vals[0], float("nan")
    return statistics.mean(vals), statistics.stdev(vals)


def compute_bag(bag_dir, speed_threshold, min_gap_s, max_dt_ns, use_tags=True):
    data = bag_io.load_bag(
        bag_dir, topics=("/labeled_markers", "/rigid_bodies"), require_all=True)
    mk_msgs, rb_msgs = data["/labeled_markers"], data["/rigid_bodies"]
    if not mk_msgs or not rb_msgs:
        return None

    if use_tags:
        tag_window = bag_io.read_tag_window(bag_dir)
        if tag_window is not None:
            t0, t1 = tag_window
            mk_msgs = bag_io.clip_track(mk_msgs, t0, t1)
            rb_msgs = bag_io.clip_track(rb_msgs, t0, t1)
        else:
            print(f"  [!] {bag_dir.name}: sin 2 mensajes /tag, usando el bag completo", file=sys.stderr)
        if not mk_msgs or not rb_msgs:
            return None

    walker_track = bag_io.rigid_body_track_xy_yaw(rb_msgs)

    tracks = {k: bag_io.marker_track(mk_msgs, k) for k in LEAN_KEYS + ("l_ankle", "r_ankle")}

    # --- Inclinacion de tronco ---
    lean = posture_metrics.trunk_lean_series(
        tracks["l_shoulder"], tracks["r_shoulder"], tracks["l_hip"], tracks["r_hip"], max_dt_ns)
    lean_total_mean, lean_total_std = _mean_std([r[1] for r in lean])
    lean_sag_mean, lean_sag_std = _mean_std([r[2] for r in lean])
    lean_fro_mean, lean_fro_std = _mean_std([r[3] for r in lean])

    # --- Eventos/intervalos de apoyo (reutilizados de gait_metrics/punto 1) ---
    l_events = contact_events.detect_stance_events(tracks["l_ankle"], speed_threshold, min_gap_s)
    r_events = contact_events.detect_stance_events(tracks["r_ankle"], speed_threshold, min_gap_s)
    l_intervals = contact_events.detect_stance_intervals(tracks["l_ankle"], speed_threshold)
    r_intervals = contact_events.detect_stance_intervals(tracks["r_ankle"], speed_threshold)

    if len(l_events) < 2 or len(r_events) < 2:
        return None

    # --- Ancho de paso ---
    widths = posture_metrics.step_width_m(l_events, r_events, walker_track, max_dt_ns, max_dt_ns)
    width_mean, width_std = _mean_std([w for _, w in widths])

    # --- Doble apoyo ---
    dsp = posture_metrics.double_support_fraction(l_intervals, r_intervals)

    # --- Margen de estabilidad lateral, en cada evento de apoyo ---
    hip_mid_track = []
    for t, l_hip in tracks["l_hip"]:
        r_hip = bag_io.nearest(tracks["r_hip"], t, max_dt_ns)
        if r_hip is not None:
            hip_mid_track.append((t, posture_metrics.midpoint(l_hip, r_hip[1])))

    all_events = sorted(l_events + r_events, key=lambda r: r[0])
    margins = [
        posture_metrics.lateral_margin_at_event(
            t, hip_mid_track, tracks["l_ankle"], tracks["r_ankle"], walker_track, max_dt_ns, max_dt_ns)
        for t, _ in all_events
    ]
    margin_mean, margin_std = _mean_std(margins)

    return {
        "trunk_lean_total_deg_mean": lean_total_mean, "trunk_lean_total_deg_std": lean_total_std,
        "trunk_lean_sagittal_deg_mean": lean_sag_mean, "trunk_lean_sagittal_deg_std": lean_sag_std,
        "trunk_lean_frontal_deg_mean": lean_fro_mean, "trunk_lean_frontal_deg_std": lean_fro_std,
        "step_width_m_mean": width_mean, "step_width_m_std": width_std,
        "double_support_fraction": dsp,
        "lateral_stability_margin_mean": margin_mean, "lateral_stability_margin_std": margin_std,
        "n_left_steps": len(l_events), "n_right_steps": len(r_events),
    }


def fmt(v, nd=3):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "-"
    return f"{v:.{nd}f}"


def print_table(results):
    cols = ["bag", "lean_deg", "lean_sag", "lean_fro", "step_w_m", "dsp_%", "lat_margin", "n_steps"]
    rows = []
    for bag_name, r in results.items():
        if r is None:
            rows.append([bag_name] + ["-"] * (len(cols) - 1))
            continue
        rows.append([
            bag_name, fmt(r["trunk_lean_total_deg_mean"], 1), fmt(r["trunk_lean_sagittal_deg_mean"], 1),
            fmt(r["trunk_lean_frontal_deg_mean"], 1), fmt(r["step_width_m_mean"]),
            fmt(100 * r["double_support_fraction"], 1), fmt(r["lateral_stability_margin_mean"]),
            r["n_left_steps"] + r["n_right_steps"],
        ])
    widths = [max(len(str(row[i])) for row in ([cols] + rows)) for i in range(len(cols))]
    line = "  ".join(str(c).ljust(w) for c, w in zip(cols, widths))
    print(line)
    print("-" * len(line))
    for row in rows:
        print("  ".join(str(c).ljust(w) for c, w in zip(row, widths)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag", help="un unico bag etiquetado")
    src.add_argument("--bags-dir", help="carpeta con varios bags etiquetados")
    ap.add_argument("--speed-threshold", type=float, default=0.25)
    ap.add_argument("--min-gap-s", type=float, default=0.3)
    ap.add_argument("--max-dt-ms", type=float, default=50.0)
    ap.add_argument("--ignore-tags", action="store_true",
                     help="usar el bag completo en vez de recortar a la ventana entre los 2 mensajes /tag")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    bag_dirs = [Path(args.bag)] if args.bag else bag_io.find_labeled_bags(args.bags_dir)
    if not bag_dirs:
        sys.exit(f"No se encontraron bags etiquetados en {args.bags_dir}")

    max_dt_ns = int(args.max_dt_ms * 1e6)
    results = {}
    for bag_dir in bag_dirs:
        try:
            results[bag_dir.name] = compute_bag(
                bag_dir, args.speed_threshold, args.min_gap_s, max_dt_ns,
                use_tags=not args.ignore_tags)
        except ValueError as e:
            print(f"[!] {bag_dir}: {e}", file=sys.stderr)
            results[bag_dir.name] = None

    print()
    print_table(results)

    if args.out:
        with open(args.out, "w") as f:
            json.dump(results, f, indent=2, ensure_ascii=False)
        print(f"\nGuardado en {args.out}")


if __name__ == "__main__":
    main()
