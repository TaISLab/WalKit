#!/usr/bin/env python3
"""Compara la salida real de las variantes de walker_step_detector (grabada
con `ros2 bag record` mientras corre launch/test_step_detector_variants.launch.py o launch/test_step_detector.launch.py)
contra el ground truth de mocap (tobillos de /labeled_markers), usando la
calibracion de calibrate_mocap_to_laser.py.

Que hay que grabar: al menos /labeled_markers, /rigid_bodies, y los topics
de salida de cada variante que quieras evaluar
(/detected_step_<variante>_left, /detected_step_<variante>_right --
walker_msgs/msg/StepStamped). El launcher ya nombra las tres variantes como
rf (Random Forest), km (k-means) y seg (segmentos), pero este script
autodetecta cualquier prefijo /detected_step_*_{left,right} presente en el
bag, no asume esos tres nombres.

Metricas por variante y lado (left/right, es decir la SALIDA del nodo, no
necesariamente el pie real):
  - n / n_matched / match_rate: de las detecciones publicadas, cuantas
    caen a <= --gate metros de alguno de los dos tobillos reales (mas
    cerca).
  - mean/median/p90 error: distancia (m) al tobillo mas cercano, solo
    entre las que hicieron match.
  - dominant_label / dominant_frac: a que tobillo real (l_ankle o r_ankle)
    esta mas cerca la mayoria de las veces -- si "left" del detector
    corresponde de verdad a l_ankle la mayor parte del tiempo, o esta
    sistematicamente cruzado.
  - swaps / swap_rate: cuantas veces, detection a detection, el tobillo
    real mas cercano cambia de identidad (l_ankle <-> r_ankle) -- proxy de
    "track swap" (ver legs_tracker.cpp: la asignacion izq/derecha no tiene
    memoria temporal, asi que un swap_rate alto es la firma esperada de
    ese problema).

Uso:
    python3 eval_against_mocap.py --eval-bag /tmp/eval_out \
        --calib ../config/mocap_to_base_footprint.yaml
"""
import argparse
import json
import math
import re
import statistics
import sys
from collections import Counter, defaultdict
from pathlib import Path

from mocap_bag_io import (
    FOOT_KEYS, bag_topics, load_bag, load_calib, markers_by_name, nearest,
    project_map_to_laser, rigid_body_pose_xy_yaw,
)

# /detected_step_<variante>_{left,right} (las del launcher de variantes) o,
# sin nombre de variante, /detected_step_{left,right} (lo que publica
# step_detector.launch.py en el robot: se evalua como "step_detector").
VARIANT_RE = re.compile(r"^(/detected_step(?:_.+)?)_(left|right)$")
UNNAMED_VARIANT = "step_detector"


def discover_variants(bag_dir):
    """{prefijo: {"left": topic_o_None, "right": topic_o_None}}"""
    topics = bag_topics(bag_dir)
    variants = defaultdict(lambda: {"left": None, "right": None})
    for name in topics:
        m = VARIANT_RE.match(name)
        if m:
            variants[m.group(1)][m.group(2)] = name
    return variants


def label_stream(msgs, rbs, mks, calib, gate, max_dt_ns):
    """Por cada deteccion: (t, dist_al_mas_cercano, 'l'|'r', matched)."""
    out = []
    for t, msg in msgs:
        rb = nearest(rbs, t, max_dt_ns)
        mk = nearest(mks, t, max_dt_ns)
        if rb is None or mk is None:
            continue
        rb_x, rb_y, rb_yaw = rigid_body_pose_xy_yaw(rb[1])
        feet = markers_by_name(mk[1], FOOT_KEYS)
        if "l_ankle" not in feet or "r_ankle" not in feet:
            continue
        lx, ly = project_map_to_laser(feet["l_ankle"][0], feet["l_ankle"][1], rb_x, rb_y, rb_yaw, calib)
        rx, ry = project_map_to_laser(feet["r_ankle"][0], feet["r_ankle"][1], rb_x, rb_y, rb_yaw, calib)
        px, py = msg.position.point.x, msg.position.point.y
        dl = math.hypot(px - lx, py - ly)
        dr = math.hypot(px - rx, py - ry)
        label, dist = ("l", dl) if dl <= dr else ("r", dr)
        out.append((t, dist, label, dist <= gate))
    return out


def summarize(records):
    n = len(records)
    matched = [r for r in records if r[3]]
    n_matched = len(matched)
    dists = [r[1] for r in matched]
    labels = [r[2] for r in matched]
    swaps = sum(1 for a, b in zip(labels, labels[1:]) if a != b)
    dominant_label, dominant_n = Counter(labels).most_common(1)[0] if labels else (None, 0)
    duration_s = (matched[-1][0] - matched[0][0]) / 1e9 if len(matched) > 1 else 0.0

    def q(vals, p):
        if len(vals) < 2:
            return float("nan")
        return statistics.quantiles(vals, n=100)[p - 1]

    return {
        "n_detections": n,
        "n_matched": n_matched,
        "match_rate": n_matched / n if n else float("nan"),
        "mean_err_m": statistics.mean(dists) if dists else float("nan"),
        "median_err_m": statistics.median(dists) if dists else float("nan"),
        "p90_err_m": q(dists, 90) if len(dists) >= 10 else float("nan"),
        "dominant_label": dominant_label,
        "dominant_frac": dominant_n / n_matched if n_matched else float("nan"),
        "swaps": swaps,
        "swap_rate_per_100": 100 * swaps / (n_matched - 1) if n_matched > 1 else float("nan"),
        "swaps_per_min": 60 * swaps / duration_s if duration_s > 0 else float("nan"),
        "duration_s": duration_s,
    }


def fmt(v, nd=3):
    if v is None:
        return "-"
    if isinstance(v, float) and math.isnan(v):
        return "-"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def print_table(results):
    cols = ["variant", "side", "n_det", "n_match", "match%", "mean_err", "median_err",
            "p90_err", "dom_label", "dom_frac", "swaps", "swap%/100", "swaps/min"]
    rows = []
    for (variant, side), s in results.items():
        rows.append([
            variant, side, s["n_detections"], s["n_matched"],
            fmt(100 * s["match_rate"], 1), fmt(s["mean_err_m"]), fmt(s["median_err_m"]),
            fmt(s["p90_err_m"]), s["dominant_label"] or "-", fmt(s["dominant_frac"], 2),
            s["swaps"], fmt(s["swap_rate_per_100"], 1), fmt(s["swaps_per_min"], 2),
        ])
    widths = [max(len(str(r[i])) for r in ([cols] + rows)) for i in range(len(cols))]
    line = "  ".join(str(c).ljust(w) for c, w in zip(cols, widths))
    print(line)
    print("-" * len(line))
    for r in rows:
        print("  ".join(str(c).ljust(w) for c, w in zip(r, widths)))


def evaluate_bag(eval_bag, calib, gate, max_dt_ns, quiet=False):
    """Un bag grabado -> {(variante, lado): records_crudos}. records_crudos es
    la lista (t, dist, label, matched) de label_stream(), SIN resumir --
    pensado para que run_multibag_eval.py pueda concatenar los de varios
    bags y resumir una sola vez sobre el conjunto (agregar medias/medianas
    ya calculadas por bag no es correcto estadisticamente)."""
    variants = discover_variants(eval_bag)
    if not variants:
        if not quiet:
            print(f"[!] {eval_bag}: sin topics /detected_step_*_{{left,right}}", file=sys.stderr)
        return {}

    try:
        gt = load_bag(eval_bag, topics=("/rigid_bodies", "/labeled_markers"), require_all=True)
    except ValueError as e:
        if not quiet:
            print(f"[!] {eval_bag}: {e}", file=sys.stderr)
        return {}
    rbs, mks = gt["/rigid_bodies"], gt["/labeled_markers"]
    if not rbs or not mks:
        if not quiet:
            print(f"[!] {eval_bag}: sin /rigid_bodies o /labeled_markers", file=sys.stderr)
        return {}

    out = {}
    for prefix, sides in sorted(variants.items()):
        variant_name = prefix[len("/detected_step"):].lstrip("_") or UNNAMED_VARIANT
        for side_name, topic in sides.items():
            if topic is None:
                continue
            data = load_bag(eval_bag, topics=(topic,), require_all=False)
            msgs = data.get(topic, [])
            if not msgs:
                continue
            out[(variant_name, side_name)] = label_stream(msgs, rbs, mks, calib, gate, max_dt_ns)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--eval-bag", required=True, help="bag grabado con ros2 bag record durante test_step_detector[_variants].launch.py")
    ap.add_argument("--calib", default=str(Path(__file__).resolve().parent.parent / "config" / "mocap_to_base_footprint.yaml"))
    ap.add_argument("--gate", type=float, default=0.3, help="distancia maxima (m) para considerar una deteccion valida")
    ap.add_argument("--max-dt-ms", type=float, default=50.0, help="tolerancia de emparejado deteccion<->ground truth")
    ap.add_argument("--out", default=None, help="JSON de salida con las metricas")
    args = ap.parse_args()

    calib = load_calib(args.calib)
    print(f"Calibracion: dx={calib[0]:+.3f} dy={calib[1]:+.3f} dyaw={math.degrees(calib[2]):+.2f} deg  ({args.calib})")

    max_dt_ns = int(args.max_dt_ms * 1e6)
    per_stream = evaluate_bag(args.eval_bag, calib, args.gate, max_dt_ns)
    if not per_stream:
        sys.exit(f"No se encontraron topics /detected_step_*_{{left,right}} o ground truth en {args.eval_bag}")

    results = {key: summarize(records) for key, records in per_stream.items()}

    print()
    print_table(results)

    if args.out:
        out = {f"{v}/{s}": r for (v, s), r in results.items()}
        out["_meta"] = {"eval_bag": args.eval_bag, "calib": args.calib, "gate_m": args.gate, "max_dt_ms": args.max_dt_ms}
        with open(args.out, "w") as f:
            json.dump(out, f, indent=2, ensure_ascii=False)
        print(f"\nGuardado en {args.out}")


if __name__ == "__main__":
    main()
