#!/usr/bin/env python3
"""Investiga, caso a caso, que dispara los swaps de rf (el momento en que la
etiqueta "mas cercana" de una deteccion cambia de un tobillo al otro entre
dos detecciones consecutivas) -- para saber si de verdad coinciden con
cruces reales de piernas (ver validate_leg_coupling.py: los cruces son
raros, ~2.3% de los frames en promedio) o vienen de otra cosa.

Para cada swap, se clasifica el instante segun el ground truth de mocap:
  - "cruzado": las dos piernas estaban realmente cruzadas en ese instante
    (orden lateral invertido respecto al orden habitual del bag).
  - "piernas juntas": no cruzadas, pero la separacion lateral entre
    tobillos era pequena (<0.15 m) -- doble apoyo o casi, ambiguo aunque
    tecnicamente no cruzado.
  - "piernas separadas": ninguna de las dos -- las piernas estaban
    claramente separadas y no cruzadas, asi que el swap no tiene una causa
    cinematica visible; hay que mirar el propio error de deteccion (salto,
    ruido) para explicarlo.

Requiere bags grabados con run_multibag_eval.py --keep-bags (necesita
/detected_step_rf_{left,right} + ground truth).

Uso:
    python3 investigate_swaps.py --bags-dir /tmp/multibag_eval_swaps --examples 5
"""
import argparse
import math
import sys
from pathlib import Path

from mocap_bag_io import (
    FOOT_KEYS, find_labeled_bags, load_bag, load_calib, markers_by_name,
    nearest, project_map_to_laser, rigid_body_pose_xy_yaw,
)

MAX_DT_NS = int(0.05 * 1e9)
GATE_M = 0.3
TOGETHER_THRESHOLD_M = 0.15


def gt_at(t, rbs, mks, calib):
    rb = nearest(rbs, t, MAX_DT_NS)
    mk = nearest(mks, t, MAX_DT_NS)
    if rb is None or mk is None:
        return None
    rb_x, rb_y, rb_yaw = rigid_body_pose_xy_yaw(rb[1])
    feet = markers_by_name(mk[1], FOOT_KEYS)
    if "l_ankle" not in feet or "r_ankle" not in feet:
        return None
    lx, ly = project_map_to_laser(feet["l_ankle"][0], feet["l_ankle"][1], rb_x, rb_y, rb_yaw, calib)
    rx, ry = project_map_to_laser(feet["r_ankle"][0], feet["r_ankle"][1], rb_x, rb_y, rb_yaw, calib)
    return (lx, ly, rx, ry)


def build_records(msgs, rbs, mks, calib):
    """(t, px, py, dl, dr, label, matched, lx,ly,rx,ry) por deteccion."""
    out = []
    for t, msg in msgs:
        gt = gt_at(t, rbs, mks, calib)
        if gt is None:
            continue
        lx, ly, rx, ry = gt
        px, py = msg.position.point.x, msg.position.point.y
        dl = math.hypot(px - lx, py - ly)
        dr = math.hypot(px - rx, py - ry)
        label, dist = ("l", dl) if dl <= dr else ("r", dr)
        out.append({"t": t, "px": px, "py": py, "dl": dl, "dr": dr, "label": label,
                    "matched": dist <= GATE_M, "lx": lx, "ly": ly, "rx": rx, "ry": ry})
    return out


def classify_swap(rec_before, rec_after, majority_sign):
    lx, ly, rx, ry = rec_after["lx"], rec_after["ly"], rec_after["rx"], rec_after["ry"]
    lateral_diff = ly - ry
    step_width = math.hypot(lx - rx, ly - ry)
    crossed = (lateral_diff >= 0) != (majority_sign >= 0)
    if crossed:
        return "cruzado", step_width
    if step_width < TOGETHER_THRESHOLD_M:
        return "piernas juntas", step_width
    return "piernas separadas", step_width


def process_bag(bag_dir, calib, side, examples_out, max_examples):
    topic = f"/detected_step_rf_{side}"
    try:
        data = load_bag(bag_dir, topics=("/rigid_bodies", "/labeled_markers", topic), require_all=True)
    except ValueError as e:
        print(f"  [!] {bag_dir.name}: {e}", file=sys.stderr)
        return None
    rbs, mks, msgs = data["/rigid_bodies"], data["/labeled_markers"], data[topic]
    if not rbs or not mks or not msgs:
        return None

    records = build_records(msgs, rbs, mks, calib)
    matched = [r for r in records if r["matched"]]
    if len(matched) < 3:
        return None

    # signo lateral habitual del bag (para detectar cruces)
    diffs = [r["ly"] - r["ry"] for r in matched]
    majority_sign = 1.0 if sum(1 for d in diffs if d >= 0) >= len(diffs) / 2 else -1.0

    counts = {"cruzado": 0, "piernas juntas": 0, "piernas separadas": 0}
    for i in range(1, len(matched)):
        if matched[i]["label"] == matched[i - 1]["label"]:
            continue
        cls, width = classify_swap(matched[i - 1], matched[i], majority_sign)
        counts[cls] += 1
        if cls == "piernas separadas" and len(examples_out) < max_examples:
            r = matched[i]
            examples_out.append(
                f"{bag_dir.name} t={r['t']}: deteccion=({r['px']:.3f},{r['py']:.3f}) "
                f"l_ankle=({r['lx']:.3f},{r['ly']:.3f}) r_ankle=({r['rx']:.3f},{r['ry']:.3f}) "
                f"dl={r['dl']:.3f} dr={r['dr']:.3f} separacion_pies={width:.3f}m")
    return counts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags-dir", required=True)
    ap.add_argument("--calib", default=str(Path(__file__).resolve().parent.parent / "config" / "mocap_to_base_footprint.yaml"))
    ap.add_argument("--examples", type=int, default=8, help="cuantos ejemplos de 'piernas separadas' imprimir")
    args = ap.parse_args()

    calib = load_calib(args.calib)
    bag_dirs = find_labeled_bags(args.bags_dir)
    print(f"{len(bag_dirs)} bags a analizar\n")

    total = {"cruzado": 0, "piernas juntas": 0, "piernas separadas": 0}
    examples = []
    n_bags_ok = 0

    for bag_dir in bag_dirs:
        bag_total = {"cruzado": 0, "piernas juntas": 0, "piernas separadas": 0}
        for side in ("left", "right"):
            r = process_bag(bag_dir, calib, side, examples, args.examples)
            if r is None:
                continue
            for k in bag_total:
                bag_total[k] += r[k]
        n_swaps = sum(bag_total.values())
        if n_swaps:
            n_bags_ok += 1
            print(f"  [{bag_dir.name}] swaps={n_swaps}  cruzado={bag_total['cruzado']}  "
                  f"juntas={bag_total['piernas juntas']}  separadas={bag_total['piernas separadas']}")
        for k in total:
            total[k] += bag_total[k]

    n_total = sum(total.values())
    print(f"\n=== Agregado sobre {n_bags_ok} bags con swaps, {n_total} swaps totales (rf, ambos lados) ===")
    for k, v in total.items():
        print(f"  {k}: {v} ({100*v/n_total:.1f}%)" if n_total else f"  {k}: 0")

    if examples:
        print(f"\n=== Ejemplos de 'piernas separadas' (swap sin causa cinematica visible) ===")
        for e in examples:
            print(f"  {e}")


if __name__ == "__main__":
    main()
