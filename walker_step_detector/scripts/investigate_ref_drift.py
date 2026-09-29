#!/usr/bin/env python3
"""Para cada swap real de rf (ver investigate_swaps.py: 99.8% ocurren con las
piernas claramente separadas, no cruzadas), comprueba si la REFERENCIA
INTERNA del tracker (LegsTracker::get_current_refs, publicada en
/detected_step_rf_{left,right}_ref -- la prediccion del EKF ANTES de que
add_detections() decida este ciclo) ya estaba mas cerca del tobillo
equivocado antes de que ocurriera el swap.

  - "deriva previa": la referencia interna, ANTES de este ciclo, ya estaba
    mas cerca del tobillo que la deteccion final "roba" (el problema es que
    la prediccion del EKF se habia desviado -- ajuste del filtro).
  - "referencia correcta": la referencia interna seguia mas cerca del
    tobillo correcto -- el swap ocurrio pese a una referencia sana (otra
    causa: una candidata mejor "robada" por el emparejamiento exclusivo,
    o un fallo puntual de la propia deteccion).

Requiere bags grabados con run_multibag_eval.py --keep-bags, incluyendo
/detected_step_rf_{left,right}_ref.

Uso:
    python3 investigate_ref_drift.py --bags-dir /tmp/multibag_eval_swaps
"""
import argparse
import math
import sys
from pathlib import Path

from mocap_bag_io import FOOT_KEYS, find_labeled_bags, load_bag, load_calib, markers_by_name, nearest, project_map_to_laser, rigid_body_pose_xy_yaw

MAX_DT_NS = int(0.05 * 1e9)
MAX_REF_DT_NS = int(0.15 * 1e9)
GATE_M = 0.3


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
    return lx, ly, rx, ry


def build_records(msgs, rbs, mks, calib):
    out = []
    for t, msg in msgs:
        gt = gt_at(t, rbs, mks, calib)
        if gt is None:
            continue
        lx, ly, rx, ry = gt
        px, py = msg.position.point.x, msg.position.point.y
        dl = math.hypot(px - lx, py - ly)
        dr = math.hypot(px - rx, py - ry)
        label = "l" if dl <= dr else "r"
        out.append({"t": t, "px": px, "py": py, "dl": dl, "dr": dr, "label": label,
                    "matched": min(dl, dr) <= GATE_M, "lx": lx, "ly": ly, "rx": rx, "ry": ry})
    return out


def process_bag(bag_dir, calib, side):
    topic = f"/detected_step_rf_{side}"
    ref_topic = f"/detected_step_rf_{side}_ref"
    try:
        data = load_bag(bag_dir, topics=("/rigid_bodies", "/labeled_markers", topic, ref_topic), require_all=True)
    except ValueError as e:
        print(f"  [!] {bag_dir.name}/{side}: {e}", file=sys.stderr)
        return None
    rbs, mks, msgs, refs = data["/rigid_bodies"], data["/labeled_markers"], data[topic], data[ref_topic]
    if not rbs or not mks or not msgs or not refs:
        return None

    records = build_records(msgs, rbs, mks, calib)
    matched = [r for r in records if r["matched"]]
    if len(matched) < 3:
        return None

    counts = {"deriva_previa": 0, "referencia_correcta": 0}
    for i in range(1, len(matched)):
        if matched[i]["label"] == matched[i - 1]["label"]:
            continue
        # swap: antes cerca de matched[i-1]["label"], ahora cerca de matched[i]["label"]
        ref = nearest(refs, matched[i]["t"], MAX_REF_DT_NS)
        if ref is None:
            continue
        rx, ry = ref[1].position.point.x, ref[1].position.point.y
        lx, ly, rrx, rry = matched[i]["lx"], matched[i]["ly"], matched[i]["rx"], matched[i]["ry"]
        d_to_l = math.hypot(rx - lx, ry - ly)
        d_to_r = math.hypot(rx - rrx, ry - rry)
        ref_label = "l" if d_to_l <= d_to_r else "r"
        # la referencia "deriva" si ya apunta hacia el lado NUEVO (matched[i])
        # en vez de seguir apuntando al lado ANTERIOR (matched[i-1])
        if ref_label == matched[i]["label"]:
            counts["deriva_previa"] += 1
        else:
            counts["referencia_correcta"] += 1
    return counts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags-dir", required=True)
    ap.add_argument("--calib", default=str(Path(__file__).resolve().parent.parent / "config" / "mocap_to_base_footprint.yaml"))
    args = ap.parse_args()

    calib = load_calib(args.calib)
    bag_dirs = find_labeled_bags(args.bags_dir)
    print(f"{len(bag_dirs)} bags a analizar\n")

    total = {"deriva_previa": 0, "referencia_correcta": 0}
    for bag_dir in bag_dirs:
        bag_total = {"deriva_previa": 0, "referencia_correcta": 0}
        for side in ("left", "right"):
            r = process_bag(bag_dir, calib, side)
            if r is None:
                continue
            for k in bag_total:
                bag_total[k] += r[k]
        n = sum(bag_total.values())
        if n:
            print(f"  [{bag_dir.name}] swaps analizados={n}  deriva_previa={bag_total['deriva_previa']}  "
                  f"referencia_correcta={bag_total['referencia_correcta']}")
        for k in total:
            total[k] += bag_total[k]

    n_total = sum(total.values())
    print(f"\n=== Agregado: {n_total} swaps con referencia disponible ===")
    for k, v in total.items():
        print(f"  {k}: {v} ({100*v/n_total:.1f}%)" if n_total else f"  {k}: 0")


if __name__ == "__main__":
    main()
