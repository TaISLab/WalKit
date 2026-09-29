#!/usr/bin/env python3
"""Analisis quirurgico del respaldo km/segmentos en detect_steps_fused: aisla
SOLO los ciclos donde el respaldo se activo (RF dio <2 candidatos utiles,
marcado por /detected_step_fused_fallback_active) y compara, justo en esos
instantes, la precision de `fused` (con respaldo) contra la de `rf` solo (lo
que hubiera hecho sin el).

Requiere bags grabados con --keep-bags (run_multibag_eval.py) que incluyan
/detected_step_fused_fallback_active, /detected_step_fused_{left,right},
/detected_step_rf_{left,right} y el ground truth (/rigid_bodies,
/labeled_markers).

Uso:
    python3 analyze_fallback_gain.py --bags-dir /tmp/multibag_eval_fallback
"""
import argparse
import math
import statistics
import sys
from pathlib import Path

from mocap_bag_io import (
    FOOT_KEYS, find_labeled_bags, load_bag, load_calib, markers_by_name,
    nearest, project_map_to_laser, rigid_body_pose_xy_yaw,
)

FALLBACK_TOPIC = "/detected_step_fused_fallback_active"
MAX_FLAG_DT_NS = int(0.15 * 1e9)   # tolerancia fused<->flag (mismo ciclo, deberia ser ~0)
MAX_RF_DT_NS = int(0.25 * 1e9)     # tolerancia fused<->deteccion de rf (mismo scan, nodos distintos)


def nearest_dist_to_gt(px, py, rb, mk, calib):
    rb_x, rb_y, rb_yaw = rigid_body_pose_xy_yaw(rb)
    feet = markers_by_name(mk, FOOT_KEYS)
    if "l_ankle" not in feet or "r_ankle" not in feet:
        return None
    lx, ly = project_map_to_laser(feet["l_ankle"][0], feet["l_ankle"][1], rb_x, rb_y, rb_yaw, calib)
    rx, ry = project_map_to_laser(feet["r_ankle"][0], feet["r_ankle"][1], rb_x, rb_y, rb_yaw, calib)
    return min(math.hypot(px - lx, py - ly), math.hypot(px - rx, py - ry))


def process_bag(bag_dir, calib, gate, max_dt_ns):
    try:
        data = load_bag(bag_dir, topics=(
            "/rigid_bodies", "/labeled_markers", FALLBACK_TOPIC,
            "/detected_step_fused_left", "/detected_step_fused_right",
            "/detected_step_rf_left", "/detected_step_rf_right",
        ), require_all=True)
    except ValueError as e:
        print(f"  [!] {bag_dir.name}: {e}", file=sys.stderr)
        return None

    rbs, mks = data["/rigid_bodies"], data["/labeled_markers"]
    flags = data[FALLBACK_TOPIC]
    if not rbs or not mks or not flags:
        return None

    out = {"fallback_on": {"fused": [], "rf": []}, "fallback_off": {"fused": [], "rf": []}}
    n_cycles = len(flags)
    n_on = sum(1 for _, f in flags if f.data)

    for side, fused_topic, rf_topic in (
        ("left", "/detected_step_fused_left", "/detected_step_rf_left"),
        ("right", "/detected_step_fused_right", "/detected_step_rf_right"),
    ):
        for t, msg in data[fused_topic]:
            flag = nearest(flags, t, MAX_FLAG_DT_NS)
            rb = nearest(rbs, t, max_dt_ns)
            mk = nearest(mks, t, max_dt_ns)
            if flag is None or rb is None or mk is None:
                continue
            d = nearest_dist_to_gt(msg.position.point.x, msg.position.point.y, rb[1], mk[1], calib)
            if d is None:
                continue
            bucket = "fallback_on" if flag[1].data else "fallback_off"
            out[bucket]["fused"].append(d <= gate)

            if flag[1].data:
                # que hizo RF SOLO en ese mismo instante (lo que hubiera
                # publicado sin el respaldo de km/segmentos)
                rf_msg = nearest(data[rf_topic], t, MAX_RF_DT_NS)
                if rf_msg is not None:
                    d_rf = nearest_dist_to_gt(rf_msg[1].position.point.x, rf_msg[1].position.point.y, rb[1], mk[1], calib)
                    if d_rf is not None:
                        out["fallback_on"]["rf"].append(d_rf <= gate)

    out["n_cycles"] = n_cycles
    out["n_fallback_cycles"] = n_on
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags-dir", required=True, help="carpeta con los bags grabados (run_multibag_eval.py --keep-bags)")
    ap.add_argument("--calib", default=str(Path(__file__).resolve().parent.parent / "config" / "mocap_to_base_footprint.yaml"))
    ap.add_argument("--gate", type=float, default=0.3)
    ap.add_argument("--max-dt-ms", type=float, default=50.0)
    args = ap.parse_args()

    calib = load_calib(args.calib)
    max_dt_ns = int(args.max_dt_ms * 1e6)

    bag_dirs = find_labeled_bags(args.bags_dir)
    print(f"{len(bag_dirs)} bags a analizar\n")

    all_on_fused, all_on_rf, all_off_fused = [], [], []
    total_cycles, total_fallback_cycles = 0, 0

    for bag_dir in bag_dirs:
        r = process_bag(bag_dir, calib, args.gate, max_dt_ns)
        if r is None:
            print(f"  [-] {bag_dir.name}: sin datos suficientes")
            continue
        pct = 100 * r["n_fallback_cycles"] / r["n_cycles"] if r["n_cycles"] else 0
        n_on = len(r["fallback_on"]["fused"])
        print(f"  [{bag_dir.name}] respaldo activo en {r['n_fallback_cycles']}/{r['n_cycles']} ciclos ({pct:.1f}%), "
              f"{n_on} detecciones comparables")
        all_on_fused.extend(r["fallback_on"]["fused"])
        all_on_rf.extend(r["fallback_on"]["rf"])
        all_off_fused.extend(r["fallback_off"]["fused"])
        total_cycles += r["n_cycles"]
        total_fallback_cycles += r["n_fallback_cycles"]

    print(f"\n=== Agregado ===")
    print(f"Ciclos con respaldo activo: {total_fallback_cycles}/{total_cycles} "
          f"({100*total_fallback_cycles/total_cycles:.1f}%)" if total_cycles else "sin ciclos")

    def rate(vals):
        return (100 * sum(vals) / len(vals), len(vals)) if vals else (float("nan"), 0)

    on_fused_rate, n1 = rate(all_on_fused)
    on_rf_rate, n2 = rate(all_on_rf)
    off_fused_rate, n3 = rate(all_off_fused)

    print(f"\nDurante el respaldo (RF <2 candidatos):")
    print(f"  fused (con respaldo):  match={on_fused_rate:.1f}%  (n={n1})")
    print(f"  rf solo (sin respaldo, lo que hubiera publicado): match={on_rf_rate:.1f}%  (n={n2})")
    print(f"  diferencia: {on_fused_rate - on_rf_rate:+.1f} puntos" if n1 and n2 else "")
    print(f"\nFuera del respaldo (RF ya tenia >=2 candidatos), referencia:")
    print(f"  fused: match={off_fused_rate:.1f}%  (n={n3})")


if __name__ == "__main__":
    main()
