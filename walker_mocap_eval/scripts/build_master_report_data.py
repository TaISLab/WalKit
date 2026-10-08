#!/usr/bin/env python3
"""Fusiona los JSON de resultados (gait, posture, cog, extra, comparison, y
los 3 de la ampliacion: imu, handle-symmetry, lean-vs-margin) en un unico
master JSON indexado por bag, listo para generar el informe LaTeX. No
recalcula nada: solo junta lo que ya han producido los otros scripts de
walker_mocap_eval.

Uso:
    python3 build_master_report_data.py \\
        --gait /tmp/report_gait_mocap.json --posture /tmp/report_posture_mocap.json \\
        --cog /tmp/report_cog_mocap.json --extra /tmp/report_extra_mocap.json \\
        --comparison /tmp/report_comparison.json \\
        --imu /tmp/report_imu_mocap.json --handle-symmetry /tmp/report_handle_symmetry.json \\
        --lean-vs-margin /tmp/report_lean_vs_margin.json --out /tmp/report_master.json
"""
import argparse
import csv
import json


def load(path):
    with open(path) as f:
        return json.load(f)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gait", required=True)
    ap.add_argument("--posture", required=True)
    ap.add_argument("--cog", required=True)
    ap.add_argument("--extra", required=True)
    ap.add_argument("--comparison", required=True)
    ap.add_argument("--imu", default=None, help="report_imu_mocap.json (punto 1 de la ampliacion)")
    ap.add_argument("--handle-symmetry", default=None, help="report_handle_symmetry.json (punto 2)")
    ap.add_argument("--lean-vs-margin", default=None, help="report_lean_vs_margin.json (punto 5)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--out-csv", default=None)
    args = ap.parse_args()

    gait = load(args.gait)
    posture = load(args.posture)
    cog = load(args.cog)
    extra = load(args.extra)
    comparison = load(args.comparison)
    comp_gait = comparison["per_bag"]["gait"]
    comp_cog = comparison["per_bag"]["cog"]

    imu = load(args.imu) if args.imu else {}
    handle_sym = load(args.handle_symmetry)["per_bag"] if args.handle_symmetry else {}
    lean_margin = load(args.lean_vs_margin)["per_bag"] if args.lean_vs_margin else {}

    bag_names = sorted(k for k in gait.keys() if not k.startswith("_"))
    master = {}
    for name in bag_names:
        master[name] = {
            "gait_mocap": gait.get(name),
            "posture_mocap": posture.get(name),
            "cog_mocap": cog.get(name),
            "extra_mocap": extra.get(name),
            "gait_comparison": comp_gait.get(name),
            "cog_comparison": comp_cog.get(name),
            "imu_stability": imu.get(name),
            "handle_symmetry": handle_sym.get(name),
            "lean_vs_margin": lean_margin.get(name),
        }

    master["_aggregate_comparison"] = comparison["aggregate"]
    if args.handle_symmetry:
        master["_handle_symmetry_correlations"] = load(args.handle_symmetry)["correlations"]
    if args.lean_vs_margin:
        master["_lean_vs_margin_correlation"] = load(args.lean_vs_margin)["correlation"]

    with open(args.out, "w") as f:
        json.dump(master, f, indent=2, ensure_ascii=False)
    print(f"Guardado {args.out} ({len(bag_names)} bags)")

    if args.out_csv:
        flat_rows = []
        for name in bag_names:
            m = master[name]
            row = {"bag": name}
            for prefix, d in [("gait", m["gait_mocap"]), ("posture", m["posture_mocap"]),
                               ("cog", m["cog_mocap"]), ("extra", m["extra_mocap"]),
                               ("imu", m["imu_stability"]), ("handle_sym", m["handle_symmetry"]),
                               ("lean_margin", m["lean_vs_margin"])]:
                if d:
                    for k, v in d.items():
                        row[f"{prefix}_{k}"] = v
            if m["gait_comparison"]:
                for k, v in m["gait_comparison"].items():
                    row[f"cmp_gait_{k}_mocap"] = v["mocap"]
                    row[f"cmp_gait_{k}_walker"] = v["walker"]
                    row[f"cmp_gait_{k}_abs_err"] = v["abs_err"]
                    row[f"cmp_gait_{k}_pct_err"] = v["pct_err"]
            if m["cog_comparison"]:
                for axis in ("x", "y", "z", "planar_xy", "euclid_3d_not_comparable"):
                    st = m["cog_comparison"].get(axis)
                    if st:
                        for sk, sv in st.items():
                            row[f"cmp_cog_{axis}_{sk}"] = sv
            flat_rows.append(row)
        all_keys = []
        seen = set()
        for row in flat_rows:
            for k in row:
                if k not in seen:
                    seen.add(k)
                    all_keys.append(k)
        with open(args.out_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=all_keys)
            w.writeheader()
            w.writerows(flat_rows)
        print(f"Guardado {args.out_csv} (tabla plana, {len(all_keys)} columnas)")


if __name__ == "__main__":
    main()
