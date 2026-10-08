#!/usr/bin/env python3
"""Punto 2 de la ampliacion del informe: indice de simetria de marcha a
partir de la fuerza en las manetas (sensor real, sin necesidad de mocap),
validado contra la asimetria cinematica de paso que ya calcula mocap
(SpT/SdT/SpL/SdL izquierda-derecha). Formula: Symmetry Index (Robinson et
al. 1987) -- ver mocap_eval/extra_metrics.py::symmetry_index().

Post-procesa dos JSON ya calculados (no lee bags): report_extra_mocap.json
(handle_left_kg_mean/handle_right_kg_mean, de la fuerza calibrada real) y
report_gait_mocap.json (left_*/right_* de SpT/SdT/SpL/SdL, de mocap). El
SI de maneta se calcula sobre la fuerza MEDIA de cada lado (no la media de
un SI por muestra): la fuerza calibrada puede dar valores puntuales
ligeramente negativos cerca de carga nula (extrapolacion de la curva de
calibracion fuera de rango), lo que distorsiona un SI por muestra
(denominador casi nulo o negativo); promediar la fuerza primero y calcular
un unico SI sobre esa media evita el artefacto.

Uso:
    python3 compute_handle_symmetry_vs_gait.py \\
        --extra-json /tmp/report_extra_mocap.json --gait-json /tmp/report_gait_mocap.json \\
        --out /tmp/report_handle_symmetry.json
"""
import argparse
import json
import math

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mocap_eval import correlation_utils, extra_metrics  # noqa: E402

GAIT_SI_PAIRS = [
    ("spt", "left_spt", "right_spt"),
    ("sdt", "left_sdt", "right_sdt"),
    ("spl", "left_spl", "right_spl"),
    ("sdl", "left_sdl", "right_sdl"),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--extra-json", required=True)
    ap.add_argument("--gait-json", required=True)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    extra_all = json.load(open(args.extra_json))
    gait_all = json.load(open(args.gait_json))
    bags = sorted(b for b in extra_all if not b.startswith("_") and extra_all[b] is not None
                  and gait_all.get(b) is not None)

    results = {}
    for b in bags:
        e, g = extra_all[b], gait_all[b]
        out = {"handle_si_pct": extra_metrics.symmetry_index(e["handle_left_kg_mean"], e["handle_right_kg_mean"]),
               "handle_left_kg_mean": e["handle_left_kg_mean"], "handle_right_kg_mean": e["handle_right_kg_mean"]}
        for key, lf, rf in GAIT_SI_PAIRS:
            out[f"gait_si_{key}_pct"] = extra_metrics.symmetry_index(g[lf], g[rf])
        results[b] = out

    handle_si = {b: results[b]["handle_si_pct"] for b in bags}

    print(f"\n{'bag':12s} {'handle_SI%':>10s} {'gait_SI_spt%':>12s} {'gait_SI_sdt%':>12s} "
          f"{'gait_SI_spl%':>12s} {'gait_SI_sdl%':>12s}")
    for b in bags:
        r = results[b]
        print(f"{b:12s} {r['handle_si_pct']:10.1f} {r['gait_si_spt_pct']:12.1f} "
              f"{r['gait_si_sdt_pct']:12.1f} {r['gait_si_spl_pct']:12.1f} {r['gait_si_sdl_pct']:12.1f}")

    print("\n=== Correlacion de Pearson: SI de maneta (real) vs SI cinematico (mocap), por sujeto ===")
    correlations = {}
    for key, _, _ in GAIT_SI_PAIRS:
        gait_si = {b: results[b][f"gait_si_{key}_pct"] for b in bags}
        by_group = correlation_utils.correlation_by_group(bags, handle_si, gait_si)
        correlations[key] = by_group
        print(f"\n  vs gait_si_{key}:")
        for g, (r, n) in by_group.items():
            print(f"    {g:10s} r={r:+.3f} (n={n})" if not math.isnan(r) else f"    {g:10s} r=-- (n={n})")

    if args.out:
        with open(args.out, "w") as f:
            json.dump({"per_bag": results, "correlations": correlations}, f, indent=2, ensure_ascii=False)
        print(f"\nGuardado en {args.out}")


if __name__ == "__main__":
    main()
