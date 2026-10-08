#!/usr/bin/env python3
"""Punto 5 de la ampliacion del informe: correlacion entre inclinacion de
tronco (sagital, hacia delante) y margen de estabilidad lateral -- ambos ya
calculados por compute_posture_from_mocap.py (report_posture_mocap.json),
sin necesidad de ningun calculo ni sensor nuevo. La literatura de andadores
instrumentados documenta que inclinarse hacia delante sobre el andador
aumenta la estabilidad ESTATICA pero reduce la capacidad de respuesta ante
perturbaciones (ver referencias en el informe); esto formaliza esa relacion
con los datos ya disponibles (no mide respuesta a perturbaciones, que no
esta en este dataset -- solo la correlacion estatica inclinacion/margen).

Uso:
    python3 compute_lean_vs_margin_correlation.py \\
        --posture-json /tmp/report_posture_mocap.json --out /tmp/report_lean_vs_margin.json
"""
import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mocap_eval import correlation_utils  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--posture-json", required=True)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    posture_all = json.load(open(args.posture_json))
    bags = sorted(b for b in posture_all if not b.startswith("_") and posture_all[b] is not None)

    lean = {b: posture_all[b]["trunk_lean_sagittal_deg_mean"] for b in bags}
    margin = {b: posture_all[b]["lateral_stability_margin_mean"] for b in bags}

    print(f"\n{'bag':12s} {'lean_sagital':>12s} {'margen_lateral':>14s}")
    for b in bags:
        print(f"{b:12s} {lean[b]:12.2f} {margin[b]:14.3f}")

    by_group = correlation_utils.correlation_by_group(bags, lean, margin)
    print("\n=== Correlacion de Pearson: inclinacion sagital de tronco vs margen de estabilidad lateral, por sujeto ===")
    for g, (r, n) in by_group.items():
        print(f"  {g:10s} r={r:+.3f} (n={n})" if not math.isnan(r) else f"  {g:10s} r=-- (n={n})")

    if args.out:
        out = {"per_bag": {b: {"trunk_lean_sagittal_deg_mean": lean[b],
                                "lateral_stability_margin_mean": margin[b]} for b in bags},
               "correlation": by_group}
        with open(args.out, "w") as f:
            json.dump(out, f, indent=2, ensure_ascii=False)
        print(f"\nGuardado en {args.out}")


if __name__ == "__main__":
    main()
