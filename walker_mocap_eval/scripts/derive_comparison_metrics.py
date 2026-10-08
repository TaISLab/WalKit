#!/usr/bin/env python3
"""Deriva las metricas de comparacion mocap-vs-walker (puntos 1-2 y CoG) a
partir de los CSVs ya exportados por export_bag_timeseries.py y de los JSON
producidos por compute_gait_from_mocap.py / compute_cog_from_mocap.py, sin
necesidad de una nueva campana de reproduccion.

Punto 2 (gait): compara el resumen final (ultima fila = promedio acumulado
publicado por el walker) frente al resumen calculado sobre mocap para el
mismo bag (report_gait_mocap.json). Solo se consideran filas con
t_s <= fin de la ventana /tag: export_bag_timeseries.py graba unos segundos
MAS alla del segundo /tag (espera duracion/rate + 3 s), y ese tramo
posterior (usuario ya parado) no corresponde a lo que mide mocap.

Punto CoG: interpola el mocap CoG en los instantes de muestra del walker
(nearest-neighbor, igual que bag_io.nearest) y calcula el error por eje y el
error euclidiano 3D en cada instante, dentro de la ventana /tag (se
descartan filas del walker posteriores al fin de la ventana: el mocap esta
recortado a ella, y el vecino mas cercano de un instante posterior seria
siempre la ultima muestra de mocap).

Formulas de error usadas (documentadas tambien en el informe):
    AE_i        = |walker_i - mocap_i|                       (error absoluto)
    MAE         = mean(AE_i)                                  (error absoluto medio)
    MedAE       = median(AE_i)                                (mediana del error absoluto)
    Bias        = mean(walker_i - mocap_i)                    (error con signo medio)
    RMSE        = sqrt(mean((walker_i - mocap_i)^2))           (raiz del error cuadratico medio)
    MAPE        = mean(|walker_i - mocap_i| / |mocap_i|) * 100 (solo si mocap_i != 0)
    MedAPE      = median(|walker_i - mocap_i| / |mocap_i|) * 100 (robusta a outliers)

Uso:
    python3 derive_comparison_metrics.py --bags-dir $DATASETS/labeled_bags \\
        --timeseries-dir /tmp/report_timeseries \\
        --gait-json /tmp/report_gait_mocap.json \\
        --out-json /tmp/report_comparison.json --out-csv-dir /tmp/report_timeseries
"""
import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from mocap_eval import bag_io  # noqa: E402

GAIT_PAIRS = [
    ("left_spt", "left_gait_stats", "spt_s"),
    ("left_sdt", "left_gait_stats", "sdt_s"),
    ("left_spl", "left_gait_stats", "spl_m"),
    ("left_sdl", "left_gait_stats", "sdl_m"),
    ("right_spt", "right_gait_stats", "spt_s"),
    ("right_sdt", "right_gait_stats", "sdt_s"),
    ("right_spl", "right_gait_stats", "spl_m"),
    ("right_sdl", "right_gait_stats", "sdl_m"),
    ("nos", "global_gait_stats", "nos"),
    ("d_m", "global_gait_stats", "d_m"),
    ("cad", "global_gait_stats", "cad"),
    ("wv", "global_gait_stats", "wv"),
]


def read_csv_rows(path):
    if not path.exists():
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def last_row_value(rows, key, t_max=None):
    """Valor de `key` en la ultima fila con t_s <= t_max (si t_max no es
    None); NaN si no hay ninguna."""
    if t_max is not None:
        rows = [r for r in rows if float(r["t_s"]) <= t_max]
    if not rows:
        return float("nan")
    try:
        return float(rows[-1][key])
    except (KeyError, ValueError):
        return float("nan")


def window_end_s(bag_dir):
    """Fin de la ventana /tag en segundos relativos a su inicio (t_s=0 en
    todos los CSV); None si el bag no tiene 2 mensajes /tag."""
    w = bag_io.read_tag_window(bag_dir)
    return None if w is None else (w[1] - w[0]) / 1e9


def err_stats(pairs):
    """pairs: list of (mocap, walker) float tuples. Returns dict of error formulas."""
    pairs = [(m, w) for m, w in pairs if not (math.isnan(m) or math.isnan(w))]
    if not pairs:
        return None
    ae = [abs(w - m) for m, w in pairs]
    signed = [w - m for m, w in pairs]
    sq = [(w - m) ** 2 for m, w in pairs]
    ape = [abs(w - m) / abs(m) * 100.0 for m, w in pairs if m != 0]
    return {
        "n": len(pairs),
        "mae": statistics.mean(ae),
        "medae": statistics.median(ae),
        "bias": statistics.mean(signed),
        "rmse": math.sqrt(statistics.mean(sq)),
        "mape_pct": statistics.mean(ape) if ape else float("nan"),
        "medape_pct": statistics.median(ape) if ape else float("nan"),
    }


def gait_comparison_for_bag(bag_name, gait_mocap, ts_dir, t_max=None):
    if gait_mocap is None:
        return None
    out = {}
    for mocap_key, csv_stub, csv_key in GAIT_PAIRS:
        rows = read_csv_rows(ts_dir / f"{bag_name}_walker_{csv_stub}.csv")
        walker_val = last_row_value(rows, csv_key, t_max)
        mocap_val = float(gait_mocap.get(mocap_key, float("nan")))
        out[mocap_key] = {"mocap": mocap_val, "walker": walker_val,
                           "abs_err": abs(walker_val - mocap_val) if not math.isnan(walker_val) else float("nan"),
                           "pct_err": (abs(walker_val - mocap_val) / abs(mocap_val) * 100.0
                                       if mocap_val not in (0, float("nan")) and not math.isnan(walker_val)
                                       else float("nan"))}
    return out


def cog_comparison_for_bag(bag_name, ts_dir, out_csv_dir=None, t_max=None):
    mocap_rows = read_csv_rows(ts_dir / f"{bag_name}_mocap_cog.csv")
    walker_rows = read_csv_rows(ts_dir / f"{bag_name}_walker_cog.csv")
    if not mocap_rows or not walker_rows:
        return None
    m_track = [(float(r["t_s"]), (float(r["x_m"]), float(r["y_m"]), float(r["z_m"])))
               for r in mocap_rows]
    query = bag_io.indexed_nearest(m_track)

    # NOTA IMPORTANTE: x/y de _mocap_cog.csv ya estan proyectados al frame
    # del andador ("laser", misma calibracion ICP que puntos 1/2), pero z NO
    # (ver docstring de export_bag_timeseries.py) -- y aunque lo estuviera,
    # /support_centroid es el centroide de la BASE DE SOPORTE (altura ~0,
    # plano del suelo), mientras que el CoM de mocap es el del CUERPO
    # COMPLETO (altura ~0.95-1.05 m): son magnitudes fisicas distintas por
    # construccion, no el mismo punto en distinto frame. Por eso aqui solo
    # se trata x/y (planas, comparables) como metrica principal; z/3D se
    # calculan igualmente pero se documentan como NO comparables.
    per_axis = {"x": [], "y": [], "z": []}
    planar = []
    euclid = []
    out_rows = []
    for r in walker_rows:
        t = float(r["t_s"])
        if t_max is not None and t > t_max:
            continue
        found = query(t)
        if found is None:
            continue
        mx, my, mz = found[1]
        wx, wy, wz = float(r["x_m"]), float(r["y_m"]), float(r["z_m"])
        per_axis["x"].append((mx, wx))
        per_axis["y"].append((my, wy))
        per_axis["z"].append((mz, wz))
        d2 = math.sqrt((wx - mx) ** 2 + (wy - my) ** 2)
        d3 = math.sqrt((wx - mx) ** 2 + (wy - my) ** 2 + (wz - mz) ** 2)
        planar.append(d2)
        euclid.append(d3)
        out_rows.append((t, mx, my, mz, wx, wy, wz, d2, d3))

    if out_csv_dir is not None and out_rows:
        with open(out_csv_dir / f"{bag_name}_cog_comparison.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "mocap_x_m", "mocap_y_m", "mocap_z_m",
                        "walker_x_m", "walker_y_m", "walker_z_m",
                        "planar_err_m", "euclid_err_m_not_comparable"])
            w.writerows(out_rows)

    result = {axis: err_stats(vals) for axis, vals in per_axis.items()}

    def _dist_stats(vals):
        if not vals:
            return None
        return {"n": len(vals), "mae": statistics.mean(vals), "medae": statistics.median(vals),
                "rmse": math.sqrt(statistics.mean(v * v for v in vals))}

    result["planar_xy"] = _dist_stats(planar)
    result["euclid_3d_not_comparable"] = _dist_stats(euclid)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags-dir", required=True)
    ap.add_argument("--timeseries-dir", required=True)
    ap.add_argument("--gait-json", required=True)
    ap.add_argument("--out-json", required=True)
    ap.add_argument("--out-csv-dir", default=None)
    args = ap.parse_args()

    ts_dir = Path(args.timeseries_dir)
    out_csv_dir = Path(args.out_csv_dir) if args.out_csv_dir else None
    gait_mocap_all = json.load(open(args.gait_json))
    bag_dirs = bag_io.find_labeled_bags(args.bags_dir)

    per_bag_gait, per_bag_cog = {}, {}
    for bag_dir in bag_dirs:
        name = bag_dir.name
        t_max = window_end_s(bag_dir)
        per_bag_gait[name] = gait_comparison_for_bag(name, gait_mocap_all.get(name), ts_dir, t_max)
        per_bag_cog[name] = cog_comparison_for_bag(name, ts_dir, out_csv_dir, t_max)

    # Agregados de gait (formula igual que err_stats, sobre todos los bags)
    agg_gait = {}
    for mocap_key, _, _ in GAIT_PAIRS:
        pairs = [(b[mocap_key]["mocap"], b[mocap_key]["walker"])
                 for b in per_bag_gait.values() if b is not None]
        agg_gait[mocap_key] = err_stats(pairs)

    # Agregados de CoG: releemos los CSV de comparacion por bag ya escritos.
    # x/y son la metrica principal (mismo frame, comparables); z/3D se
    # agregan igual pero NO son comparables (ver nota en cog_comparison_for_bag).
    agg_cog_pairs = {"x": [], "y": [], "z": []}
    agg_planar, agg_euclid = [], []

    def _dist_stats(vals):
        if not vals:
            return None
        return {"n": len(vals), "mae": statistics.mean(vals), "medae": statistics.median(vals),
                "rmse": math.sqrt(statistics.mean(v * v for v in vals))}

    for name in per_bag_cog:
        comp_csv = (out_csv_dir or ts_dir) / f"{name}_cog_comparison.csv"
        if not comp_csv.exists():
            continue
        for row in read_csv_rows(comp_csv):
            agg_cog_pairs["x"].append((float(row["mocap_x_m"]), float(row["walker_x_m"])))
            agg_cog_pairs["y"].append((float(row["mocap_y_m"]), float(row["walker_y_m"])))
            agg_cog_pairs["z"].append((float(row["mocap_z_m"]), float(row["walker_z_m"])))
            agg_planar.append(float(row["planar_err_m"]))
            agg_euclid.append(float(row["euclid_err_m_not_comparable"]))
    agg_cog = {axis: err_stats(pairs) for axis, pairs in agg_cog_pairs.items()}
    agg_cog["planar_xy"] = _dist_stats(agg_planar)
    agg_cog["euclid_3d_not_comparable"] = _dist_stats(agg_euclid)

    result = {
        "per_bag": {"gait": per_bag_gait, "cog": per_bag_cog},
        "aggregate": {"gait": agg_gait, "cog": agg_cog},
    }
    with open(args.out_json, "w") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print(f"Guardado en {args.out_json}")
    print("\n=== Agregado gait (todos los bags) ===")
    for k, v in agg_gait.items():
        if v:
            print(f"  {k:10s} MAE={v['mae']:.4f} MedAE={v['medae']:.4f} Bias={v['bias']:+.4f} "
                  f"RMSE={v['rmse']:.4f} MAPE={v['mape_pct']:.2f}% MedAPE={v['medape_pct']:.2f}% (n={v['n']})")
    print("\n=== Agregado CoG (todos los bags, todas las muestras walker) ===")
    for k, v in agg_cog.items():
        if not v:
            continue
        if "bias" in v:
            print(f"  {k:10s} MAE={v['mae']:.4f} MedAE={v['medae']:.4f} Bias={v['bias']:+.4f} "
                  f"RMSE={v['rmse']:.4f} (n={v['n']})")
        else:
            tag = " [NO COMPARABLE, ver nota]" if "not_comparable" in k else ""
            print(f"  {k:10s} MAE={v['mae']:.4f} MedAE={v['medae']:.4f} RMSE={v['rmse']:.4f} (n={v['n']}){tag}")


if __name__ == "__main__":
    main()
