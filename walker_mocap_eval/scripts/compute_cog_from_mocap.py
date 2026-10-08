#!/usr/bin/env python3
"""Calcula, desde el mocap etiquetado + los datos del usuario (sexo, peso;
altura solo como contraste, ver cog_estimation.py), el centro de gravedad
(CoM) de cuerpo completo por bag -- con el objetivo de compararlo mas
adelante contra lo que publica el andador (`support_centroid` de
walker_centroid_support, `StabilityStamped` de walker_stability).

El sexo/peso/altura por bag salen de bagsFolder_unified/test_config.txt
(mismo fichero que usa walker_loads/launch/replay_offline.launch.py),
columnas [test]:[handle_height]:[age]:[height]:[weight]:[user_id]:[gender]:[notes].

Uso:
    python3 compute_cog_from_mocap.py --bag $DATASETS/labeled_bags/CA_test01
    python3 compute_cog_from_mocap.py --bags-dir $DATASETS/labeled_bags \\
        --out /tmp/mocap_cog_results.json
"""
import argparse
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mocap_eval import bag_io, cog_estimation  # noqa: E402

DEFAULT_TEST_CONFIG = str(Path.home() / "bagsFolder_unified" / "test_config.txt")


def read_test_config_row(test_config_file, bag_name):
    """Mismo parseo que replay_offline.launch.py::read_test_config_row."""
    with open(test_config_file) as f:
        for line in f:
            fields = [t.strip() for t in line.strip().split(':')]
            if fields and fields[0] == bag_name:
                return fields
    raise RuntimeError(f"No hay fila para '{bag_name}' en {test_config_file}")


def _mean_std(values):
    vals = [v for v in values if not (isinstance(v, float) and math.isnan(v))]
    if not vals:
        return float("nan"), float("nan")
    if len(vals) == 1:
        return vals[0], float("nan")
    return statistics.mean(vals), statistics.stdev(vals)


def compute_bag(bag_dir, test_config_file, max_dt_ns, use_tags=True):
    fields = read_test_config_row(test_config_file, bag_dir.name)
    user_height_cm, user_weight_kg, user_gender = float(fields[3]), float(fields[4]), fields[6]

    data = bag_io.load_bag(bag_dir, topics=("/labeled_markers",), require_all=True)
    mk_msgs = data["/labeled_markers"]
    if not mk_msgs:
        return None

    if use_tags:
        tag_window = bag_io.read_tag_window(bag_dir)
        if tag_window is not None:
            t0, t1 = tag_window
            mk_msgs = bag_io.clip_track(mk_msgs, t0, t1)
        else:
            print(f"  [!] {bag_dir.name}: sin 2 mensajes /tag, usando el bag completo", file=sys.stderr)
        if not mk_msgs:
            return None

    tracks = {k: bag_io.marker_track(mk_msgs, k) for k in cog_estimation.REQUIRED_KEYS}
    series = cog_estimation.com_series(tracks, user_gender, max_dt_ns, weight_kg=user_weight_kg)
    if len(series) < 2:
        return None

    xs = [c[0] for _, c in series]
    ys = [c[1] for _, c in series]
    zs = [c[2] for _, c in series]
    x_mean, x_std = _mean_std(xs)
    y_mean, y_std = _mean_std(ys)
    z_mean, z_std = _mean_std(zs)

    # Contraste (no se usa para calcular la posicion, ver cog_estimation.py):
    # altura del CoM como fraccion de la estatura declarada -- valor tipico
    # en literatura para un adulto de pie es ~0.55.
    com_height_frac_of_stature = z_mean / (user_height_cm / 100.0) if user_height_cm else float("nan")

    return {
        "com_x_mean": x_mean, "com_x_std": x_std,
        "com_y_mean": y_mean, "com_y_std": y_std,
        "com_z_mean": z_mean, "com_z_std": z_std,
        "com_height_frac_of_stature": com_height_frac_of_stature,
        "user_gender": user_gender, "user_weight_kg": user_weight_kg, "user_height_cm": user_height_cm,
        "n_frames": len(series),
    }


def fmt(v, nd=3):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "-"
    return f"{v:.{nd}f}" if isinstance(v, float) else str(v)


def print_table(results):
    cols = ["bag", "gender", "weight", "height", "com_x", "com_y", "com_z", "z/estatura"]
    rows = []
    for bag_name, r in results.items():
        if r is None:
            rows.append([bag_name] + ["-"] * (len(cols) - 1))
            continue
        rows.append([
            bag_name, r["user_gender"], fmt(r["user_weight_kg"], 0), fmt(r["user_height_cm"], 0),
            fmt(r["com_x_mean"]), fmt(r["com_y_mean"]), fmt(r["com_z_mean"]),
            fmt(r["com_height_frac_of_stature"]),
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
    ap.add_argument("--test-config-file", default=DEFAULT_TEST_CONFIG)
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
                bag_dir, args.test_config_file, max_dt_ns, use_tags=not args.ignore_tags)
        except (ValueError, RuntimeError) as e:
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
