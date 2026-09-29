#!/usr/bin/env python3
"""Punto 2 del plan de validacion: compara, bag a bag, los parametros de
gait_monitor_speed calculados desde mocap (compute_gait_from_mocap.py)
contra los que PUBLICA de verdad el nodo real, reproduciendo el pipeline
completo contra cada bag etiquetado (walker_loads/launch/
replay_offline.launch.py: box_filter -> km_detect_steps -> partial_loads ->
mocap_odom_bridge -> gait_monitor_speed).

Por que reproducir el pipeline real en vez de reimplementarlo (a diferencia
de gait_metrics.py, que SI reimplementa el algoritmo en Python puro): SpT/
SdT/SpL/SdL dependen de la deteccion de piernas por laser (km_detect_steps,
un tracker con filtro de Kalman) y de la fusion de cargas (partial_loads),
que no tiene sentido reescribir en Python solo para esta comparacion --
igual que hace walker_step_detector/scripts/run_multibag_eval.py para
validar el detector de pasos, aqui se lanza el nodo de verdad contra cada
bag y se graba lo que publica.

Gait_monitor_speed publica GaitStamped/WalkStats de forma PERIODICA y
ACUMULADA (media desde el arranque del nodo, ver gait_monitor_speed.py):
por eso solo hace falta el ULTIMO mensaje de cada topic grabado -- no
importa si el grabador externo (`ros2 bag record`) se suscribe unos
segundos tarde, el nodo ya llevaba toda la cuenta desde su propio arranque.

Requiere: workspace compilado (colcon build), bagsFolder_unified/
test_config.txt (columna user_desc/handle_height por bag, ver
replay_offline.launch.py) y walker_step_detector/config/
mocap_to_base_footprint.yaml (calibracion mocap->laser, ya existente).

Uso:
    python3 compare_gait_mocap_vs_walker.py \\
        --bags-dir /home/mfcarmona/workspace/walker_ws/src/soma13kp/datasets/labeled_bags \\
        --out-dir /tmp/gait_compare_runs --out /tmp/gait_compare_result.json

    # probar con pocos bags primero
    python3 compare_gait_mocap_vs_walker.py --bags-dir ... --out-dir ... --limit 3
"""
import argparse
import json
import math
import os
import shutil
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mocap_eval import bag_io  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from compute_gait_from_mocap import compute_bag as compute_mocap_bag  # noqa: E402

ROS_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP = str(Path.home() / "workspace" / "walker_ws" / "install" / "setup.bash")
# aislado del dominio por defecto (0): walker_step_detector/scripts/
# run_multibag_eval.py detecto en vivo que otro proceso ajeno en el mismo
# dominio contamina las metricas sin ningun error visible (ver el comentario
# de su propio ROS_DOMAIN_ID_EVAL) -- mismo riesgo aqui si esta corriendo a
# la vez que otra sesion/instancia de este mismo pipeline.
ROS_DOMAIN_ID_EVAL = "92"
DEFAULT_TEST_CONFIG = str(Path.home() / "bagsFolder_unified" / "test_config.txt")
DEFAULT_CALIB = (
    Path(__file__).resolve().parent.parent.parent
    / "walker_step_detector" / "config" / "mocap_to_base_footprint.yaml"
)

WALKER_TOPICS = ["/left_gait_stats", "/right_gait_stats", "/global_gait_stats"]

# Campos a comparar: (nombre, campo del lado mocap [gait_metrics.summarize()], como leerlo del lado walker)
COMPARE_FIELDS = [
    "left_spt", "left_sdt", "left_spl", "left_sdl",
    "right_spt", "right_sdt", "right_spl", "right_sdl",
    "nos", "d_m", "cad", "wv",
]


def sh(cmd):
    """Lanza cmd bajo bash -c con el entorno ROS sourceado, en su propio
    process group -- igual que run_multibag_eval.py, para poder pararlo
    entero (nodos hijos incluidos) con un solo killpg."""
    full = f"source {ROS_SETUP}; source {WS_SETUP}; export ROS_DOMAIN_ID={ROS_DOMAIN_ID_EVAL}; exec {cmd}"
    return subprocess.Popen(
        ["bash", "-c", full],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )


def stop(proc, timeout=8):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass


def run_one_bag(bag_dir, test_config_file, out_dir, rate, startup_wait):
    """Lanza replay_offline.launch.py + graba /left_gait_stats,
    /right_gait_stats, /global_gait_stats. Devuelve la ruta del bag
    grabado, o None si algo fallo (p.ej. sin fila en test_config.txt)."""
    eval_bag = out_dir / bag_dir.name
    if eval_bag.exists():
        shutil.rmtree(eval_bag)

    launch = sh(
        "ros2 launch walker_loads replay_offline.launch.py "
        f"bag_path:={bag_dir} rate:={rate} test_config_file:={test_config_file}"
    )
    record = None
    try:
        time.sleep(startup_wait)
        if launch.poll() is not None:
            print(f"  [!] el launcher murio nada mas arrancar (sin fila en test_config.txt?)", file=sys.stderr)
            return None

        record = sh(f"ros2 bag record -s sqlite3 -o {eval_bag} " + " ".join(WALKER_TOPICS))
        time.sleep(1.5)  # que el recorder llegue a suscribirse

        duration = bag_io.bag_duration_seconds(bag_dir)
        wait_s = duration / rate + 3.0
        time.sleep(wait_s)
    finally:
        stop(record)
        stop(launch)
        time.sleep(1.0)

    if not eval_bag.exists() or not any(eval_bag.iterdir()):
        print(f"  [!] no se genero nada grabable", file=sys.stderr)
        return None
    return eval_bag


def last_msg(msgs):
    return msgs[-1][1] if msgs else None


def extract_walker_stats(recorded_bag):
    """Ultimo mensaje de cada topic -> dict con las mismas claves que
    gait_metrics.summarize(), para poder restar campo a campo."""
    try:
        data = bag_io.load_bag(recorded_bag, topics=tuple(WALKER_TOPICS), require_all=False)
    except Exception as e:
        print(f"  [!] no se pudo leer {recorded_bag}: {e}", file=sys.stderr)
        return None

    left = last_msg(data.get("/left_gait_stats", []))
    right = last_msg(data.get("/right_gait_stats", []))
    glob = last_msg(data.get("/global_gait_stats", []))
    if left is None or right is None or glob is None:
        return None

    return {
        "left_spt": left.spt, "left_sdt": left.sdt, "left_spl": left.spl, "left_sdl": left.sdl,
        "right_spt": right.spt, "right_sdt": right.sdt, "right_spl": right.spl, "right_sdl": right.sdl,
        "nos": glob.nos,
        "tr_s": glob.tr.sec + glob.tr.nanosec / 1e9,
        "d_m": glob.d, "cad": glob.cad, "wv": glob.wv,
    }


def diff_record(mocap_val, walker_val):
    if mocap_val is None or walker_val is None:
        return {"mocap": None, "walker": None, "abs_err": float("nan"), "rel_err": float("nan")}
    if isinstance(mocap_val, float) and math.isnan(mocap_val):
        return {"mocap": mocap_val, "walker": walker_val, "abs_err": float("nan"), "rel_err": float("nan")}
    abs_err = abs(walker_val - mocap_val)
    rel_err = abs_err / abs(mocap_val) if mocap_val else float("nan")
    return {"mocap": mocap_val, "walker": walker_val, "abs_err": abs_err, "rel_err": rel_err}


def compare_bag(mocap_stats, walker_stats):
    return {field: diff_record(mocap_stats.get(field), walker_stats.get(field)) for field in COMPARE_FIELDS}


def fmt(v, nd=3):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "-"
    return f"{v:.{nd}f}" if isinstance(v, float) else str(v)


def print_bag_table(per_bag_comparisons):
    cols = ["bag"] + COMPARE_FIELDS
    rows = []
    for bag_name, cmp_ in per_bag_comparisons.items():
        if cmp_ is None:
            rows.append([bag_name] + ["-"] * len(COMPARE_FIELDS))
            continue
        rows.append([bag_name] + [fmt(cmp_[f]["abs_err"]) for f in COMPARE_FIELDS])
    widths = [max(len(str(r[i])) for r in ([cols] + rows)) for i in range(len(cols))]
    line = "  ".join(str(c).ljust(w) for c, w in zip(cols, widths))
    print("\nError absoluto |walker - mocap| por bag y campo:")
    print(line)
    print("-" * len(line))
    for r in rows:
        print("  ".join(str(c).ljust(w) for c, w in zip(r, widths)))


def print_aggregate(per_bag_comparisons):
    valid = [c for c in per_bag_comparisons.values() if c is not None]
    print(f"\nAgregado sobre {len(valid)}/{len(per_bag_comparisons)} bags:")
    for field in COMPARE_FIELDS:
        abs_errs = [c[field]["abs_err"] for c in valid if not math.isnan(c[field]["abs_err"])]
        rel_errs = [c[field]["rel_err"] for c in valid if not math.isnan(c[field]["rel_err"])]
        if not abs_errs:
            print(f"  {field:10s} sin datos")
            continue
        mae = statistics.mean(abs_errs)
        mdae = statistics.median(abs_errs)
        mre = statistics.mean(rel_errs) * 100 if rel_errs else float("nan")
        print(f"  {field:10s} MAE={mae:.3f}  medianAE={mdae:.3f}  MAPE={mre:.1f}%  n={len(abs_errs)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags-dir", required=True)
    ap.add_argument("--out-dir", required=True, help="donde se guardan los bags grabados de cada corrida")
    ap.add_argument("--test-config-file", default=DEFAULT_TEST_CONFIG)
    ap.add_argument("--calib", default=str(DEFAULT_CALIB))
    ap.add_argument("--speed-threshold", type=float, default=0.25)
    ap.add_argument("--min-gap-s", type=float, default=0.3)
    ap.add_argument("--max-dt-ms", type=float, default=50.0)
    ap.add_argument("--rate", type=float, default=4.0, help="velocidad de reproduccion de cada bag")
    ap.add_argument("--startup-wait", type=float, default=4.0, help="segundos para que arranquen los nodos antes de grabar")
    ap.add_argument("--limit", type=int, default=None, help="probar solo con los N primeros bags")
    ap.add_argument("--keep-bags", action="store_true", help="no borrar los bags grabados de cada corrida al terminar")
    ap.add_argument("--out", default=None, help="JSON de salida con la comparacion completa")
    args = ap.parse_args()

    bag_dirs = bag_io.find_labeled_bags(args.bags_dir)
    if args.limit:
        bag_dirs = bag_dirs[:args.limit]
    if not bag_dirs:
        sys.exit(f"No hay bags bajo {args.bags_dir}")
    print(f"{len(bag_dirs)} bags a procesar (rate={args.rate})\n")

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    calib = bag_io.load_calib(args.calib)
    max_dt_ns = int(args.max_dt_ms * 1e6)

    per_bag_comparisons = {}
    per_bag_raw = {}
    for i, bag_dir in enumerate(bag_dirs):
        print(f"[{i + 1}/{len(bag_dirs)}] {bag_dir.name}")

        try:
            mocap_stats = compute_mocap_bag(bag_dir, calib, args.speed_threshold, args.min_gap_s, max_dt_ns)
        except Exception as e:
            print(f"  [!] mocap: {e}", file=sys.stderr)
            mocap_stats = None
        if mocap_stats is None:
            print("  [!] sin suficientes pasos en el mocap, se salta")
            per_bag_comparisons[bag_dir.name] = None
            continue

        recorded = run_one_bag(bag_dir, args.test_config_file, out_dir, args.rate, args.startup_wait)
        walker_stats = extract_walker_stats(recorded) if recorded else None
        if not args.keep_bags and recorded is not None:
            shutil.rmtree(recorded, ignore_errors=True)

        if walker_stats is None:
            print("  [!] sin GaitStamped/WalkStats grabados del pipeline real, se salta")
            per_bag_comparisons[bag_dir.name] = None
            continue

        per_bag_raw[bag_dir.name] = {"mocap": mocap_stats, "walker": walker_stats}
        per_bag_comparisons[bag_dir.name] = compare_bag(mocap_stats, walker_stats)

    print_bag_table(per_bag_comparisons)
    print_aggregate(per_bag_comparisons)

    if args.out:
        out = {
            "per_bag": per_bag_comparisons,
            "per_bag_raw": per_bag_raw,
            "_meta": {
                "bags_dir": args.bags_dir, "rate": args.rate, "calib": args.calib,
                "speed_threshold": args.speed_threshold, "min_gap_s": args.min_gap_s,
            },
        }
        with open(args.out, "w") as f:
            json.dump(out, f, indent=2, ensure_ascii=False, default=str)
        print(f"\nGuardado en {args.out}")


if __name__ == "__main__":
    main()
