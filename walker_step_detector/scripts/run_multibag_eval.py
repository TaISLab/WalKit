#!/usr/bin/env python3
"""Corre test_step_detector.launch.py contra TODOS los bags de una carpeta
(datasets/labeled_bags), grabando la salida de las tres variantes y
evaluandolas contra el ground truth de mocap -- automatiza el ciclo
lanzar -> grabar -> parar -> evaluar que se hacia a mano bag a bag.

Agregacion: NO se promedian las medias/medianas ya calculadas por bag (un
bag con 800 detecciones pesaria igual que uno con 80, y una media de
medianas no es la mediana del conjunto). En su lugar:
  - dist/label individuales se agrupan de TODOS los bags (validos para
    mean/median/p90/dominant_label, el orden no importa para estos).
  - swaps y duration_s se SUMAN tal cual los calculo eval_against_mocap por
    bag (no se recalculan sobre la concatenacion cruda de records: eso
    contaria un "swap" falso en cada frontera entre bags, donde el ultimo
    frame de un bag y el primero del siguiente no tienen relacion real).

Requiere: walker_step_detector, walker_description y laser_segmentation ya
compilados (colcon build) y disponibles via install/setup.bash.

Uso:
    python3 run_multibag_eval.py --bags-dir $DATASETS/labeled_bags \
        --out-dir /tmp/multibag_eval --rate 4.0 --out /tmp/multibag_result.json
"""
import argparse
import json
import os
import shutil
import signal
import statistics
import subprocess
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

from mocap_bag_io import bag_duration_seconds, find_labeled_bags, load_calib, repair_bag_metadata
from eval_against_mocap import evaluate_bag, print_table, summarize

ROS_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP = str(Path.home() / "workspace" / "walker_ws" / "install" / "setup.bash")
ROS_DOMAIN_ID_EVAL = "77"  # aislado del dominio por defecto (0), ver sh()

DETECTED_TOPICS = [
    "/detected_step_rf_left", "/detected_step_rf_right",
    "/detected_step_rf_left_ref", "/detected_step_rf_right_ref",
    "/detected_step_km_left", "/detected_step_km_right",
    "/detected_step_seg_left", "/detected_step_seg_right",
    "/detected_step_fused_left", "/detected_step_fused_right",
    "/detected_step_fused_fallback_active",
]
GT_TOPICS = ["/labeled_markers", "/rigid_bodies"]


def sh(cmd):
    """Lanza cmd bajo bash -c con el entorno ROS sourceado, en su propio
    process group (para poder pararlo entero, nodos hijos incluidos, con
    un solo killpg -- lanzar cada nodo suelto y perseguir PIDs a mano es
    justo lo que fallaba al hacer esto manualmente). ROS_DOMAIN_ID propio
    (ROS_DOMAIN_ID_EVAL): se detecto en vivo que otro proceso ajeno
    (walker_loads/replay_offline.launch.py, dominio por defecto 0) puede
    estar corriendo a la vez sus propias instancias de estos mismos nodos
    -- sin aislar el dominio, sus mensajes y los de esta evaluacion se
    mezclan en los mismos topics y contaminan las metricas sin ningun
    error visible."""
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


def run_one_bag(bag_dir, out_dir, rate, startup_wait):
    """Lanza el launcher + graba contra bag_dir. Devuelve la ruta del bag
    grabado, o None si algo fallo."""
    eval_bag = out_dir / bag_dir.name
    if eval_bag.exists():
        shutil.rmtree(eval_bag)

    # `ros2 bag play` (a diferencia de mocap_bag_io.load_bag) no pasa por
    # nuestra reparacion de metadata.yaml -- y la tarea de etiquetado sigue
    # corriendo en paralelo regenerando bags, asi que puede volver a
    # corromperse en cualquier momento (visto en vivo en esta sesion).
    repair_bag_metadata(bag_dir)

    launch = sh(
        "ros2 launch walker_step_detector test_step_detector.launch.py "
        f"bag_path:={bag_dir} rate:={rate}"
    )
    record = None
    try:
        time.sleep(startup_wait)
        if launch.poll() is not None:
            print(f"  [!] el launcher murio nada mas arrancar", file=sys.stderr)
            return None

        record = sh(f"ros2 bag record -o {eval_bag} " + " ".join(DETECTED_TOPICS + GT_TOPICS))
        time.sleep(1.5)  # que el recorder llegue a suscribirse antes de que empiece a llegar nada

        duration = bag_duration_seconds(bag_dir)
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


def aggregate_key(bag_summaries, pooled_records):
    """bag_summaries: lista de dicts summarize() (uno por bag) para un mismo
    (variante, lado). pooled_records: TODOS los records crudos de todos esos
    bags concatenados -- solo se usan dist/label de ellos (validos sin
    importar el orden); swaps y duration se suman de bag_summaries, no se
    recalculan sobre la concatenacion cruda (ver docstring del modulo)."""
    n = sum(s["n_detections"] for s in bag_summaries)
    n_matched = sum(s["n_matched"] for s in bag_summaries)
    dists = [r[1] for r in pooled_records if r[3]]
    labels = [r[2] for r in pooled_records if r[3]]
    swaps = sum(s["swaps"] for s in bag_summaries)
    pairs = sum(max(0, s["n_matched"] - 1) for s in bag_summaries)
    duration_s = sum(s["duration_s"] for s in bag_summaries)
    dominant_label, dominant_n = Counter(labels).most_common(1)[0] if labels else (None, 0)

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
        "swap_rate_per_100": 100 * swaps / pairs if pairs else float("nan"),
        "swaps_per_min": 60 * swaps / duration_s if duration_s > 0 else float("nan"),
        "n_bags": len(bag_summaries),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags-dir", required=True)
    ap.add_argument("--out-dir", required=True, help="donde se guardan los bags grabados de cada corrida")
    ap.add_argument("--calib", default=str(Path(__file__).resolve().parent.parent / "config" / "mocap_to_base_footprint.yaml"))
    ap.add_argument("--gate", type=float, default=0.3)
    ap.add_argument("--max-dt-ms", type=float, default=50.0)
    ap.add_argument("--rate", type=float, default=4.0, help="velocidad de reproduccion de cada bag")
    ap.add_argument("--startup-wait", type=float, default=4.0, help="segundos para que arranquen los nodos antes de grabar")
    ap.add_argument("--limit", type=int, default=None, help="probar solo con los N primeros bags")
    ap.add_argument("--keep-bags", action="store_true", help="no borrar los bags grabados de cada corrida al terminar")
    ap.add_argument("--out", default=None, help="JSON agregado de salida")
    args = ap.parse_args()

    bag_dirs = find_labeled_bags(args.bags_dir)
    if args.limit:
        bag_dirs = bag_dirs[:args.limit]
    if not bag_dirs:
        sys.exit(f"No hay bags bajo {args.bags_dir}")
    print(f"{len(bag_dirs)} bags a procesar (rate={args.rate})\n")

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    calib = load_calib(args.calib)
    max_dt_ns = int(args.max_dt_ms * 1e6)

    by_key_records = defaultdict(list)
    by_key_bag_summaries = defaultdict(list)
    per_bag_summary = {}
    failed_bags = []

    t0 = time.time()
    for i, bag_dir in enumerate(bag_dirs, 1):
        print(f"[{i}/{len(bag_dirs)}] {bag_dir.name} ...", flush=True)
        eval_bag = run_one_bag(bag_dir, out_dir, args.rate, args.startup_wait)
        if eval_bag is None:
            failed_bags.append(bag_dir.name)
            continue

        try:
            per_stream = evaluate_bag(str(eval_bag), calib, args.gate, max_dt_ns, quiet=True)
        except Exception as e:
            print(f"  [!] evaluate_bag fallo: {e}", file=sys.stderr)
            per_stream = {}
        if not per_stream:
            print(f"  [!] sin datos evaluables", file=sys.stderr)
            failed_bags.append(bag_dir.name)
            if not args.keep_bags:
                shutil.rmtree(eval_bag, ignore_errors=True)
            continue

        bag_summary = {}
        for key, records in per_stream.items():
            s = summarize(records)
            by_key_records[key].extend(records)
            by_key_bag_summaries[key].append(s)
            bag_summary[key] = s
        per_bag_summary[bag_dir.name] = bag_summary

        n_det_total = sum(len(r) for r in per_stream.values())
        print(f"  ok: {len(per_stream)} streams, {n_det_total} detecciones")

        if not args.keep_bags:
            shutil.rmtree(eval_bag, ignore_errors=True)

    elapsed = time.time() - t0
    print(f"\nTerminado en {elapsed/60:.1f} min. {len(failed_bags)}/{len(bag_dirs)} bags fallidos: {failed_bags}\n")

    if not by_key_records:
        sys.exit("Ningun bag produjo datos evaluables")

    aggregate = {
        key: aggregate_key(by_key_bag_summaries[key], records)
        for key, records in by_key_records.items()
    }
    print("=== Agregado sobre TODOS los bags (pooled, no promedio de medias por bag) ===")
    print_table(aggregate)

    if args.out:
        out = {
            "aggregate": {f"{v}/{s}": r for (v, s), r in aggregate.items()},
            "per_bag": {
                bag: {f"{v}/{s}": r for (v, s), r in summ.items()}
                for bag, summ in per_bag_summary.items()
            },
            "failed_bags": failed_bags,
            "_meta": {
                "bags_dir": args.bags_dir, "calib": args.calib, "gate_m": args.gate,
                "max_dt_ms": args.max_dt_ms, "rate": args.rate,
                "n_bags_ok": len(per_bag_summary), "n_bags_total": len(bag_dirs),
                "elapsed_min": elapsed / 60,
            },
        }
        with open(args.out, "w") as f:
            json.dump(out, f, indent=2, ensure_ascii=False)
        print(f"Guardado en {args.out}")


if __name__ == "__main__":
    main()
