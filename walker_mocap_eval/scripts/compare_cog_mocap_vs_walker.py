#!/usr/bin/env python3
"""Punto 3 (comparacion): compara, bag a bag, el centro de gravedad
estimado desde mocap (cog_estimation.py, modelo segmentario de De Leva +
sexo/peso del usuario) contra `/support_centroid` que publica de verdad
`walker_centroid_support` (centroide ponderado de las 4 cargas: 2 pies +
2 manetas), reproduciendo el pipeline completo igual que
compare_gait_mocap_vs_walker.py (punto 2) pero con `walker_centroid_support`
anadido a replay_offline.launch.py.

Por que comparar MEDIAS y no instante a instante: `/support_centroid` hereda
la calidad de deteccion de piernas de km_detect_steps/partial_loads (punto
2: tras los dos fixes, NoS todavia tiene ~25% de error/ruido) -- comparar
medias sobre toda la grabacion es mas robusto a ese ruido residual que
intentar emparejar muestra a muestra, y responde la pregunta relevante aqui
("¿el centroide medio de carga cae cerca de donde el mocap dice que esta el
CoM medio?"), no una traza dinamica.

El CoM de mocap (frame "map") se proyecta al frame del andador ("laser",
donde publica StepStamped y por tanto support_centroid) con la misma
calibracion ICP que usa compute_gait_from_mocap.py -- no es una posicion
absoluta, es relativa al andador en cada instante, asi que la comparacion
tiene sentido pese a que el andador se mueve.

Uso:
    python3 compare_cog_mocap_vs_walker.py \\
        --bags-dir /home/mfcarmona/workspace/walker_ws/src/soma13kp/datasets/labeled_bags \\
        --out-dir /tmp/cog_compare_runs --out /tmp/cog_compare_result.json
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

from mocap_eval import bag_io, cog_estimation  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from compute_cog_from_mocap import read_test_config_row  # noqa: E402
from compare_gait_mocap_vs_walker import tag_window_offset_and_duration  # noqa: E402

ROS_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP = str(Path.home() / "workspace" / "walker_ws" / "install" / "setup.bash")
# Mismo aislamiento que compare_gait_mocap_vs_walker.py (ver ese script):
# un dominio DDS propio evita que otro proceso/sesion ajena contamine las
# metricas sin error visible.
ROS_DOMAIN_ID_EVAL = "93"
DEFAULT_TEST_CONFIG = str(Path.home() / "bagsFolder_unified" / "test_config.txt")
DEFAULT_CALIB = (
    Path(__file__).resolve().parent.parent.parent
    / "walker_step_detector" / "config" / "mocap_to_base_footprint.yaml"
)


def sh(cmd):
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


def mocap_centroid_mean(bag_dir, test_config_file, calib, max_dt_ns, use_tags=True, return_series=False):
    fields = read_test_config_row(test_config_file, bag_dir.name)
    user_weight_kg, user_gender = float(fields[4]), fields[6]

    data = bag_io.load_bag(bag_dir, topics=("/labeled_markers", "/rigid_bodies"), require_all=True)
    mk_msgs, rb_msgs = data["/labeled_markers"], data["/rigid_bodies"]
    if not mk_msgs or not rb_msgs:
        return None

    if use_tags:
        tag_window = bag_io.read_tag_window(bag_dir)
        if tag_window is not None:
            t0, t1 = tag_window
            mk_msgs = bag_io.clip_track(mk_msgs, t0, t1)
            rb_msgs = bag_io.clip_track(rb_msgs, t0, t1)
        else:
            print(f"  [!] {bag_dir.name}: sin 2 mensajes /tag, usando el bag completo", file=sys.stderr)
        if not mk_msgs or not rb_msgs:
            return None

    tracks = {k: bag_io.marker_track(mk_msgs, k) for k in cog_estimation.REQUIRED_KEYS}
    series = cog_estimation.com_series(tracks, user_gender, max_dt_ns, weight_kg=user_weight_kg)
    if len(series) < 2:
        return None

    projected = []  # (t_ns, lx, ly, cz) -- lx/ly en frame laser, cz sin proyectar (altura, invariante al frame XY)
    for t, (cx, cy, cz) in series:
        rb = bag_io.nearest(rb_msgs, t, max_dt_ns)
        if rb is None:
            continue
        rb_x, rb_y, rb_yaw = bag_io.rigid_body_pose_xy_yaw(rb[1])
        lx, ly = bag_io.project_map_to_laser(cx, cy, rb_x, rb_y, rb_yaw, calib)
        projected.append((t, lx, ly, cz))
    if len(projected) < 2:
        return None

    xs = [p[1] for p in projected]
    ys = [p[2] for p in projected]
    stats = {"x_mean": statistics.mean(xs), "y_mean": statistics.mean(ys),
             "x_std": statistics.stdev(xs), "y_std": statistics.stdev(ys), "n": len(xs)}
    return (stats, projected) if return_series else stats


def run_one_bag(bag_dir, test_config_file, out_dir, rate, startup_wait, use_tags=True):
    eval_bag = out_dir / bag_dir.name
    if eval_bag.exists():
        shutil.rmtree(eval_bag)

    start_offset_s = 0.0
    duration = bag_io.bag_duration_seconds(bag_dir)
    if use_tags:
        window = tag_window_offset_and_duration(bag_dir)
        if window is not None:
            start_offset_s, duration = window
        else:
            print(f"  [!] {bag_dir.name}: sin 2 mensajes /tag, usando el bag completo", file=sys.stderr)

    launch = sh(
        "ros2 launch walker_loads replay_offline.launch.py "
        f"bag_path:={bag_dir} rate:={rate} test_config_file:={test_config_file} "
        f"start_offset:={start_offset_s}"
    )
    record = None
    try:
        time.sleep(startup_wait)
        if launch.poll() is not None:
            print("  [!] el launcher murio nada mas arrancar", file=sys.stderr)
            return None

        record = sh(f"ros2 bag record -s sqlite3 -o {eval_bag} /support_centroid")
        time.sleep(1.5)

        wait_s = duration / rate + 3.0
        time.sleep(wait_s)
    finally:
        stop(record)
        stop(launch)
        time.sleep(1.0)

    if not eval_bag.exists() or not any(eval_bag.iterdir()):
        print("  [!] no se genero nada grabable", file=sys.stderr)
        return None
    return eval_bag


def walker_centroid_mean(recorded_bag, return_series=False):
    try:
        data = bag_io.load_bag(recorded_bag, topics=("/support_centroid",), require_all=False)
    except Exception as e:
        print(f"  [!] no se pudo leer {recorded_bag}: {e}", file=sys.stderr)
        return None
    msgs = data.get("/support_centroid", [])
    if len(msgs) < 2:
        return None
    xs = [m.point.x for _, m in msgs]
    ys = [m.point.y for _, m in msgs]
    stats = {"x_mean": statistics.mean(xs), "y_mean": statistics.mean(ys),
             "x_std": statistics.stdev(xs), "y_std": statistics.stdev(ys), "n": len(xs)}
    if return_series:
        series = [(t, m.point.x, m.point.y, m.point.z) for t, m in msgs]
        return stats, series
    return stats


def fmt(v, nd=3):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "-"
    return f"{v:.{nd}f}"


def print_table(results):
    cols = ["bag", "mocap_x", "mocap_y", "walker_x", "walker_y", "dist_err_m"]
    rows = []
    for bag, r in results.items():
        if r is None:
            rows.append([bag] + ["-"] * (len(cols) - 1))
            continue
        rows.append([bag, fmt(r["mocap"]["x_mean"]), fmt(r["mocap"]["y_mean"]),
                     fmt(r["walker"]["x_mean"]), fmt(r["walker"]["y_mean"]), fmt(r["dist_err_m"])])
    widths = [max(len(str(row[i])) for row in ([cols] + rows)) for i in range(len(cols))]
    line = "  ".join(str(c).ljust(w) for c, w in zip(cols, widths))
    print("\nCentroide medio, mocap (De Leva) vs andador (support_centroid), frame laser:")
    print(line)
    print("-" * len(line))
    for row in rows:
        print("  ".join(str(c).ljust(w) for c, w in zip(row, widths)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags-dir", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--test-config-file", default=DEFAULT_TEST_CONFIG)
    ap.add_argument("--calib", default=str(DEFAULT_CALIB))
    ap.add_argument("--max-dt-ms", type=float, default=50.0)
    ap.add_argument("--rate", type=float, default=4.0)
    ap.add_argument("--startup-wait", type=float, default=4.0)
    ap.add_argument("--limit", type=int, default=None)
    ap.add_argument("--keep-bags", action="store_true")
    ap.add_argument("--ignore-tags", action="store_true",
                     help="usar el bag completo (mocap Y replay real) en vez de recortar a la ventana entre los 2 /tag")
    ap.add_argument("--out", default=None)
    ap.add_argument("--csv-dir", default=None,
                     help="si se da, escribe <bag>_cog_mocap.csv y <bag>_cog_walker.csv "
                          "(serie temporal completa, t relativo al inicio de la ventana) en este directorio")
    args = ap.parse_args()
    csv_dir = Path(args.csv_dir) if args.csv_dir else None
    if csv_dir:
        csv_dir.mkdir(parents=True, exist_ok=True)

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

    results = {}
    for i, bag_dir in enumerate(bag_dirs):
        print(f"[{i + 1}/{len(bag_dirs)}] {bag_dir.name}")

        try:
            mocap_result = mocap_centroid_mean(bag_dir, args.test_config_file, calib, max_dt_ns,
                                                use_tags=not args.ignore_tags, return_series=bool(csv_dir))
        except (ValueError, RuntimeError) as e:
            print(f"  [!] mocap: {e}", file=sys.stderr)
            mocap_result = None
        if mocap_result is None:
            print("  [!] mocap insuficiente, se salta")
            results[bag_dir.name] = None
            continue
        mocap, mocap_series = mocap_result if csv_dir else (mocap_result, None)

        recorded = run_one_bag(bag_dir, args.test_config_file, out_dir, args.rate, args.startup_wait,
                               use_tags=not args.ignore_tags)
        walker_result = walker_centroid_mean(recorded, return_series=bool(csv_dir)) if recorded else None
        if not args.keep_bags and recorded is not None:
            shutil.rmtree(recorded, ignore_errors=True)

        if walker_result is None:
            print("  [!] sin support_centroid grabado, se salta")
            results[bag_dir.name] = None
            continue
        walker, walker_series = walker_result if csv_dir else (walker_result, None)

        if csv_dir:
            t0_ref = mocap_series[0][0]
            with open(csv_dir / f"{bag_dir.name}_cog_mocap.csv", "w") as f:
                f.write("t_s,x_m,y_m,z_m\n")
                for t, x, y, z in mocap_series:
                    f.write(f"{(t - t0_ref) / 1e9:.4f},{x:.4f},{y:.4f},{z:.4f}\n")
            with open(csv_dir / f"{bag_dir.name}_cog_walker.csv", "w") as f:
                f.write("t_s,x_m,y_m,z_m\n")
                for t, x, y, z in walker_series:
                    f.write(f"{(t - t0_ref) / 1e9:.4f},{x:.4f},{y:.4f},{z:.4f}\n")

        dist_err = math.hypot(walker["x_mean"] - mocap["x_mean"], walker["y_mean"] - mocap["y_mean"])
        results[bag_dir.name] = {"mocap": mocap, "walker": walker, "dist_err_m": dist_err}

    print_table(results)

    valid = [r for r in results.values() if r is not None]
    print(f"\nAgregado sobre {len(valid)}/{len(results)} bags:")
    if valid:
        dists = [r["dist_err_m"] for r in valid]
        print(f"  dist_err_m  mean={statistics.mean(dists):.3f}  median={statistics.median(dists):.3f}  "
              f"max={max(dists):.3f}  n={len(dists)}")

    if args.out:
        with open(args.out, "w") as f:
            json.dump(results, f, indent=2, ensure_ascii=False, default=str)
        print(f"\nGuardado en {args.out}")


if __name__ == "__main__":
    main()
