#!/usr/bin/env python3
"""Compara una odometria grabada (por defecto /odom, la salida del EKF de
walker_laser_imu_odom.launch.py -- laser CSM/PL-ICP + IMU WitMotion) contra
el ground truth de mocap (rigid_bodies[0] = el andador), usando la misma
calibracion rigid_body->base_footprint que walker_step_detector/scripts/
eval_against_mocap.py (mocap_odometry/config/mocap_to_base_footprint.yaml,
fuente unica: se referencia desde aqui, no se duplica).

Que hay que grabar: al menos el topic de odometria a evaluar y /rigid_bodies,
mientras corre walker_laser_imu_odom_test.launch.py contra un bag definitivo
(bagsFolder/definitive, reproducido con walker_replay), p.ej.:

    ros2 launch walker_bringup walker_laser_imu_odom_test.launch.py \\
        bag_name:=MF_test01 rate:=1.0 &
    ros2 bag record -o /tmp/odom_eval_MF_test01 /odom /rigid_bodies

Metrica -- ATE (Absolute Trajectory Error), como en el benchmark TUM RGB-D:
  1. /rigid_bodies[0].pose (frame "map") se proyecta a base_footprint (mismo
     calib que eval_against_mocap.py) -> trayectoria de referencia en "map".
  2. La odometria evaluada vive en su propio frame "odom" (origen y
     orientacion arbitrarios: el EKF arranca en (0,0,yaw=0) donde este el
     andador en ese instante) -- para poder comparar posiciones hace falta
     alinear esa trayectoria con la de referencia. Se calcula la
     transformacion rigida 2D (rotacion + traslacion, sin escala) que mejor
     alinea ambas trayectorias en el sentido de minimos cuadrados (Umeyama/
     Kabsch en 2D, sobre TODOS los pares emparejados por tiempo) -- no un
     único punto de anclaje al principio, que seria mucho mas sensible al
     ruido de una sola muestra.
  3. Con la trayectoria ya alineada: error de posicion (distancia euclidea)
     y de rumbo (yaw) punto a punto, resumidos en mean/median/rmse/p90.
  4. "drift_pct": ATE rmse como porcentaje de la longitud total recorrida
     (segun el ground truth) -- una ATE de 10 cm en un trayecto de 2 m es
     mucho peor que en uno de 40 m; salvo aviso, "buena" odometria de
     interior suele rondar 1-5% de deriva.

Correccion temporal del mocap. En algunos bags el eje de tiempo del mocap esta desplazado y escalado respecto al
de los sensores del robot (hasta 5 s y 7 %); sin corregirlo el ATE y el rumbo salen mal por un problema de
referencia, no de la odometria. La correccion por bag esta en mocap_odometry/config/time_warp.yaml (la genera
odom_tuning/fit_time_warp.py) y este script la aplica a los sellos del mocap. La clave del yaml es el nombre del bag
ORIGINAL (CA_test05): se deduce del nombre del directorio del eval-bag (odom_eval_CA_test05 -> CA_test05) o se da con
--bag-name. Como el yaml esta expresado en sellos de cabecera, el emparejado odom<->mocap se hace por defecto con
sellos de cabecera (--time-basis header; el odom lleva el tiempo de simulacion = el del bag original); con
--time-basis record se usa el tiempo de grabacion del eval-bag, como antes, y la correccion NO se aplica.
--time-warp none la desactiva.

Uso:
    python3 walker_mocap_eval/scripts/eval_odom_against_mocap.py \\
        --eval-bag /tmp/odom_eval_MF_test01
"""
from mocap_odometry.core import default_calibration_file  # noqa: E402
import argparse
import json
import math
import statistics
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mocap_odometry import core, metrics  # noqa: E402
from mocap_eval.bag_io import bag_topics, load_bag, load_calib, nearest, rigid_body_pose_xy_yaw  # noqa: E402

DEFAULT_CALIB = default_calibration_file()

DEFAULT_ODOM_TOPICS = ("/odom", "/ekf_odom", "/scan_odom")

DEFAULT_TIME_WARP = core.default_time_warp_file()


def load_time_warp(path=DEFAULT_TIME_WARP):
    """{bag: {offset_s, slope, t_ref, ...}} de time_warp.yaml; {} si path es None o no existe.
    La correccion vive en mocap_odometry (core.warp_ns / find_warp); el fichero sigue siendo el de aqui."""
    if not path or not Path(path).exists():
        return {}
    return core.load_time_warp(path)


warp_ns = core.warp_ns
find_warp = core.find_warp


def stamp_ns(header):
    return header.stamp.sec * 10**9 + header.stamp.nanosec


def discover_odom_topic(bag_dir, requested=None):
    topics = bag_topics(bag_dir)
    if requested:
        if requested not in topics:
            sys.exit(f"{bag_dir}: no tiene el topic {requested} (topics: {sorted(topics)})")
        return requested
    for t in DEFAULT_ODOM_TOPICS:
        if t in topics:
            return t
    sys.exit(f"{bag_dir}: ninguno de {DEFAULT_ODOM_TOPICS} presente, usa --odom-topic (topics: {sorted(topics)})")


def yaw_from_quat(q):
    return core.yaw_from_quat(q.x, q.y, q.z, q.w)


wrap_angle = core.wrap_angle


def mocap_to_base_footprint(rb_x, rb_y, rb_yaw, calib):
    """rigid_body.pose (frame map) -> pose de base_footprint (frame map): offset fijo de montaje (dx, dy, dyaw).
    Implementacion en mocap_odometry.core.apply_calibration (la misma que usa el nodo)."""
    return core.apply_calibration(rb_x, rb_y, rb_yaw, calib)


def load_gt_trajectory(rbs, calib, warp=None, use_header=False):
    """Referencia en base_footprint. use_header: sellos de cabecera en vez del tiempo de grabacion; warp:
    correccion de time_warp.yaml (solo tiene sentido con sellos de cabecera)."""
    out = []
    for t, msg in rbs:
        if use_header:
            t = stamp_ns(msg.header)
        if warp:
            t = warp_ns(t, warp)
        out.append((t, *mocap_to_base_footprint(*rigid_body_pose_xy_yaw(msg), calib)))
    return out


def load_odom_trajectory(msgs, use_header=False):
    out = []
    for t, msg in msgs:
        p = msg.pose.pose.position
        out.append((stamp_ns(msg.header) if use_header else t, p.x, p.y, yaw_from_quat(msg.pose.pose.orientation)))
    return out


# Alineacion 2D (Umeyama), emparejado por tiempo y longitud de trayectoria: en mocap_odometry.metrics
pair_by_time = metrics.pair_by_time
align_2d = metrics.align_2d
apply_2d = metrics.apply_2d
path_length = metrics.path_length


def q(vals, p):
    if len(vals) < 2:
        return float("nan")
    return statistics.quantiles(vals, n=100)[p - 1]


def evaluate(pairs):
    if len(pairs) < 5:
        sys.exit(f"Muy pocos pares emparejados ({len(pairs)}), revisa --max-dt-ms y los topics del bag")

    src_xy = [(p[1], p[2]) for p in pairs]   # odom (x, y)
    dst_xy = [(p[4], p[5]) for p in pairs]   # gt   (x, y)
    theta, tx, ty = align_2d(src_xy, dst_xy)

    pos_err, yaw_err_deg = [], []
    aligned_xy = []
    for t, ox, oy, oyaw, gx, gy, gyaw in pairs:
        ax, ay = apply_2d(theta, tx, ty, ox, oy)
        aligned_xy.append((ax, ay))
        pos_err.append(math.hypot(ax - gx, ay - gy))
        ayaw = wrap_angle(oyaw + theta)
        yaw_err_deg.append(abs(math.degrees(wrap_angle(ayaw - gyaw))))

    gt_path_m = path_length([(p[4], p[5]) for p in pairs])
    rmse = math.sqrt(statistics.mean(e * e for e in pos_err))

    return {
        "n_pairs": len(pairs),
        "duration_s": (pairs[-1][0] - pairs[0][0]) / 1e9,
        "gt_path_length_m": gt_path_m,
        "alignment_theta_deg": math.degrees(theta),
        "alignment_tx_m": tx,
        "alignment_ty_m": ty,
        "ate_mean_m": statistics.mean(pos_err),
        "ate_median_m": statistics.median(pos_err),
        "ate_rmse_m": rmse,
        "ate_p90_m": q(pos_err, 90),
        "ate_max_m": max(pos_err),
        "drift_pct": 100 * rmse / gt_path_m if gt_path_m > 1e-6 else float("nan"),
        "yaw_err_mean_deg": statistics.mean(yaw_err_deg),
        "yaw_err_rmse_deg": math.sqrt(statistics.mean(e * e for e in yaw_err_deg)),
        "yaw_err_p90_deg": q(yaw_err_deg, 90),
    }


def print_report(name, m):
    print(f"\n=== {name} ===")
    print(f"  pares emparejados: {m['n_pairs']}   duracion: {m['duration_s']:.1f} s"
          f"   longitud recorrida (gt): {m['gt_path_length_m']:.2f} m")
    print(f"  alineacion odom->map: theta={m['alignment_theta_deg']:+.2f} deg  "
          f"t=({m['alignment_tx_m']:+.3f}, {m['alignment_ty_m']:+.3f}) m")
    print(f"  ATE posicion (m):  mean={m['ate_mean_m']:.3f}  median={m['ate_median_m']:.3f}  "
          f"rmse={m['ate_rmse_m']:.3f}  p90={m['ate_p90_m']:.3f}  max={m['ate_max_m']:.3f}")
    print(f"  deriva: {m['drift_pct']:.2f} % de la trayectoria recorrida")
    print(f"  error de rumbo (deg): mean={m['yaw_err_mean_deg']:.2f}  "
          f"rmse={m['yaw_err_rmse_deg']:.2f}  p90={m['yaw_err_p90_deg']:.2f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--eval-bag", required=True, help="bag grabado con ros2 bag record durante walker_laser_imu_odom_test.launch.py")
    ap.add_argument("--odom-topic", default=None, help=f"por defecto autodetecta entre {DEFAULT_ODOM_TOPICS}")
    ap.add_argument("--calib", default=str(DEFAULT_CALIB),
                    help="mocap_to_base_footprint.yaml (por defecto, el de walker_step_detector)")
    ap.add_argument("--max-dt-ms", type=float, default=50.0, help="tolerancia de emparejado odom<->rigid_bodies")
    ap.add_argument("--out", default=None, help="JSON de salida con las metricas")
    ap.add_argument("--time-warp", default=str(DEFAULT_TIME_WARP),
                    help="time_warp.yaml con la correccion temporal del mocap por bag; 'none' la desactiva")
    ap.add_argument("--bag-name", default=None,
                    help="nombre del bag ORIGINAL (clave de time_warp.yaml); por defecto se deduce del nombre del eval-bag")
    ap.add_argument("--time-basis", choices=("header", "record"), default="header",
                    help="sellos con los que se empareja odom y mocap: cabecera (defecto; admite la correccion) "
                         "o tiempo de grabacion del eval-bag (sin correccion)")
    args = ap.parse_args()

    calib = load_calib(args.calib)
    print(f"Calibracion mocap->base_footprint: dx={calib[0]:+.3f} dy={calib[1]:+.3f} "
          f"dyaw={math.degrees(calib[2]):+.2f} deg  ({args.calib})")

    odom_topic = discover_odom_topic(args.eval_bag, args.odom_topic)
    print(f"Topic de odometria evaluado: {odom_topic}")

    data = load_bag(args.eval_bag, topics=(odom_topic, "/rigid_bodies"), require_all=True)
    use_header = args.time_basis == "header"
    odom_msgs = data[odom_topic]
    if use_header and odom_msgs and all(stamp_ns(m.header) == 0 for _, m in odom_msgs):
        print("AVISO: la odometria no lleva sellos de cabecera; se empareja por tiempo de grabacion y no se corrige el mocap")
        use_header = False
    warp, warp_key = None, None
    if use_header and args.time_warp.lower() != "none":
        warps = load_time_warp(args.time_warp)
        warp_key, warp = find_warp(args.bag_name or Path(args.eval_bag).resolve().name, warps)
        if warp:
            print(f"Correccion temporal del mocap ({warp_key}, {args.time_warp}): offset={warp['offset_s']:+.2f} s "
                  f"deriva={warp['slope']:+.4f} s/s")
        else:
            print(f"Correccion temporal del mocap: ninguna (bag no figura en {args.time_warp}; "
                  f"--bag-name para indicarlo)")
    elif not use_header:
        print("Correccion temporal del mocap: no aplicable con --time-basis record")
    gt_traj = load_gt_trajectory(data["/rigid_bodies"], calib, warp, use_header)
    odom_traj = load_odom_trajectory(odom_msgs, use_header)
    if not gt_traj or not odom_traj:
        sys.exit(f"{args.eval_bag}: sin datos en /rigid_bodies o {odom_topic}")

    max_dt_ns = int(args.max_dt_ms * 1e6)
    pairs = pair_by_time(odom_traj, gt_traj, max_dt_ns)
    metrics = evaluate(pairs)
    print_report(odom_topic, metrics)

    if args.out:
        out = {**metrics, "_meta": {"eval_bag": args.eval_bag, "odom_topic": odom_topic,
                                     "calib": args.calib, "max_dt_ms": args.max_dt_ms,
                                     "time_basis": "header" if use_header else "record",
                                     "time_warp": {"bag": warp_key, **{k: warp[k] for k in ("offset_s", "slope", "t_ref")}}
                                     if warp else None}}
        with open(args.out, "w") as f:
            json.dump(out, f, indent=2, ensure_ascii=False)
        print(f"\nGuardado en {args.out}")


if __name__ == "__main__":
    main()
