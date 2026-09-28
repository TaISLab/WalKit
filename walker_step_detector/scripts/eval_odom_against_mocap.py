#!/usr/bin/env python3
"""Compara una odometria grabada (por defecto /odom, la salida del EKF de
walker_laser_imu_odom.launch.py -- laser CSM/PL-ICP + IMU WitMotion) contra
el ground truth de mocap (rigid_bodies[0] = el andador), usando la misma
calibracion rigid_body->base_footprint que walker_step_detector/scripts/
eval_against_mocap.py (config/mocap_to_base_footprint.yaml).

Que hay que grabar: al menos el topic de odometria a evaluar y /rigid_bodies,
mientras corre test_laser_imu_odom.launch.py contra un bag de
bagsFolder_unified, p.ej.:

    ros2 launch walker_bringup test_laser_imu_odom.launch.py \\
        bag_path:=/home/mfcarmona/bagsFolder_unified/MF_test01 rate:=1.0 &
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

Uso:
    python3 eval_odom_against_mocap.py --eval-bag /tmp/odom_eval_MF_test01 \\
        --calib ../config/mocap_to_base_footprint.yaml
"""
import argparse
import json
import math
import statistics
import sys
from pathlib import Path

import numpy as np

from mocap_bag_io import bag_topics, load_bag, load_calib, nearest, rigid_body_pose_xy_yaw

DEFAULT_ODOM_TOPICS = ("/odom", "/ekf_odom", "/scan_odom")


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
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def mocap_to_base_footprint(rb_x, rb_y, rb_yaw, calib):
    """rigid_body[0].pose (frame map) -> pose de base_footprint (frame map),
    aplicando el offset fijo de montaje (dx, dy, dyaw) de calibrate_mocap_to_laser.py.
    Misma formula que la primera mitad de mocap_bag_io.project_map_to_laser,
    sin el paso final (esa funcion sigue hasta el frame laser; aqui hace falta
    quedarse en base_footprint, frame comun con /odom)."""
    dx, dy, dyaw = calib
    c, s = math.cos(rb_yaw), math.sin(rb_yaw)
    bx = rb_x + c * dx - s * dy
    by = rb_y + s * dx + c * dy
    byaw = wrap_angle(rb_yaw + dyaw)
    return bx, by, byaw


def load_gt_trajectory(rbs, calib):
    return [(t, *mocap_to_base_footprint(*rigid_body_pose_xy_yaw(msg), calib)) for t, msg in rbs]


def load_odom_trajectory(msgs):
    out = []
    for t, msg in msgs:
        p = msg.pose.pose.position
        out.append((t, p.x, p.y, yaw_from_quat(msg.pose.pose.orientation)))
    return out


def pair_by_time(odom_traj, gt_traj, max_dt_ns):
    gt_list = [(t, x, y, yaw) for t, x, y, yaw in gt_traj]
    gt_index = [(t, (x, y, yaw)) for t, x, y, yaw in gt_list]
    pairs = []
    for t, ox, oy, oyaw in odom_traj:
        best = nearest(gt_index, t, max_dt_ns)
        if best is None:
            continue
        gx, gy, gyaw = best[1]
        pairs.append((t, ox, oy, oyaw, gx, gy, gyaw))
    return pairs


def align_2d(src_xy, dst_xy):
    """Rotacion + traslacion (sin escala) que mejor lleva src_xy a dst_xy en
    minimos cuadrados (Umeyama/Kabsch restringido a 2D, via SVD). Devuelve
    (theta, tx, ty) tal que dst ~= R(theta) @ src + t."""
    src = np.asarray(src_xy)
    dst = np.asarray(dst_xy)
    src_c = src.mean(axis=0)
    dst_c = dst.mean(axis=0)
    src0 = src - src_c
    dst0 = dst - dst_c
    H = src0.T @ dst0
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, d]) @ U.T
    theta = math.atan2(R[1, 0], R[0, 0])
    t = dst_c - R @ src_c
    return theta, t[0], t[1]


def apply_2d(theta, tx, ty, x, y):
    c, s = math.cos(theta), math.sin(theta)
    return c * x - s * y + tx, s * x + c * y + ty


def path_length(traj_xy):
    return sum(math.hypot(x2 - x1, y2 - y1) for (x1, y1), (x2, y2) in zip(traj_xy, traj_xy[1:]))


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
    ap.add_argument("--eval-bag", required=True, help="bag grabado con ros2 bag record durante test_laser_imu_odom.launch.py")
    ap.add_argument("--odom-topic", default=None, help=f"por defecto autodetecta entre {DEFAULT_ODOM_TOPICS}")
    ap.add_argument("--calib", default=str(Path(__file__).resolve().parent.parent / "config" / "mocap_to_base_footprint.yaml"))
    ap.add_argument("--max-dt-ms", type=float, default=50.0, help="tolerancia de emparejado odom<->rigid_bodies")
    ap.add_argument("--out", default=None, help="JSON de salida con las metricas")
    args = ap.parse_args()

    calib = load_calib(args.calib)
    print(f"Calibracion mocap->base_footprint: dx={calib[0]:+.3f} dy={calib[1]:+.3f} "
          f"dyaw={math.degrees(calib[2]):+.2f} deg  ({args.calib})")

    odom_topic = discover_odom_topic(args.eval_bag, args.odom_topic)
    print(f"Topic de odometria evaluado: {odom_topic}")

    data = load_bag(args.eval_bag, topics=(odom_topic, "/rigid_bodies"), require_all=True)
    gt_traj = load_gt_trajectory(data["/rigid_bodies"], calib)
    odom_traj = load_odom_trajectory(data[odom_topic])
    if not gt_traj or not odom_traj:
        sys.exit(f"{args.eval_bag}: sin datos en /rigid_bodies o {odom_topic}")

    max_dt_ns = int(args.max_dt_ms * 1e6)
    pairs = pair_by_time(odom_traj, gt_traj, max_dt_ns)
    metrics = evaluate(pairs)
    print_report(odom_topic, metrics)

    if args.out:
        out = {**metrics, "_meta": {"eval_bag": args.eval_bag, "odom_topic": odom_topic,
                                     "calib": args.calib, "max_dt_ms": args.max_dt_ms}}
        with open(args.out, "w") as f:
            json.dump(out, f, indent=2, ensure_ascii=False)
        print(f"\nGuardado en {args.out}")


if __name__ == "__main__":
    main()
