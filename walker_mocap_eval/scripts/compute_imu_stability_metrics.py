#!/usr/bin/env python3
"""Punto 1 de la ampliacion del informe: metricas de estabilidad derivadas
de aceleracion (Harmonic Ratio, RMS, regularidad de paso/zancada por
autocorrelacion -- ver mocap_eval/imu_metrics.py para las formulas) sobre
DOS fuentes independientes, sin reproducir ningun bag:

  - IMU real del andador (/bwt901cl/imu_filtered, grabado en todos los bags).
  - Equivalente estimado por mocap (doble derivada de la posicion del rigid
    body del andador, proyectada a ejes AP/ML del propio andador).

No hay comparacion a nivel de muestra (serian dos señales de aceleracion
con ruido de naturaleza muy distinta -- sensor real vs. doble derivada
numerica de una posicion -- dificiles de alinear con sentido); se comparan
los INDICES resumen (RMS/HR/Ad1/Ad2) calculados independientemente por cada
fuente sobre la misma ventana /tag de cada bag.

Uso:
    python3 compute_imu_stability_metrics.py --bags-dir $DATASETS \\
        --gait-json /tmp/report_gait_mocap.json --out /tmp/report_imu_mocap.json
"""
import argparse
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mocap_eval import bag_io, imu_metrics  # noqa: E402

IMU_TOPIC_PREFERRED = "/bwt901cl/imu_filtered"
IMU_TOPIC_FALLBACK = "/bwt901cl/imu"


def _mean_dt(ts):
    diffs = [b - a for a, b in zip(ts, ts[1:]) if b > a]
    return statistics.median(diffs) if diffs else float("nan")


def read_real_imu_ap_ml_v(bag_dir, t0, t1):
    data = bag_io.load_bag(bag_dir, topics=(IMU_TOPIC_PREFERRED, IMU_TOPIC_FALLBACK), require_all=False)
    msgs = data.get(IMU_TOPIC_PREFERRED) or data.get(IMU_TOPIC_FALLBACK)
    if not msgs:
        return None
    msgs = [(t, m) for t, m in msgs if t0 <= t <= t1]
    if len(msgs) < 8:
        return None
    t0_ns = msgs[0][0]
    ts = [(t - t0_ns) / 1e9 for t, _ in msgs]
    # Convencion de montaje asumida (no verificada fisicamente): x=AP
    # (longitudinal, direccion de avance), y=ML (lateral), z=V (vertical).
    ap = [m.linear_acceleration.x for _, m in msgs]
    ml = [m.linear_acceleration.y for _, m in msgs]
    v = [m.linear_acceleration.z for _, m in msgs]
    return ts, ap, ml, v


def compute_bag(bag_dir, gait_entry, max_dt_ns):
    tag_window = bag_io.read_tag_window(bag_dir)
    if tag_window is None:
        return None
    t0, t1 = tag_window

    real = read_real_imu_ap_ml_v(bag_dir, t0, t1)

    rb_data = bag_io.load_bag(bag_dir, topics=("/rigid_bodies",), require_all=False)
    rb_msgs = rb_data.get("/rigid_bodies", [])
    rb_msgs = bag_io.clip_track(rb_msgs, t0, t1)
    rb_track_yaw = bag_io.rigid_body_track_xy_yaw(rb_msgs) if rb_msgs else []
    mocap_accel = imu_metrics.walker_accel_ap_ml(rb_track_yaw) if len(rb_track_yaw) > 3 else []

    if gait_entry is None:
        stride_time_s = step_time_s = float("nan")
    else:
        sdt = [v for v in (gait_entry.get("left_sdt"), gait_entry.get("right_sdt"))
               if v is not None and not math.isnan(v)]
        spt = [v for v in (gait_entry.get("left_spt"), gait_entry.get("right_spt"))
               if v is not None and not math.isnan(v)]
        stride_time_s = statistics.mean(sdt) if sdt else float("nan")
        step_time_s = statistics.mean(spt) if spt else float("nan")

    out = {"stride_time_s": stride_time_s, "step_time_s": step_time_s}

    if real is not None:
        ts, ap, ml, v = real
        fs = 1.0 / _mean_dt(ts)
        rt, ap_u = imu_metrics.resample_uniform(ts, ap, fs)
        _, ml_u = imu_metrics.resample_uniform(ts, ml, fs)
        _, v_u = imu_metrics.resample_uniform(ts, v, fs)
        out.update({
            "imu_fs_hz": fs,
            "imu_rms_ap": imu_metrics.rms(ap_u), "imu_rms_ml": imu_metrics.rms(ml_u),
            "imu_rms_v": imu_metrics.rms(v_u),
            "imu_hr_ap": imu_metrics.harmonic_ratio(ap_u, fs, stride_time_s, kind="even_over_odd"),
            "imu_hr_ml": imu_metrics.harmonic_ratio(ml_u, fs, stride_time_s, kind="odd_over_even"),
            "imu_hr_v": imu_metrics.harmonic_ratio(v_u, fs, stride_time_s, kind="even_over_odd"),
        })
        ad1, ad2 = imu_metrics.autocorr_regularity(ap_u, fs, step_time_s, stride_time_s)
        out["imu_ad1_step_regularity"] = ad1
        out["imu_ad2_stride_regularity"] = ad2
    else:
        out.update({k: float("nan") for k in (
            "imu_fs_hz", "imu_rms_ap", "imu_rms_ml", "imu_rms_v", "imu_hr_ap", "imu_hr_ml", "imu_hr_v",
            "imu_ad1_step_regularity", "imu_ad2_stride_regularity")})

    if mocap_accel:
        mt = [t for t, _, _ in mocap_accel]
        map_ = [a for _, a, _ in mocap_accel]
        mml = [m for _, _, m in mocap_accel]
        fs_m = 1.0 / _mean_dt(mt)
        _, ap_u = imu_metrics.resample_uniform(mt, map_, fs_m)
        _, ml_u = imu_metrics.resample_uniform(mt, mml, fs_m)
        out.update({
            "mocap_fs_hz": fs_m,
            "mocap_rms_ap": imu_metrics.rms(ap_u), "mocap_rms_ml": imu_metrics.rms(ml_u),
            "mocap_hr_ap": imu_metrics.harmonic_ratio(ap_u, fs_m, stride_time_s, kind="even_over_odd"),
            "mocap_hr_ml": imu_metrics.harmonic_ratio(ml_u, fs_m, stride_time_s, kind="odd_over_even"),
        })
        ad1, ad2 = imu_metrics.autocorr_regularity(ap_u, fs_m, step_time_s, stride_time_s)
        out["mocap_ad1_step_regularity"] = ad1
        out["mocap_ad2_stride_regularity"] = ad2
    else:
        out.update({k: float("nan") for k in (
            "mocap_fs_hz", "mocap_rms_ap", "mocap_rms_ml", "mocap_hr_ap", "mocap_hr_ml",
            "mocap_ad1_step_regularity", "mocap_ad2_stride_regularity")})

    return out


def fmt(v, nd=3):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "-"
    return f"{v:.{nd}f}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag")
    src.add_argument("--bags-dir")
    ap.add_argument("--gait-json", required=True, help="report_gait_mocap.json (tiempos de paso/zancada)")
    ap.add_argument("--max-dt-ms", type=float, default=50.0)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    bag_dirs = [Path(args.bag)] if args.bag else bag_io.find_labeled_bags(args.bags_dir)
    if not bag_dirs:
        sys.exit("No hay bags")
    gait_all = json.load(open(args.gait_json))
    max_dt_ns = int(args.max_dt_ms * 1e6)

    results = {}
    for bag_dir in bag_dirs:
        try:
            results[bag_dir.name] = compute_bag(bag_dir, gait_all.get(bag_dir.name), max_dt_ns)
        except (ValueError, RuntimeError) as e:
            print(f"[!] {bag_dir}: {e}", file=sys.stderr)
            results[bag_dir.name] = None

    cols = ["bag", "imu_rms_ap", "mocap_rms_ap", "imu_hr_ap", "mocap_hr_ap", "imu_ad1", "mocap_ad1"]
    print()
    print("  ".join(c.ljust(12) for c in cols))
    for name, r in results.items():
        if r is None:
            print(name.ljust(12), "sin datos")
            continue
        print("  ".join(s.ljust(12) for s in [
            name, fmt(r["imu_rms_ap"]), fmt(r["mocap_rms_ap"]), fmt(r["imu_hr_ap"], 2), fmt(r["mocap_hr_ap"], 2),
            fmt(r["imu_ad1_step_regularity"], 2), fmt(r["mocap_ad1_step_regularity"], 2)]))

    if args.out:
        with open(args.out, "w") as f:
            json.dump(results, f, indent=2, ensure_ascii=False)
        print(f"\nGuardado en {args.out}")


if __name__ == "__main__":
    main()
