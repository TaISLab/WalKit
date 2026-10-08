#!/usr/bin/env python3
"""Calcula, desde el mocap etiquetado, los puntos 3/4/5/8/9/11 del plan
(los que no cubren gait_metrics.py/posture_metrics.py/cog_estimation.py):
angulo de codo, altura/distancia de muñeca, reparto de peso en manetas,
curvatura de trayectoria, oscilacion vertical del CoM, e indice compuesto
de inestabilidad exploratorio (ver extra_metrics.py para las formulas y
limitaciones de cada uno).

Uso:
    python3 compute_extra_metrics_from_mocap.py --bags-dir $DATASETS/labeled_bags \\
        --out /tmp/mocap_extra_results.json
"""
import argparse
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mocap_eval import bag_io, cog_estimation, contact_events, extra_metrics  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from compute_gait_from_mocap import DEFAULT_CALIB  # noqa: E402
from compute_cog_from_mocap import read_test_config_row, DEFAULT_TEST_CONFIG  # noqa: E402

HANDLE_CALIB_YAML = (
    Path(__file__).resolve().parent.parent.parent
    / "walker_loads" / "config" / "handle_calib.yaml"
)


def _mean_std(values):
    vals = [v for v in values if not (isinstance(v, float) and math.isnan(v))]
    if not vals:
        return float("nan"), float("nan")
    if len(vals) == 1:
        return vals[0], float("nan")
    return statistics.mean(vals), statistics.stdev(vals)


def compute_bag(bag_dir, test_config_file, calib, speed_threshold, min_gap_s, max_dt_ns):
    fields = read_test_config_row(test_config_file, bag_dir.name)
    user_weight_kg, user_gender = float(fields[4]), fields[6]

    data = bag_io.load_bag(
        bag_dir, topics=("/labeled_markers", "/rigid_bodies", "/left_handle", "/right_handle"),
        require_all=False)
    mk_msgs, rb_msgs = data.get("/labeled_markers", []), data.get("/rigid_bodies", [])
    if not mk_msgs or not rb_msgs:
        return None

    tag_window = bag_io.read_tag_window(bag_dir)
    if tag_window is not None:
        t0, t1 = tag_window
        mk_msgs = bag_io.clip_track(mk_msgs, t0, t1)
        rb_msgs = bag_io.clip_track(rb_msgs, t0, t1)
    if not mk_msgs or not rb_msgs:
        return None

    tracks = {k: bag_io.marker_track(mk_msgs, k) for k in cog_estimation.REQUIRED_KEYS}
    rb_track_yaw = bag_io.rigid_body_track_xy_yaw(rb_msgs)

    # --- Punto 3: angulo de codo ---
    l_elbow = extra_metrics.elbow_angle_series(tracks["l_shoulder"], tracks["l_elbow"], tracks["l_wrist"], max_dt_ns)
    r_elbow = extra_metrics.elbow_angle_series(tracks["r_shoulder"], tracks["r_elbow"], tracks["r_wrist"], max_dt_ns)
    l_elbow_mean, l_elbow_std = _mean_std([a for _, a in l_elbow])
    r_elbow_mean, r_elbow_std = _mean_std([a for _, a in r_elbow])

    # --- Punto 4: altura/distancia de muñeca ---
    l_wrist = extra_metrics.wrist_height_and_ankle_dist_series(tracks["l_wrist"], tracks["l_ankle"], max_dt_ns)
    r_wrist = extra_metrics.wrist_height_and_ankle_dist_series(tracks["r_wrist"], tracks["r_ankle"], max_dt_ns)
    l_wh_mean, l_wh_std = _mean_std([h for _, h, _ in l_wrist])
    l_wd_mean, l_wd_std = _mean_std([d for _, _, d in l_wrist])
    r_wh_mean, r_wh_std = _mean_std([h for _, h, _ in r_wrist])
    r_wd_mean, r_wd_std = _mean_std([d for _, _, d in r_wrist])

    # --- Punto 5: reparto de peso en manetas (sensor real + calibracion) ---
    left_h, right_h = data.get("/left_handle", []), data.get("/right_handle", [])
    if tag_window is not None:
        left_h = bag_io.clip_track(left_h, t0, t1)
        right_h = bag_io.clip_track(right_h, t0, t1)
    hw_frac_mean = hw_frac_std = hw_left_kg_mean = hw_right_kg_mean = float("nan")
    if left_h and right_h and HANDLE_CALIB_YAML.exists():
        interp_l, interp_r = extra_metrics.build_handle_calibration(str(HANDLE_CALIB_YAML))
        hw_series = extra_metrics.handle_weight_fraction_series(left_h, right_h, interp_l, interp_r, user_weight_kg)
        hw_frac_mean, hw_frac_std = _mean_std([f for _, _, _, f in hw_series])
        hw_left_kg_mean, _ = _mean_std([lk for _, lk, _, _ in hw_series])
        hw_right_kg_mean, _ = _mean_std([rk for _, _, rk, _ in hw_series])

    # --- Punto 8: curvatura/yaw del andador ---
    yaw_series = extra_metrics.yaw_rate_series(rb_track_yaw)
    yaw_abs_mean, yaw_abs_std = _mean_std([abs(yr) for _, yr, _ in yaw_series])
    radii = [r for _, _, r in yaw_series if not math.isnan(r) and r < 50]  # descarta "recto" (radio ~inf)
    radius_mean, radius_std = _mean_std(radii)

    # --- Punto 9: oscilacion vertical del CoM ---
    com_series = cog_estimation.com_series(tracks, user_gender, max_dt_ns, weight_kg=user_weight_kg)
    com_z_std, com_z_range, com_z_mean = extra_metrics.vertical_com_oscillation(com_series)

    # --- Punto 11: indice compuesto de inestabilidad (exploratorio) ---
    from mocap_eval import posture_metrics
    lean = posture_metrics.trunk_lean_series(
        tracks["l_shoulder"], tracks["r_shoulder"], tracks["l_hip"], tracks["r_hip"], max_dt_ns)
    _, frontal_std = _mean_std([r[3] for r in lean])

    l_events = contact_events.detect_stance_events(tracks["l_ankle"], speed_threshold, min_gap_s)
    r_events = contact_events.detect_stance_events(tracks["r_ankle"], speed_threshold, min_gap_s)
    l_intervals = contact_events.detect_stance_intervals(tracks["l_ankle"], speed_threshold)
    r_intervals = contact_events.detect_stance_intervals(tracks["r_ankle"], speed_threshold)

    step_time_cv = float("nan")
    dsp = float("nan")
    if len(l_events) >= 2 and len(r_events) >= 2:
        from mocap_eval import gait_metrics
        events = ([(t, "left", p) for t, p in l_events] + [(t, "right", p) for t, p in r_events])
        left_log, right_log = gait_metrics.process_events(events)
        all_step_times = left_log.step_times + right_log.step_times
        if len(all_step_times) >= 2:
            m, s = statistics.mean(all_step_times), statistics.stdev(all_step_times)
            step_time_cv = s / m if m else float("nan")
        dsp = posture_metrics.double_support_fraction(l_intervals, r_intervals)

    instability_index = extra_metrics.composite_instability_index(frontal_std, step_time_cv, dsp)

    return {
        "elbow_left_deg_mean": l_elbow_mean, "elbow_left_deg_std": l_elbow_std,
        "elbow_right_deg_mean": r_elbow_mean, "elbow_right_deg_std": r_elbow_std,
        "wrist_left_height_m_mean": l_wh_mean, "wrist_left_height_m_std": l_wh_std,
        "wrist_left_dist_ankle_m_mean": l_wd_mean, "wrist_left_dist_ankle_m_std": l_wd_std,
        "wrist_right_height_m_mean": r_wh_mean, "wrist_right_height_m_std": r_wh_std,
        "wrist_right_dist_ankle_m_mean": r_wd_mean, "wrist_right_dist_ankle_m_std": r_wd_std,
        "handle_weight_fraction_mean": hw_frac_mean, "handle_weight_fraction_std": hw_frac_std,
        "handle_left_kg_mean": hw_left_kg_mean, "handle_right_kg_mean": hw_right_kg_mean,
        "yaw_rate_abs_deg_s_mean": yaw_abs_mean, "yaw_rate_abs_deg_s_std": yaw_abs_std,
        "curvature_radius_m_mean": radius_mean, "curvature_radius_m_std": radius_std,
        "com_z_std_m": com_z_std, "com_z_range_m": com_z_range, "com_z_mean_m": com_z_mean,
        "lateral_sway_frontal_std_deg": frontal_std, "step_time_cv": step_time_cv,
        "double_support_fraction": dsp, "instability_index": instability_index,
    }


def fmt(v, nd=3):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "-"
    return f"{v:.{nd}f}"


def print_table(results):
    cols = ["bag", "elb_L", "elb_R", "wr_h_L", "wr_h_R", "hw_frac", "yaw_abs", "radius", "com_z_std", "instab"]
    rows = []
    for bag_name, r in results.items():
        if r is None:
            rows.append([bag_name] + ["-"] * (len(cols) - 1))
            continue
        rows.append([
            bag_name, fmt(r["elbow_left_deg_mean"], 1), fmt(r["elbow_right_deg_mean"], 1),
            fmt(r["wrist_left_height_m_mean"]), fmt(r["wrist_right_height_m_mean"]),
            fmt(r["handle_weight_fraction_mean"]), fmt(r["yaw_rate_abs_deg_s_mean"], 1),
            fmt(r["curvature_radius_m_mean"], 1), fmt(r["com_z_std_m"]), fmt(r["instability_index"], 1),
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
    src.add_argument("--bag")
    src.add_argument("--bags-dir")
    ap.add_argument("--test-config-file", default=DEFAULT_TEST_CONFIG)
    ap.add_argument("--calib", default=str(DEFAULT_CALIB))
    ap.add_argument("--speed-threshold", type=float, default=0.25)
    ap.add_argument("--min-gap-s", type=float, default=0.3)
    ap.add_argument("--max-dt-ms", type=float, default=50.0)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    bag_dirs = [Path(args.bag)] if args.bag else bag_io.find_labeled_bags(args.bags_dir)
    if not bag_dirs:
        sys.exit("No hay bags")

    calib = bag_io.load_calib(args.calib)
    max_dt_ns = int(args.max_dt_ms * 1e6)
    results = {}
    for bag_dir in bag_dirs:
        try:
            results[bag_dir.name] = compute_bag(
                bag_dir, args.test_config_file, calib, args.speed_threshold, args.min_gap_s, max_dt_ns)
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
