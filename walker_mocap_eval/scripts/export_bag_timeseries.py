#!/usr/bin/env python3
"""Exporta, por bag, TODAS las series temporales comparables mocap<->andador
a CSV, con columnas consistentes entre ambas fuentes, para poder graficar
cualquier parametro de las dos en cualquier rosbag (t_s = segundos desde el
inicio de la ventana /tag, mismo origen de tiempo en todos los CSV de un
mismo bag).

Lado MOCAP (bag_io.py + gait_metrics/posture_metrics/cog_estimation/
extra_metrics.py, sin replay -- calculado directamente de /labeled_markers,
/rigid_bodies, /left_handle, /right_handle):
  {bag}_mocap_cog.csv           t_s,x_m,y_m,z_m           (frame laser, x/y; z sin proyectar)
  {bag}_mocap_ankle_left.csv    t_s,x_m,y_m,z_m           (frame laser -- equivalente a StepStamped.position)
  {bag}_mocap_ankle_right.csv   idem
  {bag}_mocap_trunk_lean.csv    t_s,total_deg,sagittal_deg,frontal_deg
  {bag}_mocap_elbow_angle.csv   t_s,left_deg,right_deg
  {bag}_mocap_wrist.csv         t_s,left_height_m,left_dist_ankle_m,right_height_m,right_dist_ankle_m
  {bag}_mocap_walker_yaw.csv    t_s,yaw_rate_deg_s,curvature_radius_m
  {bag}_mocap_handle_weight.csv t_s,left_kg,right_kg,total_fraction  (sensor real del andador, sin replay: solo calibracion offline)

Series de la AMPLIACION (sin replay; tambien con --ampliacion-only):
  {bag}_imu_real_accel.csv         t_s,ap_m_s2,ml_m_s2,v_m_s2   (IMU real /bwt901cl/imu_filtered, crudo; asume x=AP,y=ML,z=V)
  {bag}_mocap_walker_accel.csv     t_s,ap_m_s2,ml_m_s2          (2a derivada de rigid_bodies[0], remuestreo 60 Hz + suavizado)
  {bag}_mocap_handle_symmetry.csv  t_s,left_kg,right_kg,si_pct  (Symmetry Index por muestra; kg>=0, SI=NaN si L+R<0.5 kg)

Lado ANDADOR (replay real de replay_offline.launch.py, recorta a la misma
ventana /tag, UNA sola pasada de replay graba todos los topics de golpe):
  {bag}_walker_cog.csv                t_s,x_m,y_m,z_m       (/support_centroid)
  {bag}_walker_left_loads.csv         t_s,x_m,y_m,load_kg,speed_x,speed_y   (/left_loads)
  {bag}_walker_right_loads.csv        idem                                  (/right_loads)
  {bag}_walker_left_gait_stats.csv    t_s,spt_s,sdt_s,spl_m,sdl_m           (/left_gait_stats)
  {bag}_walker_right_gait_stats.csv   idem                                  (/right_gait_stats)
  {bag}_walker_global_gait_stats.csv  t_s,nos,d_m,cad,wv                    (/global_gait_stats)
  {bag}_walker_hand_loads.csv         t_s,left_kg,right_kg                  (/left_hand_loads, /right_hand_loads)

Uso:
    python3 export_bag_timeseries.py --bag $DATASETS/labeled_bags/CA_test01 --out-dir /tmp/timeseries
    python3 export_bag_timeseries.py --bags-dir $DATASETS/labeled_bags --out-dir /tmp/timeseries
"""
import argparse
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mocap_eval import bag_io, cog_estimation, extra_metrics  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from compute_cog_from_mocap import read_test_config_row  # noqa: E402
from compare_gait_mocap_vs_walker import tag_window_offset_and_duration  # noqa: E402

ROS_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP = str(Path.home() / "workspace" / "walker_ws" / "install" / "setup.bash")
ROS_DOMAIN_ID_EVAL = "94"
DEFAULT_TEST_CONFIG = str(Path.home() / "bagsFolder_unified" / "test_config.txt")
DEFAULT_CALIB = (
    Path(__file__).resolve().parent.parent.parent
    / "walker_step_detector" / "config" / "mocap_to_base_footprint.yaml"
)
HANDLE_CALIB_YAML = (
    Path(__file__).resolve().parent.parent.parent
    / "walker_loads" / "config" / "handle_calib.yaml"
)

WALKER_TOPICS = [
    "/support_centroid", "/left_loads", "/right_loads",
    "/left_gait_stats", "/right_gait_stats", "/global_gait_stats",
    "/left_hand_loads", "/right_hand_loads",
]


def sh(cmd):
    full = f"source {ROS_SETUP}; source {WS_SETUP}; export ROS_DOMAIN_ID={ROS_DOMAIN_ID_EVAL}; exec {cmd}"
    return subprocess.Popen(["bash", "-c", full], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             preexec_fn=os.setsid)


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


def write_csv(path, header, rows):
    with open(path, "w") as f:
        f.write(",".join(header) + "\n")
        for row in rows:
            f.write(",".join(f"{v:.5f}" if isinstance(v, float) else str(v) for v in row) + "\n")


def export_mocap_side(bag_dir, out_dir, calib, test_config_file, max_dt_ns, t0_ref):
    fields = read_test_config_row(test_config_file, bag_dir.name)
    user_weight_kg, user_gender = float(fields[4]), fields[6]

    data = bag_io.load_bag(bag_dir, topics=("/labeled_markers", "/rigid_bodies", "/left_handle", "/right_handle"),
                            require_all=False)
    mk_msgs, rb_msgs = data.get("/labeled_markers", []), data.get("/rigid_bodies", [])
    if not mk_msgs or not rb_msgs:
        return False

    tag_window = bag_io.read_tag_window(bag_dir)
    if tag_window is not None:
        t0, t1 = tag_window
        mk_msgs = bag_io.clip_track(mk_msgs, t0, t1)
        rb_msgs = bag_io.clip_track(rb_msgs, t0, t1)
    if not mk_msgs or not rb_msgs:
        return False

    rb_track_yaw = bag_io.rigid_body_track_xy_yaw(rb_msgs)
    rb_yaw_q = bag_io.indexed_nearest(rb_track_yaw)

    def project(x, y, t):
        rb = rb_yaw_q(t, max_dt_ns)
        if rb is None:
            return None
        rb_x, rb_y, rb_yaw = rb[1]
        return bag_io.project_map_to_laser(x, y, rb_x, rb_y, rb_yaw, calib)

    tracks = {k: bag_io.marker_track(mk_msgs, k) for k in cog_estimation.REQUIRED_KEYS}

    # --- CoG ---
    com_series = cog_estimation.com_series(tracks, user_gender, max_dt_ns, weight_kg=user_weight_kg)
    rows = []
    for t, (x, y, z) in com_series:
        p = project(x, y, t)
        if p is None:
            continue
        rows.append(((t - t0_ref) / 1e9, p[0], p[1], z))
    write_csv(out_dir / f"{bag_dir.name}_mocap_cog.csv", ["t_s", "x_m", "y_m", "z_m"], rows)

    # --- Tobillos (equivalente a StepStamped.position) ---
    for side in ("l", "r"):
        rows = []
        for t, (x, y, z) in tracks[f"{side}_ankle"]:
            p = project(x, y, t)
            if p is None:
                continue
            rows.append(((t - t0_ref) / 1e9, p[0], p[1], z))
        name = "left" if side == "l" else "right"
        write_csv(out_dir / f"{bag_dir.name}_mocap_ankle_{name}.csv", ["t_s", "x_m", "y_m", "z_m"], rows)

    # --- Inclinacion de tronco ---
    from mocap_eval import posture_metrics
    lean = posture_metrics.trunk_lean_series(
        tracks["l_shoulder"], tracks["r_shoulder"], tracks["l_hip"], tracks["r_hip"], max_dt_ns)
    rows = [((t - t0_ref) / 1e9, total, sag, fro) for t, total, sag, fro in lean]
    write_csv(out_dir / f"{bag_dir.name}_mocap_trunk_lean.csv",
              ["t_s", "total_deg", "sagittal_deg", "frontal_deg"], rows)

    # --- Angulo de codo ---
    l_elbow = extra_metrics.elbow_angle_series(tracks["l_shoulder"], tracks["l_elbow"], tracks["l_wrist"], max_dt_ns)
    r_elbow = extra_metrics.elbow_angle_series(tracks["r_shoulder"], tracks["r_elbow"], tracks["r_wrist"], max_dt_ns)
    l_elbow_q = bag_io.indexed_nearest(l_elbow) if l_elbow else None
    rows = []
    for t, r_deg in r_elbow:
        l_match = l_elbow_q(t, max_dt_ns) if l_elbow_q else None
        rows.append(((t - t0_ref) / 1e9, l_match[1] if l_match else float("nan"), r_deg))
    write_csv(out_dir / f"{bag_dir.name}_mocap_elbow_angle.csv", ["t_s", "left_deg", "right_deg"], rows)

    # --- Muñeca: altura y distancia al tobillo del mismo lado ---
    l_wrist_s = extra_metrics.wrist_height_and_ankle_dist_series(tracks["l_wrist"], tracks["l_ankle"], max_dt_ns)
    r_wrist_s = extra_metrics.wrist_height_and_ankle_dist_series(tracks["r_wrist"], tracks["r_ankle"], max_dt_ns)
    r_wrist_q = bag_io.indexed_nearest([(t, (h, d)) for t, h, d in r_wrist_s]) if r_wrist_s else None
    rows = []
    for t, lh, ld in l_wrist_s:
        rm = r_wrist_q(t, max_dt_ns) if r_wrist_q else None
        rh, rd = rm[1] if rm else (float("nan"), float("nan"))
        rows.append(((t - t0_ref) / 1e9, lh, ld, rh, rd))
    write_csv(out_dir / f"{bag_dir.name}_mocap_wrist.csv",
              ["t_s", "left_height_m", "left_dist_ankle_m", "right_height_m", "right_dist_ankle_m"], rows)

    # --- Yaw/curvatura del andador (mocap ground truth) ---
    yaw_series = extra_metrics.yaw_rate_series(rb_track_yaw)
    rows = [((t - t0_ref) / 1e9, yr, rad) for t, yr, rad in yaw_series]
    write_csv(out_dir / f"{bag_dir.name}_mocap_walker_yaw.csv", ["t_s", "yaw_rate_deg_s", "curvature_radius_m"], rows)

    # --- Peso en manetas (sensor real, calibracion offline, sin replay) ---
    left_h, right_h = data.get("/left_handle", []), data.get("/right_handle", [])
    if tag_window is not None:
        left_h = bag_io.clip_track(left_h, t0, t1)
        right_h = bag_io.clip_track(right_h, t0, t1)
    if left_h and right_h and os.path.exists(HANDLE_CALIB_YAML):
        interp_l, interp_r = extra_metrics.build_handle_calibration(HANDLE_CALIB_YAML)
        hw_series = extra_metrics.handle_weight_fraction_series(left_h, right_h, interp_l, interp_r, user_weight_kg)
        rows = [((t - t0_ref) / 1e9, lk, rk, frac) for t, lk, rk, frac in hw_series]
        write_csv(out_dir / f"{bag_dir.name}_mocap_handle_weight.csv",
                  ["t_s", "left_kg", "right_kg", "total_fraction"], rows)

    return True


def export_ampliacion_series(bag_dir, out_dir, t0_ref):
    """Series de la ampliacion (sin replay, solo lectura de bag):
    IMU real y aceleracion del andador por mocap (punto 1), y simetria de
    fuerza en manetas muestra a muestra (punto 2)."""
    from mocap_eval import imu_metrics

    tag_window = bag_io.read_tag_window(bag_dir)
    data = bag_io.load_bag(
        bag_dir, topics=("/bwt901cl/imu_filtered", "/bwt901cl/imu", "/rigid_bodies", "/left_handle", "/right_handle"),
        require_all=False)

    def clip(msgs):
        return bag_io.clip_track(msgs, *tag_window) if tag_window is not None else msgs

    name = bag_dir.name
    imu = clip(data.get("/bwt901cl/imu_filtered") or data.get("/bwt901cl/imu") or [])
    if imu:
        rows = [((t - t0_ref) / 1e9, m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)
                for t, m in imu]
        write_csv(out_dir / f"{name}_imu_real_accel.csv", ["t_s", "ap_m_s2", "ml_m_s2", "v_m_s2"], rows)

    rb = clip(data.get("/rigid_bodies", []))
    if rb:
        track = bag_io.rigid_body_track_xy_yaw(rb)
        offset_s = (track[0][0] - t0_ref) / 1e9
        accel = imu_metrics.walker_accel_ap_ml(track)
        rows = [(offset_s + t, ap, ml) for t, ap, ml in accel]
        write_csv(out_dir / f"{name}_mocap_walker_accel.csv", ["t_s", "ap_m_s2", "ml_m_s2"], rows)

    left_h, right_h = clip(data.get("/left_handle", [])), clip(data.get("/right_handle", []))
    if left_h and right_h and os.path.exists(HANDLE_CALIB_YAML):
        interp_l, interp_r = extra_metrics.build_handle_calibration(HANDLE_CALIB_YAML)
        series = extra_metrics.handle_force_symmetry_series(left_h, right_h, interp_l, interp_r)
        rows = [((t - t0_ref) / 1e9, lk, rk, si) for t, si, lk, rk in series]
        write_csv(out_dir / f"{name}_mocap_handle_symmetry.csv", ["t_s", "left_kg", "right_kg", "si_pct"], rows)


def run_replay_and_export_walker_side(bag_dir, out_dir, test_config_file, rate, startup_wait, t0_ref):
    window = tag_window_offset_and_duration(bag_dir)
    start_offset_s, duration = window if window is not None else (0.0, bag_io.bag_duration_seconds(bag_dir))

    tmp_bag = out_dir / f"_tmp_{bag_dir.name}"
    if tmp_bag.exists():
        shutil.rmtree(tmp_bag)

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
            return False
        record = sh(f"ros2 bag record -s sqlite3 -o {tmp_bag} " + " ".join(WALKER_TOPICS))
        time.sleep(1.5)
        time.sleep(duration / rate + 3.0)
    finally:
        stop(record)
        stop(launch)
        time.sleep(1.0)

    if not tmp_bag.exists() or not any(tmp_bag.iterdir()):
        print("  [!] no se genero nada grabable", file=sys.stderr)
        return False

    data = bag_io.load_bag(tmp_bag, topics=tuple(WALKER_TOPICS), require_all=False)
    shutil.rmtree(tmp_bag, ignore_errors=True)

    def stamp_ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    # Todos los topics del andador ya llevan timestamp de bag real (no
    # reloj de pared) en su header:
    #   - /left_loads, /right_loads (StepStamped, .position.header.stamp),
    #     /support_centroid (PointStamped), /left_hand_loads,
    #     /right_hand_loads (ForceStamped): reenvian el header de su
    #     entrada (detect_steps, ya corregido a tiempo de escaner; o
    #     /left_handle//right_handle crudo, replay puro).
    #   - /left_gait_stats, /right_gait_stats (GaitStamped) y
    #     /global_gait_stats (WalkStats, header anadido en walker_msgs):
    #     gait_monitor_speed.py les pone self.get_clock().now(), que con
    #     use_sim_time:=true + `ros2 bag play --clock` en
    #     replay_offline.launch.py devuelve tiempo de bag real (no reloj de
    #     pared) -- ver el comentario de ese nodo en el launch file.
    # Todos, por tanto, comparables directamente contra t0_ref (mismo
    # origen que el lado mocap).
    def rel_from_msg_header(m):
        return (stamp_ns(m.header.stamp) - t0_ref) / 1e9

    def rel_from_step_header(m):
        return (stamp_ns(m.position.header.stamp) - t0_ref) / 1e9

    cc = data.get("/support_centroid", [])
    write_csv(out_dir / f"{bag_dir.name}_walker_cog.csv", ["t_s", "x_m", "y_m", "z_m"],
              [(rel_from_msg_header(m), m.point.x, m.point.y, m.point.z) for t, m in cc])

    for topic, name in (("/left_loads", "left_loads"), ("/right_loads", "right_loads")):
        msgs = data.get(topic, [])
        write_csv(out_dir / f"{bag_dir.name}_walker_{name}.csv",
                  ["t_s", "x_m", "y_m", "load_kg", "speed_x", "speed_y"],
                  [(rel_from_step_header(m), m.position.point.x, m.position.point.y, m.load, m.speed.x, m.speed.y)
                   for t, m in msgs])

    for topic, name in (("/left_gait_stats", "left_gait_stats"), ("/right_gait_stats", "right_gait_stats")):
        msgs = data.get(topic, [])
        write_csv(out_dir / f"{bag_dir.name}_walker_{name}.csv",
                  ["t_s", "spt_s", "sdt_s", "spl_m", "sdl_m"],
                  [(rel_from_msg_header(m), m.spt, m.sdt, m.spl, m.sdl) for t, m in msgs])

    msgs = data.get("/global_gait_stats", [])
    write_csv(out_dir / f"{bag_dir.name}_walker_global_gait_stats.csv",
              ["t_s", "nos", "d_m", "cad", "wv"],
              [(rel_from_msg_header(m), m.nos, m.d, m.cad, m.wv) for t, m in msgs])

    lh, rh = data.get("/left_hand_loads", []), data.get("/right_hand_loads", [])
    lh_q = bag_io.indexed_nearest([(t, m.force) for t, m in lh]) if lh else None
    rows = []
    for t, m in rh:
        lmatch = lh_q(t, int(0.2e9)) if lh_q else None
        rows.append((rel_from_msg_header(m), lmatch[1] if lmatch else float("nan"), m.force))
    write_csv(out_dir / f"{bag_dir.name}_walker_hand_loads.csv", ["t_s", "left_kg", "right_kg"], rows)

    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag")
    src.add_argument("--bags-dir")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--test-config-file", default=DEFAULT_TEST_CONFIG)
    ap.add_argument("--calib", default=str(DEFAULT_CALIB))
    ap.add_argument("--max-dt-ms", type=float, default=50.0)
    ap.add_argument("--rate", type=float, default=4.0)
    ap.add_argument("--startup-wait", type=float, default=4.0)
    ap.add_argument("--limit", type=int, default=None)
    ap.add_argument("--mocap-only", action="store_true", help="no lanzar el replay real, solo los CSV de mocap")
    ap.add_argument("--ampliacion-only", action="store_true",
                    help="solo las series de la ampliacion (IMU, aceleracion mocap, simetria de maneta); "
                         "no recalcula el resto ni lanza replay")
    args = ap.parse_args()

    bag_dirs = [Path(args.bag)] if args.bag else bag_io.find_labeled_bags(args.bags_dir)
    if args.limit:
        bag_dirs = bag_dirs[:args.limit]
    if not bag_dirs:
        sys.exit("No hay bags")

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    calib = bag_io.load_calib(args.calib)
    max_dt_ns = int(args.max_dt_ms * 1e6)

    for i, bag_dir in enumerate(bag_dirs):
        print(f"[{i + 1}/{len(bag_dirs)}] {bag_dir.name}")
        tag_window = bag_io.read_tag_window(bag_dir)
        t0_ref = tag_window[0] if tag_window is not None else 0

        if args.ampliacion_only:
            try:
                export_ampliacion_series(bag_dir, out_dir, t0_ref)
            except (ValueError, RuntimeError) as e:
                print(f"  [!] ampliacion: {e}", file=sys.stderr)
            continue

        try:
            ok = export_mocap_side(bag_dir, out_dir, calib, args.test_config_file, max_dt_ns, t0_ref)
        except (ValueError, RuntimeError) as e:
            print(f"  [!] mocap: {e}", file=sys.stderr)
            ok = False
        if not ok:
            print("  [!] mocap insuficiente, se salta el bag entero")
            continue

        export_ampliacion_series(bag_dir, out_dir, t0_ref)

        if not args.mocap_only:
            run_replay_and_export_walker_side(bag_dir, out_dir, args.test_config_file, args.rate,
                                               args.startup_wait, t0_ref)

    print("\nHecho.")


if __name__ == "__main__":
    main()
