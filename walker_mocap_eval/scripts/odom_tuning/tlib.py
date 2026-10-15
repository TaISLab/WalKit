"""Utilidades comunes de odom_tuning: rutas, conjuntos de bags, trayectoria de referencia del mocap (cacheada)
y metricas. Usa las mismas funciones que ../eval_odom_against_mocap.py.

Variables de entorno (todas opcionales):
  ODOM_TUNE_DIR  directorio de trabajo con grabaciones y caches (por defecto ~/odom_tuning; NO /tmp: las
                 grabaciones tardan ~25 min en rehacerse y /tmp se vacia al reiniciar)
  BAGS_DIR       carpeta con los rosbags definitivos (por defecto ~/bagsFolder/definitive)
  MOCAP_CALIB    calibracion rigid_body -> base_footprint (por defecto la de eval_odom_against_mocap.py)
  WS_SETUP       install/setup.bash del workspace (por defecto el del arbol de fuentes de este fichero)
  ROS_SETUP      setup.bash de ROS 2 (por defecto /opt/ros/jazzy/setup.bash)
"""
import glob
import math
import os
import pickle
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
SCRIPTS = HERE.parent
WALKIT = HERE.parents[2]
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(SCRIPTS.parent))
import eval_odom_against_mocap as ev                       # noqa: E402
from mocap_eval.bag_io import load_calib                   # noqa: E402

WORK = Path(os.environ.get("ODOM_TUNE_DIR", Path.home() / "odom_tuning"))
BAGS_DIR = Path(os.environ.get("BAGS_DIR", Path.home() / "bagsFolder" / "definitive"))
ROS_SETUP = os.environ.get("ROS_SETUP", "/opt/ros/jazzy/setup.bash")
WS_SETUP = os.environ.get("WS_SETUP", str(HERE.parents[4] / "install" / "setup.bash"))
CALIB = load_calib(os.environ.get("MOCAP_CALIB", str(ev.DEFAULT_CALIB)))
BRINGUP_CONFIG = WALKIT / "walker_bringup" / "config"

ALL_BAGS = sorted(Path(p).parent.name for p in glob.glob(str(BAGS_DIR / "*" / "metadata.yaml")))

_Q = yaml.safe_load(open(HERE / "bag_quality.yaml"))
WARP = ev.load_time_warp(ev.DEFAULT_TIME_WARP)
SCAN_ANOM = sorted(_Q["scan_matching_anomalous"])
MOCAP_SUSPECT = sorted(_Q.get("mocap_suspect") or {})
IMU_ANOM = sorted(_Q.get("imu_gyro_scale") or {})
NO_IMU = sorted(_Q["no_imu"])
CLEAN = [b for b in ALL_BAGS if b not in SCAN_ANOM + MOCAP_SUSPECT + IMU_ANOM + NO_IMU]


def bag_set(name):
    return {"clean": CLEAN, "scan_anom": [b for b in SCAN_ANOM if b in ALL_BAGS],
            "mocap_suspect": [b for b in MOCAP_SUSPECT if b in ALL_BAGS],
            "imu_anom": [b for b in IMU_ANOM if b in ALL_BAGS], "all": ALL_BAGS}[name]


def read_topic(bag, topic):
    """[(t_ns_grabacion, msg)] de un topic de un bag (sqlite3 o mcap, segun su metadata.yaml)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    meta = yaml.safe_load(open(f"{bag}/metadata.yaml"))["rosbag2_bagfile_information"]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=meta.get("storage_identifier", "sqlite3")),
           rosbag2_py.ConverterOptions("", ""))
    tm = {t.name: t.type for t in r.get_all_topics_and_types()}
    if topic not in tm:
        return []
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    cls = get_message(tm[topic])
    out = []
    while r.has_next():
        _, d, t = r.read_next()
        out.append((t, deserialize_message(d, cls)))
    return out


def stamp_ns(h):
    return h.stamp.sec * 10**9 + h.stamp.nanosec


warp_ns = ev.warp_ns        # sello del mocap -> sello del robot (ver time_warp.yaml)


def gt_traj(bag, apply_warp=True):
    """[(t_ns_cabecera, x, y, yaw)] de base_footprint en 'map' segun el mocap, con el eje temporal del mocap
    corregido al del robot si el bag figura en time_warp.yaml; cacheado en WORK/gtcache."""
    w = WARP.get(bag) if apply_warp else None
    cache = WORK / "gtcache" / (f"{bag}.warp.pkl" if w else f"{bag}.pkl")
    if cache.exists():
        return pickle.load(open(cache, "rb"))
    from mocap_odometry import offline
    rows = offline.trajectory_from_bag(BAGS_DIR / bag, 0, CALIB, w)   # misma pose calibrada y mismo warp
    out = [r[:4] for r in rows]
    cache.parent.mkdir(parents=True, exist_ok=True)
    pickle.dump(out, open(cache, "wb"))
    return out


def odom_traj(bagdir, topic):
    out = []
    for _, m in read_topic(bagdir, topic):
        p = m.pose.pose.position
        out.append((stamp_ns(m.header), p.x, p.y, ev.yaw_from_quat(m.pose.pose.orientation)))
    return out


def score_full(odom, gt, max_dt_ms=50.0):
    """ATE alineado + error de rumbo separado en sesgo constante y variacion (mocap_odometry.metrics.score_full).
    Claves historicas de este arnes: n, ate, drift, path, yaw_raw, yaw_bias, yaw_c, ate_p90, ate_max. None si
    hay < 20 pares."""
    from mocap_odometry import metrics
    s = metrics.score_full(ev.pair_by_time(odom, gt, int(max_dt_ms * 1e6)))
    if s is None:
        return None
    return dict(n=s["n"], ate=s["ate_rmse_m"], drift=s["drift_pct"], path=s["path_ref_m"],
                yaw_raw=s["yaw_rmse_deg"], yaw_bias=s["yaw_bias_deg"], yaw_c=s["yaw_c_rmse_deg"],
                ate_p90=s["ate_p90_m"], ate_max=s["ate_max_m"])


if __name__ == "__main__":
    # precalcula la cache del mocap de todos los bags
    from multiprocessing import Pool
    with Pool(8) as p:
        p.map(gt_traj, ALL_BAGS)
    print("cache del mocap:", len(list((WORK / "gtcache").glob("*.pkl"))), "bags")
