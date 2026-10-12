"""Utilidades comunes de odom_tuning: rutas, conjuntos de bags, trayectoria de referencia del mocap (cacheada)
y metricas. Usa las mismas funciones que ../eval_odom_against_mocap.py.

Variables de entorno (todas opcionales):
  ODOM_TUNE_DIR  directorio de trabajo con grabaciones y caches (por defecto ~/odom_tuning; NO /tmp: las
                 grabaciones tardan ~25 min en rehacerse y /tmp se vacia al reiniciar)
  BAGS_DIR       carpeta con los rosbags con mocap (por defecto ~/bagsFolder_unified)
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
BAGS_DIR = Path(os.environ.get("BAGS_DIR", Path.home() / "bagsFolder_unified"))
ROS_SETUP = os.environ.get("ROS_SETUP", "/opt/ros/jazzy/setup.bash")
WS_SETUP = os.environ.get("WS_SETUP", str(HERE.parents[4] / "install" / "setup.bash"))
CALIB = load_calib(os.environ.get("MOCAP_CALIB", str(ev.DEFAULT_CALIB)))
BRINGUP_CONFIG = WALKIT / "walker_bringup" / "config"

ALL_BAGS = sorted(Path(p).parent.name for p in glob.glob(str(BAGS_DIR / "*" / "metadata.yaml")))

_Q = yaml.safe_load(open(HERE / "bag_quality.yaml"))
_WARP_FILE = HERE / "time_warp.yaml"
WARP = yaml.safe_load(open(_WARP_FILE)) if _WARP_FILE.exists() else {}
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


def warp_ns(t_ns, w):
    """Sello del mocap -> sello del robot (ver time_warp.yaml)."""
    t = t_ns * 1e-9
    return int(round((t + w["offset_s"] + w["slope"] * (t - w["t_ref"])) * 1e9))


def gt_traj(bag, apply_warp=True):
    """[(t_ns_cabecera, x, y, yaw)] de base_footprint en 'map' segun el mocap, con el eje temporal del mocap
    corregido al del robot si el bag figura en time_warp.yaml; cacheado en WORK/gtcache."""
    w = WARP.get(bag) if apply_warp else None
    cache = WORK / "gtcache" / (f"{bag}.warp.pkl" if w else f"{bag}.pkl")
    if cache.exists():
        return pickle.load(open(cache, "rb"))
    out = []
    for _, m in read_topic(BAGS_DIR / bag, "/rigid_bodies"):
        x, y, yaw = ev.rigid_body_pose_xy_yaw(m)
        t = warp_ns(stamp_ns(m.header), w) if w else stamp_ns(m.header)
        out.append((t,) + ev.mocap_to_base_footprint(x, y, yaw, CALIB))
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
    """ATE alineado + error de rumbo separado en sesgo constante (calibracion/montaje del cuerpo rigido) y
    variacion alrededor de ese sesgo (deriva real de la odometria). None si hay <20 pares."""
    pairs = ev.pair_by_time(odom, gt, int(max_dt_ms * 1e6))
    if len(pairs) < 20:
        return None
    src = [(p[1], p[2]) for p in pairs]
    dst = [(p[4], p[5]) for p in pairs]
    th, tx, ty = ev.align_2d(src, dst)
    pe, ye = [], []
    for t, ox, oy, oyaw, gx, gy, gyaw in pairs:
        ax, ay = ev.apply_2d(th, tx, ty, ox, oy)
        pe.append(math.hypot(ax - gx, ay - gy))
        ye.append(ev.wrap_angle(oyaw + th - gyaw))
    pe = np.array(pe)
    ye = np.degrees(np.array(ye))
    bias = float(np.degrees(math.atan2(np.mean(np.sin(np.radians(ye))), np.mean(np.cos(np.radians(ye))))))
    yc = np.degrees(np.array([ev.wrap_angle(math.radians(v - bias)) for v in ye]))
    path = ev.path_length([(p[4], p[5]) for p in pairs])
    rm = float(np.sqrt(np.mean(pe ** 2)))
    return dict(n=len(pairs), ate=rm, drift=100 * rm / path if path > 1e-6 else float("nan"), path=path,
                yaw_raw=float(np.sqrt(np.mean(ye ** 2))), yaw_bias=bias, yaw_c=float(np.sqrt(np.mean(yc ** 2))),
                ate_p90=float(np.percentile(pe, 90)), ate_max=float(pe.max()))


if __name__ == "__main__":
    # precalcula la cache del mocap de todos los bags
    from multiprocessing import Pool
    with Pool(8) as p:
        p.map(gt_traj, ALL_BAGS)
    print("cache del mocap:", len(list((WORK / "gtcache").glob("*.pkl"))), "bags")
