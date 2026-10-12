"""Arnes de ajuste: repite SOLO el EKF sobre grabaciones de la etapa 2 (no repite el scan matching).

Por que asi: la salida del scan matcher no depende de los parametros del EKF, asi que se graba una vez por bag
(stage1.py) y despues cada configuracion del EKF cuesta ~1 min para 16 bags en vez de ~25.

Cada configuracion es un dict `cfg`; claves reconocidas (todas opcionales):
  pn {estado: varianza}   ruido de proceso (diagonal) del EKF        p0 {estado: varianza}  covarianza inicial
  imu_config / odom_config   listas de 15 booleanos                  odom_rej / imu_rej     umbrales de rechazo
  freq                       frecuencia del EKF (Hz simulados)       no_imu                 sin imu0
  imu_topic                  IMU a fusionar (/bwt901cl/imu cruda)    gyro_var / gyro_bias_z / gyro_sign
  pose_cov / twist_cov       diagonal (6) del /scan_odom             scan_stamp_offset      s sumados al sello
  imu_roll_deg               roll del TF base_footprint->witmotion (0: eje z del sensor hacia arriba)
  extra                      parametros del ekf_node a sobrescribir tal cual
  supervisor / supervisor_params   intercala walker_bringup/scripts/odom_supervisor.py entre el rele y el EKF
  imu_dropout_start_s / imu_dropout_len_s   fallo inyectado: el IMU deja de publicar
`prod_cfg()` es la configuracion equivalente a lo que hay comiteado en walker_bringup/config.
"""
import copy
import os
import signal
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import yaml

import tlib

WORK = tlib.WORK
PROD_EKF = tlib.BRINGUP_CONFIG / "walker_ekf.yaml"
CLOCK_HZ = 1000     # /clock a 40 Hz (defecto) hace que el temporizador del EKF caiga a ~14-19 Hz simulados y
                    # que el resultado dependa de la velocidad de reproduccion; a 1000 Hz no
STATE = ["x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "vz", "vroll", "vpitch", "vyaw", "ax", "ay", "az"]
CLEAN, SCAN_ANOM, MOCAP_SUSPECT = tlib.CLEAN, tlib.SCAN_ANOM, tlib.MOCAP_SUSPECT


def prod_cfg():
    """Configuracion del arnes equivalente a walker_ekf.yaml + walker_lsm.yaml + wt901.yaml del repo."""
    ekf = yaml.safe_load(open(PROD_EKF))["walker_ekf_node"]["ros__parameters"]
    lsm = yaml.safe_load(open(tlib.BRINGUP_CONFIG / "walker_lsm.yaml"))["laser_scan_matcher_node"]["ros__parameters"]
    wt = yaml.safe_load(open(tlib.BRINGUP_CONFIG / "wt901.yaml"))["witmotion"]["ros__parameters"]
    gyro_cov = wt["imu_publisher"]["measurements"]["angular_velocity"]["covariance"]
    return dict(imu_topic=ekf["imu0"], imu_roll_deg=0.0, gyro_var=float(gyro_cov[8]),
                pose_cov=list(lsm["pose_covariance_diagonal"]), twist_cov=list(lsm["twist_covariance_diagonal"]),
                scan_stamp_offset=float(lsm.get("odom_time_offset_ns", 0)) * 1e-9)


def mk(base, **kw):
    """Copia de base con cambios; atajos: pose_yaw, pose_xy, twist_v, twist_w; pn se fusiona."""
    c = copy.deepcopy(base)
    for k, v in kw.items():
        if k == "pn":
            c["pn"] = {**c.get("pn", {}), **v}
        elif k == "pose_yaw":
            c["pose_cov"][5] = v
        elif k == "pose_xy":
            c["pose_cov"][0] = c["pose_cov"][1] = v
        elif k == "twist_v":
            c["twist_cov"][0] = c["twist_cov"][1] = v
        elif k == "twist_w":
            c["twist_cov"][5] = v
        else:
            c[k] = v
    return c


def sh(cmd, dom, log=None):
    full = f"export ROS_DOMAIN_ID={dom}; source {tlib.ROS_SETUP}; source {tlib.WS_SETUP}; exec {cmd}"
    return subprocess.Popen(["bash", "-c", full], preexec_fn=os.setsid,
                            stdout=open(log, "w") if log else subprocess.DEVNULL, stderr=subprocess.STDOUT)


def stop(p, t=6):
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
        p.wait(timeout=t)
    except Exception:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def make_ekf_yaml(cfg, path):
    d = yaml.safe_load(open(PROD_EKF))
    p = d["walker_ekf_node"]["ros__parameters"]
    for key, name in (("pn", "process_noise_covariance"), ("p0", "initial_estimate_covariance")):
        if key in cfg:
            m = np.array(p[name], dtype=float).reshape(15, 15)
            for k, v in cfg[key].items():
                i = STATE.index(k)
                m[i, i] = v
            p[name] = m.flatten().tolist()
    if "imu_config" in cfg:
        p["imu0_config"] = cfg["imu_config"]
    if "odom_config" in cfg:
        p["odom0_config"] = cfg["odom_config"]
    if "odom_rej" in cfg:
        p["odom0_pose_rejection_threshold"] = p["odom0_twist_rejection_threshold"] = cfg["odom_rej"]
    if "imu_rej" in cfg:
        for k in ("imu0_pose_rejection_threshold", "imu0_twist_rejection_threshold",
                  "imu0_linear_acceleration_rejection_threshold"):
            p[k] = cfg["imu_rej"]
    if "freq" in cfg:
        p["frequency"] = float(cfg["freq"])
    for k in ("odom0_relative", "odom0_differential", "imu0_relative", "imu0_differential"):
        if k in cfg:
            p[k] = cfg[k]
    p.update(cfg.get("extra", {}))
    p["imu0"] = "/imu_tuned"        # la salida del rele de tune_node.py
    if cfg.get("supervisor"):       # el EKF se alimenta de las salidas de odom_supervisor.py
        p["odom0"] = "/scan_odom_ekf"
        p["imu0"] = "/imu_ekf"
        if "odom_rej" not in cfg:   # igual que walker_laser_imu_odom.launch.py con supervise_imu:=true
            p["odom0_pose_rejection_threshold"] = p["odom0_twist_rejection_threshold"] = 100.0
    if cfg.get("no_imu"):
        p.pop("imu0")
    p["debug"] = False
    yaml.safe_dump(d, open(path, "w"))


def run_bag(bag, cfg, tag, dom, rate=10.0):
    """Reproduce la etapa 2 de un bag a traves del EKF con cfg; devuelve el CSV de /odom."""
    out = WORK / "runs" / tag
    out.mkdir(parents=True, exist_ok=True)
    csv, yml = out / f"{bag}.csv", out / "ekf.yaml"
    if not yml.exists():
        make_ekf_yaml(cfg, yml)
    procs = []
    try:
        na = [f"-p out:={csv}", "-p use_sim_time:=true"]
        if "pose_cov" in cfg:
            na += ["-p override_scan_cov:=true",
                   f"-p pose_cov:=[{','.join(map(str, cfg['pose_cov']))}]",
                   f"-p twist_cov:=[{','.join(map(str, cfg['twist_cov']))}]"]
        for k in ("scan_stamp_offset", "gyro_var", "gyro_bias_z", "gyro_sign", "imu_topic",
                  "imu_dropout_start_s", "imu_dropout_len_s"):
            if k in cfg:
                na.append(f"-p {k}:={cfg[k]}")
        procs.append(sh(f"python3 {tlib.HERE}/tune_node.py --ros-args {' '.join(na)}", dom))
        roll = np.radians(cfg.get("imu_roll_deg", 0.0))
        procs.append(sh("ros2 run tf2_ros static_transform_publisher --x 0.565 --y 0 --z 0.305 "
                        f"--roll {roll} --pitch 0 --yaw 0 --frame-id base_footprint --child-frame-id witmotion", dom))
        procs.append(sh("ros2 run robot_localization ekf_node --ros-args -r __node:=walker_ekf_node "
                        f"-r /odometry/filtered:=/odom -p use_sim_time:=true --params-file {yml}", dom))
        if cfg.get("supervisor"):
            sp = [f"-p {k}:={v}" for k, v in cfg.get("supervisor_params", {}).items()]
            procs.append(sh("python3 " + str(tlib.WALKIT / "walker_bringup" / "scripts" / "odom_supervisor.py")
                            + " --ros-args -p use_sim_time:=true -p scan_in:=/scan_odom -p imu_in:=/imu_tuned "
                            + " ".join(sp), dom, out / f"{bag}.supervisor.log"))
        time.sleep(3.0)
        play = sh(f"ros2 bag play {WORK}/stage2/{bag} --clock {CLOCK_HZ} -r {rate} "
                  "--topics /scan_odom /bwt901cl/imu /bwt901cl/imu_filtered --remap /scan_odom:=/scan_odom_raw", dom)
        play.wait(timeout=900)
        time.sleep(1.5)
    finally:
        if procs:
            stop(procs[0])      # tune_node primero: vuelca el CSV
        for p in procs[1:]:
            stop(p)
    return csv


_LASER = {}


def laser_ref(bag):
    """Metricas del scan matcher solo (/scan_odom de la etapa 1), la referencia de cada bag."""
    if bag not in _LASER:
        _LASER[bag] = tlib.score_full(tlib.odom_traj(WORK / "stage1" / bag, "/scan_odom"), tlib.gt_traj(bag))
    return _LASER[bag]


def eval_config(cfg, tag, bags=None, workers=8, rate=10.0, domain_base=130):
    """cfg sobre bags en paralelo (un dominio DDS por hilo) -> {bag: metricas}."""
    bags = bags or CLEAN
    res = {}
    chunks = [bags[k::workers] for k in range(workers)]

    def worker(k):
        out = []
        for b in chunks[k]:
            c = run_bag(b, cfg, tag, domain_base + k, rate=rate)
            m = None
            if c.exists():
                odom = [(int(l.split(",")[0]), *map(float, l.split(",")[1:])) for l in open(c)]
                m = tlib.score_full(odom, tlib.gt_traj(b)) if len(odom) > 20 else None
            out.append((b, m))
        return out

    with ThreadPoolExecutor(workers) as ex:
        for out in ex.map(worker, range(workers)):
            for b, m in out:
                res[b] = m
    return res


def summarize(res, bags=None):
    """Medias y J = media(ATE/ATE_laser, rumbo/rumbo_laser): <1 es mejor que el scan matcher solo."""
    bags = [b for b in (bags or CLEAN) if res.get(b)]
    rows = [(res[b], laser_ref(b)) for b in bags]
    r_ate = np.mean([m["ate"] / l["ate"] for m, l in rows])
    r_yaw = np.mean([m["yaw_c"] / max(l["yaw_c"], 0.3) for m, l in rows])
    return dict(n=len(rows), ate=float(np.mean([m["ate"] for m, l in rows])),
                drift=float(np.mean([m["drift"] for m, l in rows])),
                yaw_c=float(np.mean([m["yaw_c"] for m, l in rows])), ate_ratio=float(r_ate), yaw_ratio=float(r_yaw),
                J=float((r_ate + r_yaw) / 2),
                laser_ate=float(np.mean([l["ate"] for m, l in rows])),
                laser_yaw_c=float(np.mean([l["yaw_c"] for m, l in rows])),
                laser_drift=float(np.mean([l["drift"] for m, l in rows])))
