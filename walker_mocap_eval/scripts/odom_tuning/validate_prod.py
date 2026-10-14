#!/usr/bin/env python3
"""Valida lo que hay comiteado en walker_bringup/config de extremo a extremo con el launcher de test REAL
(walker_laser_imu_odom_test.launch.py), sin el arnes. Compara /odom (EKF) con /scan_odom (laser solo).
Uso: python3 validate_prod.py [BAG ...]   (por defecto un conjunto mixto de limpios y anomalos)"""
import os
import shutil
import signal
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

import yaml

import tlib

DEFAULT = ["CA_test01", "LA_test03", "MF_test04", "MT_test17", "LA_test07", "MF_test20", "CA_test03", "LA_test01"]
RATE = 1.5


def sh(cmd, dom, log):
    return subprocess.Popen(["bash", "-c", f"export ROS_DOMAIN_ID={dom}; source {tlib.ROS_SETUP}; "
                             f"source {tlib.WS_SETUP}; exec {cmd}"],
                            preexec_fn=os.setsid, stdout=open(log, "w"), stderr=subprocess.STDOUT)


def stop(p, t=10):
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


def run(i, bag):
    dom, bp, out = 170 + i, tlib.BAGS_DIR / bag, tlib.WORK / "prod" / bag
    if out.exists():
        shutil.rmtree(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    dur = yaml.safe_load(open(bp / "metadata.yaml"))["rosbag2_bagfile_information"]["duration"]["nanoseconds"] / 1e9
    rec = sh(f"ros2 bag record --use-sim-time -o {out} /odom /scan_odom", dom, out.parent / f"{bag}_rec.log")
    time.sleep(3)
    la = sh(f"ros2 launch walker_bringup walker_laser_imu_odom_test.launch.py bag_folder:={bp.parent} bag_name:={bag} rate:={RATE}",
            dom, out.parent / f"{bag}_launch.log")
    try:
        time.sleep(dur / RATE + 12)
    finally:
        stop(la)
        stop(rec)
        time.sleep(1)
    gt = tlib.gt_traj(bag)
    lo, eo = tlib.odom_traj(out, "/scan_odom"), tlib.odom_traj(out, "/odom")
    return bag, tlib.score_full(lo, gt), tlib.score_full(eo, gt), len(lo), len(eo)


def main():
    bags = sys.argv[1:] or DEFAULT
    with ThreadPoolExecutor(4) as ex:
        res = list(ex.map(lambda a: run(*a), enumerate(bags)))
    print(f"{'bag':10s} {'scans':>5s} {'/odom':>5s} | {'laser: ATE  yaw_c':>20s} | {'EKF: ATE  yaw_c':>20s}")
    for b, l, e, nl, ne in res:
        if l and e:
            print(f"{b:10s} {nl:5d} {ne:5d} | {l['ate']:8.3f} {l['yaw_c']:8.2f} | {e['ate']:8.3f} {e['yaw_c']:8.2f}")
        else:
            print(f"{b} n/a")


if __name__ == "__main__":
    main()
