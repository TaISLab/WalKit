#!/usr/bin/env python3
"""Etapa 1: por cada bag corre el test launcher real (laser_filter -> nav_laser_filter -> scan matcher + EKF) y graba
/scan_odom con reloj simulado en WORK/stage1/<bag>. El scan matching no depende de los parametros del EKF, asi que
solo hay que repetirlo si cambian los parametros del scan matcher (covarianza no: la sobrescribe el arnes).

Se corre con walker_lsm.yaml pero con odom_time_offset_ns=0 (el desfase temporal lo aplica el arnes, para poder
barrerlo sin repetir esto).

Uso:  python3 stage1.py [--workers 6] [--rate 2.0] [--bags clean|scan_anom|mocap_suspect|all]
                       [--lsm-set clave=valor ...]   p.ej. --lsm-set imu_topic=/bwt901cl/imu  (prior del gyro)
Para comparar con y sin un cambio del scan matcher, usar dos ODOM_TUNE_DIR distintos.
Tarda ~25 min para los 47 bags con 6 trabajos. Los bags ya grabados y completos se saltan.
"""
import argparse
import os
import shutil
import signal
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor

import yaml

import tlib

W = tlib.WORK


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


def complete(bag):
    """La grabacion tiene >=97 % de los scans del bag?"""
    try:
        s = yaml.safe_load(open(W / "stage1" / bag / "metadata.yaml"))["rosbag2_bagfile_information"]
        n = {t["topic_metadata"]["name"]: t["message_count"] for t in s["topics_with_message_count"]}
        o = yaml.safe_load(open(tlib.BAGS_DIR / bag / "metadata.yaml"))["rosbag2_bagfile_information"]
        ns = {t["topic_metadata"]["name"]: t["message_count"] for t in o["topics_with_message_count"]}
        return n.get("/scan_odom", 0) >= 0.97 * ns.get("/scan", 1)
    except Exception:
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--rate", default="2.0")
    ap.add_argument("--bags", default="all", choices=["clean", "scan_anom", "mocap_suspect", "all"])
    ap.add_argument("--domain-base", type=int, default=150)
    ap.add_argument("--lsm-set", action="append", default=[], metavar="CLAVE=VALOR",
                    help="parametro del scan matcher a fijar (repetible); el valor se interpreta como YAML")
    a = ap.parse_args()

    lsm = yaml.safe_load(open(tlib.BRINGUP_CONFIG / "walker_lsm.yaml"))
    lsm["laser_scan_matcher_node"]["ros__parameters"]["odom_time_offset_ns"] = 0
    for kv in a.lsm_set:
        k, v = kv.split("=", 1)
        lsm["laser_scan_matcher_node"]["ros__parameters"][k] = yaml.safe_load(v)
    W.mkdir(parents=True, exist_ok=True)
    lsm_file = W / "lsm_stage1.yaml"
    yaml.safe_dump(lsm, open(lsm_file, "w"))
    (W / "stage1" / "logs").mkdir(parents=True, exist_ok=True)

    bags = tlib.bag_set(a.bags)
    chunks = [bags[k::a.workers] for k in range(a.workers)]

    def run(k, bag):
        if complete(bag):
            return bag, "skip"
        out = W / "stage1" / bag
        if out.exists():
            shutil.rmtree(out)
        bp = tlib.BAGS_DIR / bag
        dur = yaml.safe_load(open(bp / "metadata.yaml"))["rosbag2_bagfile_information"]["duration"]["nanoseconds"] / 1e9
        dom = a.domain_base + k
        rec = sh(f"ros2 bag record --use-sim-time -o {out} /scan_odom /odom", dom, W / "stage1" / "logs" / f"{bag}_rec.log")
        time.sleep(3)
        la = sh(f"ros2 launch walker_bringup walker_laser_imu_odom_test.launch.py bag_path:={bp} rate:={a.rate} "
                f"lsm_params_file:={lsm_file}", dom, W / "stage1" / "logs" / f"{bag}_launch.log")
        try:
            time.sleep(dur / float(a.rate) + 12)
        finally:
            stop(la)
            stop(rec)
            time.sleep(1)
        return bag, "ok" if complete(bag) else "INCOMPLETO"

    def worker(k):
        for b in chunks[k]:
            print(run(k, b), flush=True)

    with ThreadPoolExecutor(a.workers) as ex:
        list(ex.map(worker, range(a.workers)))
    print("STAGE1 DONE", flush=True)


if __name__ == "__main__":
    main()
