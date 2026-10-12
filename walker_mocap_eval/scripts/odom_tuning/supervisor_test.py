#!/usr/bin/env python3
"""Prueba odom_supervisor.py con fallos del IMU inyectados, con y sin supervisor, sobre bags limpios.

Escenarios: normal (el supervisor NO debe conmutar), IMU caido a los 25 s, signo del gyro invertido, sesgo +0.05 rad/s.
Para cada uno: rumbo centrado y ATE medios, y cuantas veces conmuto el supervisor.
Uso: python3 supervisor_test.py [--bags N]   (N bags limpios, por defecto 12; todos los escenarios x {sin, con})
"""
import argparse
import hashlib
import json
import re

import numpy as np

import harness as h
import tlib

SCENARIOS = {
    "normal":            {},
    "IMU caido a 25 s":  {"imu_dropout_start_s": 25.0},
    "signo invertido":   {"gyro_sign": -1.0},
    "sesgo +0.05 rad/s": {"gyro_bias_z": -0.05},
}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags", type=int, default=12)
    ap.add_argument("--rej", type=float, default=None,
                    help="umbral de rechazo (Mahalanobis) de odom0 CON supervisor; None = el de walker_ekf.yaml")
    ap.add_argument("--set", default="clean", help="clean | scan_anom | imu_anom ...: conjunto de bags")
    ap.add_argument("--sup", default="", help="parametros del supervisor, k=v separados por comas")
    ap.add_argument("--scenarios", default="", help="subconjunto (indices separados por coma, de 0)")
    a = ap.parse_args()
    pool = tlib.bag_set(a.set)
    bags = pool[:: max(1, len(pool) // a.bags)][: a.bags]
    sup_params = {k: float(v) for k, v in (kv.split('=') for kv in a.sup.split(',') if kv)}
    base = h.prod_cfg()
    laser = [h.laser_ref(b) for b in bags]
    print(f"{len(bags)} bags limpios | scan matcher solo: rumbo {np.mean([x['yaw_c'] for x in laser]):.2f} deg  "
          f"ATE {np.mean([x['ate'] for x in laser]):.3f} m\n")
    print(f"{'escenario':20s} | {'SIN supervisor':>22s} | {'CON supervisor':>22s} | conmutaciones (bags con >=1 / total)")
    names = list(SCENARIOS)
    if a.scenarios:
        names = [names[int(i)] for i in a.scenarios.split(",")]
    for name in names:
        kw = SCENARIOS[name]
        row = {}
        for sup in (False, True):
            cfg = h.mk(base, **kw)
            if sup:
                cfg["supervisor"] = True
                if sup_params:
                    cfg["supervisor_params"] = sup_params
                if a.rej is not None:
                    cfg["odom_rej"] = a.rej
            tag = "S_" + hashlib.md5(json.dumps(cfg, sort_keys=True).encode()).hexdigest()[:10]
            r = h.eval_config(cfg, tag, bags=bags, workers=6)
            s = h.summarize(r, bags)
            sw = 0
            if sup:
                for b in bags:
                    log = tlib.WORK / "runs" / tag / f"{b}.supervisor.log"
                    if log.exists() and re.search(r"DEGRADADO", log.read_text()):
                        sw += 1
            row[sup] = (s, sw)
        (s0, _), (s1, sw) = row[False], row[True]
        print(f"{name:20s} | rumbo {s0['yaw_c']:6.2f} ATE {s0['ate']:.3f} | rumbo {s1['yaw_c']:6.2f} ATE {s1['ate']:.3f} | "
              f"{sw}/{len(bags)}", flush=True)


if __name__ == "__main__":
    main()
