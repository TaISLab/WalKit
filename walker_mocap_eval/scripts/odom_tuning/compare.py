#!/usr/bin/env python3
"""Comparacion justa de configuraciones: cada una N veces sobre un conjunto de bags, con media y desviacion.
Es la unica evidencia que vale para decidir un cambio (ver el aviso de tune.py sobre el ruido).

  python3 compare.py                              # sin_imu y produccion sobre los bags limpios, 3 repeticiones
  python3 compare.py --bags scan_anom --reps 2    # donde el scan matching falla: rescata la IMU el rumbo?
  python3 compare.py --candidates cands.json      # {"nombre": {cambios sobre prod_cfg}}  p.ej.
                                                  #  {"gyro 4e-3": {"gyro_var": 0.004},
                                                  #   "yaw laser debil": {"pose_yaw": 0.2, "pn": {"yaw": 0.0005}}}
Claves de los cambios: ver harness.py (pose_yaw, pose_xy, pn, gyro_var, scan_stamp_offset, no_imu, ...).
"""
import argparse
import hashlib
import json
import time

import numpy as np

import harness as h
import tlib


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags", default="clean", choices=["clean", "scan_anom", "mocap_suspect", "all"])
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--candidates", help="JSON nombre -> cambios sobre prod_cfg()")
    ap.add_argument("--out", default=None, help="JSON de salida")
    a = ap.parse_args()

    bags = tlib.bag_set(a.bags)
    base = h.prod_cfg()
    cands = {"sin_imu": dict(base, no_imu=True), "produccion": base}
    if a.candidates:
        cands = {n: h.mk(base, **ch) for n, ch in json.load(open(a.candidates)).items()}

    out = {}
    for name, cfg in cands.items():
        Js, Ates, Yaws, per = [], [], [], {}
        for rep in range(a.reps):
            tag = "C_" + hashlib.md5((json.dumps(cfg, sort_keys=True) + str(rep)).encode()).hexdigest()[:10]
            r = h.eval_config(cfg, tag, bags=bags)
            s = h.summarize(r, bags)
            Js.append(s["J"])
            Ates.append(s["ate"])
            Yaws.append(s["yaw_c"])
            for b, m in r.items():
                if m:
                    per.setdefault(b, []).append((m["ate"], m["yaw_c"]))
        sd = lambda v: float(np.std(v, ddof=1)) if len(v) > 1 else 0.0           # noqa: E731
        print(f"{name:34s} J {np.mean(Js):.4f} +-{sd(Js):.4f} | ATE {np.mean(Ates):.4f} +-{sd(Ates):.4f} | "
              f"yaw_c {np.mean(Yaws):.3f} +-{sd(Yaws):.3f}   (n={s['n']})", flush=True)
        out[name] = dict(J=Js, ate=Ates, yaw_c=Yaws, cfg=cfg, per_bag={b: np.mean(v, axis=0).tolist() for b, v in per.items()})
    lr = [h.laser_ref(b) for b in bags if h.laser_ref(b)]
    print(f"{'laser solo (/scan_odom)':34s} J 1.0000 | ATE {np.mean([x['ate'] for x in lr]):.4f} | "
          f"yaw_c {np.mean([x['yaw_c'] for x in lr]):.3f}")
    if a.out:
        json.dump(out, open(a.out, "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
