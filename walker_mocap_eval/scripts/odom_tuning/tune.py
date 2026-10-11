#!/usr/bin/env python3
"""Busqueda por coordenadas de parametros del EKF y de las covarianzas del scan matcher, partiendo de la
configuracion comiteada (harness.prod_cfg()). Puntua sobre la mitad 'train' de los bags limpios y valida en la otra.

ADVERTENCIA (aprendida a la mala): la evaluacion tiene ruido (std ~0.007-0.01 de J entre repeticiones identicas, y
mas en configuraciones cerca de inestabilidad). Elegir el minimo de muchos candidatos medidos UNA vez selecciona la
suerte: un candidato "0.93" resulto ser ~1.05 al repetirlo. Por eso:
  - ACCEPT exige mejorar mas que el ruido para cambiar de configuracion;
  - lo que salga de aqui hay que confirmarlo con compare.py (varias repeticiones) antes de comitearlo.

Uso:  python3 tune.py [--grid] [--passes 2]
  --grid  antes de los barridos, rejilla conjunta (covarianza de rumbo del laser x ruido de proceso de yaw x
          varianza del gyro): los barridos de uno en uno no encuentran compromisos entre parametros acoplados.
"""
import argparse
import hashlib
import itertools
import json
import time

import harness as h
import tlib

ACCEPT = 0.012          # ~1.7 sigma del ruido de J medido
TRAIN, VAL = tlib.CLEAN[0::2], tlib.CLEAN[1::2]


def evaluate(name, cfg, bags, log):
    t0 = time.time()
    tag = "T_" + hashlib.md5(json.dumps(cfg, sort_keys=True).encode()).hexdigest()[:10]
    r = h.eval_config(cfg, tag, bags=bags)
    s = h.summarize(r, bags)
    line = (f"{name:44s} n={s['n']:2d} ATE {s['ate']:.4f} drift {s['drift']:.3f}% yaw_c {s['yaw_c']:.3f} | "
            f"x laser: ATE {s['ate_ratio']:.3f} yaw {s['yaw_ratio']:.3f} J={s['J']:.4f} ({time.time() - t0:.0f}s)")
    print(line, flush=True)
    log.write(line + "\n")
    log.flush()
    return s, r


def sweep(cfg, best_j, label, variants, log):
    for nm, kw in variants:
        c = h.mk(cfg, **kw)
        s, _ = evaluate(f"{label}={nm}", c, TRAIN, log)
        if s["J"] < best_j - ACCEPT:
            best_j, cfg = s["J"], c
    print(f"  -> mejor {label}: J={best_j:.4f}", flush=True)
    return cfg, best_j


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grid", action="store_true")
    ap.add_argument("--passes", type=int, default=2)
    a = ap.parse_args()

    base = h.prod_cfg()
    odom_vxy = [bool(v) for v in __import__("yaml").safe_load(open(h.PROD_EKF))["walker_ekf_node"]
                ["ros__parameters"]["odom0_config"]]
    odom_novel = list(odom_vxy)
    odom_novel[6] = odom_novel[7] = False
    log = open(tlib.WORK / "tune_log.txt", "a")

    s, _ = evaluate("ref sin_imu", dict(base, no_imu=True), TRAIN, log)
    s, _ = evaluate("ref produccion", base, TRAIN, log)
    cfg, j = base, s["J"]
    if a.grid:
        for ry, qy, gv in itertools.product((0.0012, 0.01, 0.05), (0.001, 0.01, 0.06), (4e-4, 4e-3)):
            c = h.mk(base, pose_yaw=ry, gyro_var=gv, pn={"yaw": qy})
            s, _ = evaluate(f"grid Rl_yaw={ry} Qyaw={qy} Rgyro={gv}", c, TRAIN, log)
            if s["J"] < j - ACCEPT:
                j, cfg = s["J"], c
    for p in range(a.passes):
        print(f"=== pasada {p + 1} ===", flush=True)
        cfg, j = sweep(cfg, j, "gyro_var", [(str(v), dict(gyro_var=v)) for v in (1e-4, 4e-4, 1.6e-3, 6.4e-3, 0.0378)], log)
        cfg, j = sweep(cfg, j, "pose_yaw", [(str(v), dict(pose_yaw=v)) for v in (0.0012, 0.01, 0.05, 0.2)], log)
        cfg, j = sweep(cfg, j, "pose_xy", [(str(v), dict(pose_xy=v)) for v in (0.0006, 0.0025, 0.01)], log)
        cfg, j = sweep(cfg, j, "pn_yaw", [(str(v), dict(pn={"yaw": v})) for v in (0.0005, 0.001, 0.01, 0.06)], log)
        cfg, j = sweep(cfg, j, "pn_vyaw", [(str(v), dict(pn={"vyaw": v})) for v in (0.002, 0.02, 0.2)], log)
        cfg, j = sweep(cfg, j, "pn_xy", [(str(v), dict(pn={"x": v, "y": v})) for v in (0.01, 0.05, 0.2)], log)
        cfg, j = sweep(cfg, j, "odom_vel", [("sin_vx_vy", dict(odom_config=odom_novel)),
                                           ("con_vx_vy", dict(odom_config=odom_vxy))], log)
        cfg, j = sweep(cfg, j, "scan_stamp_offset", [(str(v), dict(scan_stamp_offset=v)) for v in (0.0, 0.04, 0.08, 0.12)], log)
        cfg, j = sweep(cfg, j, "imu_rej", [(str(v), dict(imu_rej=v)) for v in (3.0, 5.0, 1e6)], log)
        cfg, j = sweep(cfg, j, "odom_rej", [(str(v), dict(odom_rej=v)) for v in (3.0, 5.0, 1e6)], log)
        json.dump(cfg, open(tlib.WORK / f"tune_best_pass{p + 1}.json", "w"), indent=1)
    sv, _ = evaluate("FINAL val", cfg, VAL, log)
    print("TUNE DONE -> confirma el resultado con compare.py antes de usarlo", flush=True)


if __name__ == "__main__":
    main()
