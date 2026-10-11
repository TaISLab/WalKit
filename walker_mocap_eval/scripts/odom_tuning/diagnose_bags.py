#!/usr/bin/env python3
"""Arbitra, con el gyro crudo, quien se equivoca en cada bag -- laser (scan matching), mocap o ninguno -- y lo
compara con las etiquetas de bag_quality.yaml.

Para cada bag compara tres fuentes de velocidad de guinada por intervalo de scan: laser (/scan_odom, etapa 1),
gyro crudo (etapa 2) y mocap (rigid_bodies[0]):
  - el laser discrepa de AMBOS (gyro y mocap coinciden)  -> scan_matching_anomalous
  - el mocap discrepa de AMBOS (gyro y laser coinciden)  -> mocap_suspect
Requiere stage1.py y build_stage2.py ejecutados. Uso: python3 diagnose_bags.py [--out diagnose.json]
"""
import argparse
import json
from multiprocessing import Pool

import numpy as np
import yaml

import tlib

W = tlib.WORK
Q = yaml.safe_load(open(tlib.HERE / "bag_quality.yaml"))


def rates(bag):
    s1 = W / "stage1" / bag
    lo = np.array(tlib.odom_traj(s1, "/scan_odom"))
    t = lo[:, 0] / 1e9
    yl = np.unwrap(lo[:, 3])
    g = np.array(tlib.gt_traj(bag))
    tg, ym_full = g[:, 0] / 1e9, np.unwrap(g[:, 3])
    keep = np.concatenate([[True], np.diff(tg) > 0])          # muestras del mocap con sello duplicado
    tg, ym_full = tg[keep], ym_full[keep]
    imu = tlib.read_topic(W / "stage2" / bag, "/bwt901cl/imu")
    if len(imu) < 100:
        return dict(bag=bag, error="sin IMU")
    ti = np.array([tlib.stamp_ns(m.header) for _, m in imu]) / 1e9
    gz = np.array([m.angular_velocity.z for _, m in imu])
    cum = np.concatenate([[0], np.cumsum(0.5 * (gz[1:] + gz[:-1]) * np.diff(ti))])      # yaw integrado del gyro
    lo_t, hi_t = max(t[0], tg[0], ti[0]), min(t[-1], tg[-1], ti[-1])
    m = (t >= lo_t) & (t <= hi_t)
    t, yl = t[m], yl[m]
    ym, yg = np.interp(t, tg, ym_full), np.interp(t, ti, cum)
    dt = np.diff(t)
    good = (dt > 0.05) & (dt < 0.5)
    rl, rm, rg = (np.diff(y)[good] / dt[good] for y in (yl, ym, yg))

    def stat(a, b):
        return float(np.degrees(np.sqrt(np.mean((a - b) ** 2)))), float(np.corrcoef(a, b)[0, 1])

    def cumdiff(a, b):
        return float(np.degrees(np.sqrt(np.mean(((a - a[0]) - (b - b[0])) ** 2))))

    d = dict(bag=bag)
    d["lm_rms"], d["lm_corr"] = stat(rl, rm)
    d["gm_rms"], d["gm_corr"] = stat(rg, rm)
    d["lg_rms"], d["lg_corr"] = stat(rl, rg)
    d["cum_lm"], d["cum_gm"], d["cum_lg"] = cumdiff(yl, ym), cumdiff(yg, ym), cumdiff(yl, yg)
    f = tlib.score_full(tlib.odom_traj(s1, "/scan_odom"), tlib.gt_traj(bag))
    d.update(drift=f["drift"], ate=f["ate"], yaw_bias=f["yaw_bias"], yaw_c=f["yaw_c"])
    d["suggest"] = suggest(d)
    return d


def suggest(d):
    if d["gm_corr"] < 0.85 and d["lg_corr"] > 0.88:
        return "mocap_suspect"
    if (d["cum_lm"] > 10 and d["cum_gm"] < 0.55 * d["cum_lm"]) or (d["lm_rms"] > 2 * d["gm_rms"] and d["lm_corr"] < 0.85):
        return "scan_matching_anomalous"
    return "ok"


def run(bag):
    try:
        return rates(bag)
    except Exception as e:
        return dict(bag=bag, error=repr(e)[:80])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    with Pool(8) as p:
        R = p.map(run, tlib.ALL_BAGS)
    labelled = {b: k for k in ("scan_matching_anomalous", "mocap_suspect", "no_imu") for b in Q[k]}
    print(f"{'bag':10s} {'drift%':>6s} {'yaw_c':>6s} {'bias':>6s} | {'l-m rms/corr':>13s} {'g-m rms/corr':>13s} "
          f"{'l-g rms/corr':>13s} | {'cum l-m':>7s} {'cum g-m':>7s} | {'sugerido':>24s} {'etiqueta actual':>24s}")
    for r in R:
        if "error" in r:
            print(f"{r['bag']:10s} ERROR {r['error']}")
            continue
        print(f"{r['bag']:10s} {r['drift']:6.2f} {r['yaw_c']:6.1f} {r['yaw_bias']:6.1f} | "
              f"{r['lm_rms']:6.1f}/{r['lm_corr']:5.2f} {r['gm_rms']:6.1f}/{r['gm_corr']:5.2f} "
              f"{r['lg_rms']:6.1f}/{r['lg_corr']:5.2f} | {r['cum_lm']:7.1f} {r['cum_gm']:7.1f} | "
              f"{r['suggest']:>24s} {labelled.get(r['bag'], 'ok'):>24s}")
    if a.out:
        json.dump(R, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
