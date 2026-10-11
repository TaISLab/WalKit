#!/usr/bin/env python3
"""Comprueba el IMU contra el mocap en cada bag: signo/orientacion, escala, sesgo y desfase de reloj.

Para el gyro z crudo y el filtrado: correlacion con la velocidad de guinada del mocap, pendiente (escala),
ordenada (sesgo, rad/s), desfase de reloj (s) y desviacion del residuo. Ademas imprime la media de la aceleracion z:
con el robot parado debe ser +9.8 si el eje z del sensor apunta ARRIBA (entonces base_footprint->witmotion NO debe
llevar un giro de 180 deg; con el roll=180 antiguo del xacro robot_localization fusionaba la guinada con signo cambiado).

Hallazgo de 2026-10: el gyro crudo tiene sesgo 0.000 +-0.003 rad/s, escala 1.00, corr 0.99-1.00; el filtrado
(imu_complementary_filter con do_bias_estimation) tiene sesgos de hasta 0.13 rad/s distintos en cada bag.
Uso: python3 gyro_check.py [--out gyro.json]
"""
import argparse
import json
import math
from multiprocessing import Pool

import numpy as np

import tlib


def analyse(bag):
    try:
        src = tlib.BAGS_DIR / bag
        rb = tlib.read_topic(src, "/rigid_bodies")
        tm = np.array([tlib.stamp_ns(m.header) for _, m in rb]) / 1e9
        yaw = np.unwrap([tlib.ev.yaw_from_quat(m.rigidbodies[0].pose.orientation) for _, m in rb])
        k = 6
        rate_m = (yaw[2 * k:] - yaw[:-2 * k]) / (tm[2 * k:] - tm[:-2 * k])
        tmr = tm[k:-k]
        res = {"bag": bag}
        for topic, key in (("/bwt901cl/imu", "raw"), ("/bwt901cl/imu_filtered", "filt")):
            ms = [m for _, m in tlib.read_topic(src, topic)]
            if len(ms) < 100:
                continue
            ti = np.array([tlib.stamp_ns(m.header) for m in ms]) / 1e9
            gz = np.array([m.angular_velocity.z for m in ms])
            az = float(np.mean([m.linear_acceleration.z for m in ms]))
            w = max(1, int(round(0.1 / np.median(np.diff(ti)))))
            gz_s = np.convolve(gz, np.ones(w) / w, mode="same")
            best = None
            for lag in np.arange(-1.0, 1.0001, 0.02):
                rm = np.interp(ti + lag, tmr, rate_m, left=np.nan, right=np.nan)
                ok = np.isfinite(rm)
                if ok.sum() < 200:
                    continue
                c = np.corrcoef(gz_s[ok], rm[ok])[0, 1]
                if best is None or c > best[1]:
                    best = (lag, c)
            lag, c = best
            rm = np.interp(ti + lag, tmr, rate_m, left=np.nan, right=np.nan)
            ok = np.isfinite(rm)
            (a, b), *_ = np.linalg.lstsq(np.vstack([gz_s[ok], np.ones(ok.sum())]).T, rm[ok], rcond=None)
            res[key] = dict(lag_s=float(lag), corr=float(c), scale=float(a), offset_rad_s=float(b),
                            resid_std=float((rm[ok] - (a * gz_s[ok] + b)).std()), acc_z_mean=az,
                            cov_z=float(ms[0].angular_velocity_covariance[8]), hz=float(1 / np.median(np.diff(ti))))
        return res
    except Exception as e:
        return {"bag": bag, "error": repr(e)[:80]}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    with Pool(4) as p:
        R = p.map(analyse, tlib.ALL_BAGS)
    print(f"{'bag':10s} | {'RAW corr':>8s} {'scale':>6s} {'offset':>8s} {'resid':>6s} {'lag':>5s} {'acc z':>6s} | "
          f"{'FILT corr':>9s} {'scale':>6s} {'offset':>8s} {'resid':>6s}")
    for r in R:
        if "error" in r or "raw" not in r:
            print(f"{r['bag']:10s} | sin IMU / {r.get('error', '')}")
            continue
        a_, f = r["raw"], r.get("filt")
        line = (f"{r['bag']:10s} | {a_['corr']:8.2f} {a_['scale']:6.2f} {a_['offset_rad_s']:8.4f} {a_['resid_std']:6.3f} "
                f"{a_['lag_s']:5.2f} {a_['acc_z_mean']:6.2f} | ")
        line += (f"{f['corr']:9.2f} {f['scale']:6.2f} {f['offset_rad_s']:8.4f} {f['resid_std']:6.3f}" if f else "")
        print(line)
    if a.out:
        json.dump(R, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
