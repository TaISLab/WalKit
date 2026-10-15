#!/usr/bin/env python3
"""Ajusta, por bag, la correccion del eje temporal del mocap que maximiza su coherencia con el gyro crudo.

Que mide y que NO. Dentro de un bag la guinada del mocap (/rigid_bodies) y la del gyro (/bwt901cl/imu) describen el
mismo movimiento fisico; sus sellos de cabecera estan en el mismo eje de tiempo del grabador (header - tiempo de
grabacion ~6 ms en el mocap, deriva < 1e-3 s/s en IMU y scan, y frame_number avanza a 120.000 fps). Aun asi, en algunos
bags la correlacion de la velocidad de guinada sube de 0.5-0.8 a 0.99+ si se desplaza y escala linealmente el eje del
mocap hasta 7 %. Este script mide ese desajuste EMPIRICO (offset + deriva lineal); no dice de que reloj ni de que
proceso viene, y no hay que interpretarlo como "dos PCs desincronizados".

Metodo:
  1. velocidad de guinada del mocap (diferencia centrada de +-6 muestras, ~0.1 s) y gyro z suavizado 0.1 s;
  2. ventanas de --win s: retardo (-max-lag..+max-lag) que maximiza la correlacion en cada una; se guardan las que
     tienen giro suficiente y correlacion >= --min-win-corr;
  3. recta retardo(t) = a + b*(t - t_ref) por ajuste robusto (Theil-Sen) a esos puntos;
  4. refinado local de (a, b) maximizando la correlacion global;
  5. se escribe solo si la correlacion mejora mas de --min-gain (por defecto 0.01) y el ajuste es creible.

Convencion (la que lee tlib.warp_ns): t_robot = t_mocap + offset_s + slope * (t_mocap - t_ref), con t_ref el primer
sello del mocap. corr_before = correlacion sin corregir (retardo 0); corr_const_lag, la mejor con un retardo constante de +-1 s.

Uso:
  python3 fit_time_warp.py                       # todos los bags; tabla por pantalla, no escribe nada
  python3 fit_time_warp.py --bags CA_test05 MT_test14
  python3 fit_time_warp.py --write               # reescribe time_warp.yaml (conserva su cabecera de comentarios)
  python3 fit_time_warp.py --json fit_all.json   # resultado completo de todos los bags
Tras cambiar time_warp.yaml borrar $ODOM_TUNE_DIR/gtcache/<bag>.warp.pkl (tlib cachea la verdad de campo).
"""
import argparse
import json
from multiprocessing import Pool

import numpy as np
import yaml

import tlib


def series(bag):
    """(t_mocap, vel. de guinada mocap, t_imu, gyro z suavizado, t_ref) o None si no hay IMU/mocap."""
    src = tlib.BAGS_DIR / bag
    rb = tlib.read_topic(src, "/rigid_bodies")
    imu = tlib.read_topic(src, "/bwt901cl/imu")
    if len(rb) < 200 or len(imu) < 200:
        return None
    tm = np.array([tlib.stamp_ns(m.header) for _, m in rb]) / 1e9
    yaw = np.unwrap([tlib.ev.yaw_from_quat(m.rigidbodies[0].pose.orientation) for _, m in rb])
    k = 6
    dt = tm[2 * k:] - tm[:-2 * k]
    ok = dt > 0.05
    rate = np.where(ok, (yaw[2 * k:] - yaw[:-2 * k]) / np.where(ok, dt, 1.0), np.nan)
    tmr = tm[k:-k]
    good = np.isfinite(rate) & (np.abs(rate) < 6.0)       # descarta saltos de orientacion del rigid body
    ti = np.array([tlib.stamp_ns(m.header) for _, m in imu]) / 1e9
    gz = np.array([m.angular_velocity.z for _, m in imu])
    w = max(1, int(round(0.1 / np.median(np.diff(ti)))))
    gz = np.convolve(gz, np.ones(w) / w, mode="same")
    return tmr[good], rate[good], ti, gz, float(tm[0])


def corr_under(tm, rate, ti, gz, offset, slope, t_ref, step=0.02):
    """Correlacion de la guinada del mocap con el gyro tras t_robot = t_mocap + offset + slope*(t_mocap - t_ref)."""
    tr = tm + offset + slope * (tm - t_ref)
    t = np.arange(max(tr[0], ti[0]), min(tr[-1], ti[-1]), step)
    if len(t) < 200:
        return -1.0
    m = np.interp(t, tr, rate, left=np.nan, right=np.nan)
    g = np.interp(t, ti, gz)
    ok = np.isfinite(m)
    return float(np.corrcoef(m[ok], g[ok])[0, 1]) if ok.sum() > 200 else -1.0


def theil_sen(x, y):
    i, j = np.triu_indices(len(x), 1)
    dx = x[j] - x[i]
    ok = np.abs(dx) > 1e-6
    b = float(np.median((y[j] - y[i])[ok] / dx[ok]))
    return float(np.median(y - b * x)), b


def fit_bag(bag, win=15.0, max_lag=8.0, min_win_corr=0.7, min_gain=0.01):
    try:
        s = series(bag)
        if s is None:
            return {"bag": bag, "error": "sin IMU o mocap"}
        tm, rate, ti, gz, t_ref = s
        # corr_before: correlacion SIN corregir; const_lag: mejor retardo constante dentro de +-1 s (gyro_check.py)
        c_zero = corr_under(tm, rate, ti, gz, 0.0, 0.0, t_ref)
        best0 = max((corr_under(tm, rate, ti, gz, o, 0.0, t_ref), o) for o in np.arange(-1.0, 1.0001, 0.02))
        # retardo por ventanas
        pts = []
        t0, t1 = tm[0], tm[-1]
        for a in np.arange(t0, t1 - win, win / 2):
            sel = (tm >= a) & (tm < a + win)
            if sel.sum() < 100 or np.std(rate[sel]) < 0.05:          # casi sin giro: no informa
                continue
            c_best, o_best = max((corr_under(tm[sel], rate[sel], ti, gz, o, 0.0, t_ref, 0.03), o)
                                 for o in np.arange(-max_lag, max_lag + 1e-9, 0.1))
            if c_best >= min_win_corr:
                pts.append((a + win / 2 - t_ref, o_best))
        if len(pts) < 4:
            return {"bag": bag, "corr_before": round(c_zero, 4), "error": f"solo {len(pts)} ventanas con giro y corr >= {min_win_corr}"}
        x, y = np.array(pts).T
        a0, b0 = theil_sen(x, y)
        # refinado local
        best = (corr_under(tm, rate, ti, gz, a0, b0, t_ref), a0, b0)
        for da, db, n in ((0.5, 0.004, 11), (0.1, 0.0008, 11), (0.02, 0.0002, 9), (0.004, 0.00004, 9)):
            c, o, sl = best
            for oo in o + np.linspace(-da, da, n):
                for ss in sl + np.linspace(-db, db, n):
                    cc = corr_under(tm, rate, ti, gz, oo, ss, t_ref)
                    if cc > best[0]:
                        best = (cc, oo, ss)
        c, o, sl = best
        return {"bag": bag, "corr_before": round(c_zero, 4), "corr_after": round(c, 4),
                "offset_s": round(o, 2), "slope": round(sl, 4), "t_ref": round(t_ref, 3),
                "windows": len(pts), "const_lag_s": round(best0[1], 2),
                "corr_const_lag": round(best0[0], 4),
                "apply": bool(c - c_zero > min_gain and abs(sl) < 0.15 and c > 0.9)}
    except Exception as e:      # un bag roto no tumba el resto
        return {"bag": bag, "error": repr(e)[:100]}


def _fit(args):
    return fit_bag(*args)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags", nargs="*", help="nombres de bag (por defecto todos)")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--win", type=float, default=15.0)
    ap.add_argument("--max-lag", type=float, default=8.0)
    ap.add_argument("--min-win-corr", type=float, default=0.7)
    ap.add_argument("--min-gain", type=float, default=0.01, help="mejora minima de correlacion para aplicar la correccion")
    ap.add_argument("--write", action="store_true", help="reescribe time_warp.yaml (solo bags con apply=True)")
    ap.add_argument("--json", help="guarda el resultado completo de todos los bags")
    a = ap.parse_args()
    bags = a.bags or tlib.ALL_BAGS
    with Pool(a.workers) as p:
        res = p.map(_fit, [(b, a.win, a.max_lag, a.min_win_corr, a.min_gain) for b in bags])
    print(f"{'bag':10s} {'corr antes':>10s} {'despues':>8s} {'offset_s':>9s} {'slope':>8s} {'vent.':>5s}  aplica")
    for r in res:
        if "error" in r:
            print(f"{r['bag']:10s} {r.get('corr_before', float('nan')):10.3f}  -- {r['error']}")
            continue
        print(f"{r['bag']:10s} {r['corr_before']:10.3f} {r['corr_after']:8.3f} {r['offset_s']:9.2f} {r['slope']:8.4f} "
              f"{r['windows']:5d}  {'SI' if r['apply'] else '-'}")
    if a.json:
        json.dump({r["bag"]: r for r in res}, open(a.json, "w"), indent=1)
    if a.write:
        from mocap_odometry import core
        path = core.writable_config_file(core.default_time_warp_file())
        header = "".join(l for l in open(path) if l.startswith("#") or not l.strip()) if path.exists() else ""
        keep = {r["bag"]: {k: r[k] for k in ("corr_before", "corr_after", "offset_s", "slope", "t_ref")}
                for r in res if r.get("apply")}
        with open(path, "w") as f:
            f.write(header.rstrip() + "\n" if header else "")
            yaml.safe_dump(keep, f)
        print(f"escrito {path} ({len(keep)} bags)")


if __name__ == "__main__":
    main()
