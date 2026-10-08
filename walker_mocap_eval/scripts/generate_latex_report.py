#!/usr/bin/env python3
"""Genera el informe LaTeX completo (11 puntos + puntos 1/2 de base) a partir
de /tmp/report_master.json (ver build_master_report_data.py). No hace ningun
calculo nuevo: solo formatea lo que ya existe en el master JSON en tablas
longtable (una fila por bag + filas de agregado final AGRUPADAS POR SUJETO
-- CA/LA/MF/MT, un sujeto por prefijo de bag, varias repeticiones de test
cada uno -- en vez de una unica media global que mezclaria sujetos de
complexion/sexo distintos) y texto de metodologia/formulas/limitaciones ya
validado durante el analisis.

Uso:
    python3 generate_latex_report.py --master /tmp/report_master.json \\
        --out /tmp/informe/informe.tex
"""
import argparse
import json
import math
import statistics
from pathlib import Path


def esc(s):
    return str(s).replace("_", "\\_")


def fmt(v, nd=3, na="--"):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return na
    if isinstance(v, str):
        return esc(v)
    return f"{v:.{nd}f}"


def mean_std_of(values):
    vals = [v for v in values if v is not None and not (isinstance(v, float) and math.isnan(v))]
    if not vals:
        return float("nan"), float("nan")
    if len(vals) == 1:
        return vals[0], float("nan")
    return statistics.mean(vals), statistics.stdev(vals)


def user_of(bag):
    """El prefijo antes de '_test' identifica al sujeto (CA/LA/MF/MT); cada
    uno aporta entre 7 y 20 repeticiones de test (bags)."""
    return bag.split("_")[0]


def group_bags(bags):
    groups = {}
    for b in bags:
        groups.setdefault(user_of(b), []).append(b)
    return dict(sorted(groups.items()))


def build_gait_error_summary(master):
    agg = master.get("_aggregate_comparison", {}).get("gait", {})
    names = [("left_spt", "SpT izq. (s)", 3), ("right_spt", "SpT dcha. (s)", 3),
             ("left_sdt", "SdT izq. (s)", 3), ("right_sdt", "SdT dcha. (s)", 3),
             ("left_spl", "SpL izq. (m)", 3), ("right_spl", "SpL dcha. (m)", 3),
             ("left_sdl", "SdL izq. (m)", 3), ("right_sdl", "SdL dcha. (m)", 3),
             ("nos", "NoS", 1), ("d_m", "$d$ (m)$^*$", 3), ("cad", "CAD (pasos/min)", 1),
             ("wv", "$WV$ (m/s)$^*$", 3)]
    rows = []
    for key, label, nd in names:
        v = agg.get(key)
        if not v:
            continue
        rows.append([label, fmt(v["mae"], nd), fmt(v["medae"], nd), f"{v['bias']:+.{nd}f}", fmt(v["rmse"], nd),
                     fmt(v["mape_pct"], 1), fmt(v["medape_pct"], 1)])
    return _small_table(
        "Punto 2: error del andador frente a mocap, agregado sobre los 47 bags (solo filas con $t\\le$ fin de "
        "la ventana \\texttt{/tag}; ejecuci\\'on del replay de octubre de 2026). $^*$: $d$ y $WV$ comparten fuente de posici\\'on con mocap "
        "(ver \\S\\ref{sec:metodologia_p2}); no validan la odometr\\'ia del andador.",
        "tab:gait_err_summary",
        [("Par\\'ametro", "l"), ("MAE", "r"), ("MedAE", "r"), ("Bias", "r"), ("RMSE", "r"),
         ("MAPE (\\%)", "r"), ("MedAPE (\\%)", "r")], rows)


def build_group_agg_rows(bags, raw_by_bag, ncols, nds, with_median=False):
    """raw_by_bag: bag -> lista de ncols valores crudos (float/int/str/nan).
    Devuelve una fila de agregado (media +/- sigma) por cada sujeto
    (CA/LA/MF/MT) mas una fila TOTAL final; columnas no numericas (p.ej.
    sexo) muestran el valor si es constante dentro del grupo, si no '--'.
    Con with_median=True se añaden despues filas de MEDIANA (una por sujeto
    y TOTAL): robusta a unos pocos bags con errores muy grandes, que
    inflan la media (ver tablas de comparacion mocap-vs-andador)."""
    groups = group_bags(bags)

    def agg_for(subset, label, use_median=False):
        row = [label]
        for i in range(ncols):
            col_vals = [raw_by_bag[b][i] for b in subset]
            if all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in col_vals):
                if use_median:
                    vals = [v for v in col_vals if not math.isnan(v)]
                    row.append(fmt(statistics.median(vals), nds[i]) if vals else "--")
                else:
                    m, s = mean_std_of(col_vals)
                    row.append(f"{fmt(m, nds[i])}$\\pm${fmt(s, nds[i])}")
            else:
                uniq = set(col_vals)
                row.append(esc(col_vals[0]) if len(uniq) == 1 else "--")
        return row

    rows = [agg_for(gbags, rf"\textbf{{{esc(g)} (n={len(gbags)})}}") for g, gbags in groups.items()]
    rows.append(agg_for(bags, rf"\textbf{{TOTAL (n={len(bags)})}}"))
    if with_median:
        rows += [agg_for(gbags, rf"mediana {esc(g)}", True) for g, gbags in groups.items()]
        rows.append(agg_for(bags, r"\textbf{mediana TOTAL}", True))
    return rows


class Table:
    """Acumula un longtable LaTeX: cabecera fija, una fila por bag, y filas
    finales de agregado (una por sujeto + TOTAL, ver build_group_agg_rows)."""

    def __init__(self, caption, label, col_specs):
        self.caption = caption
        self.label = label
        self.headers = [h for h, _ in col_specs]
        self.aligns = "".join(a for _, a in col_specs)
        self.rows = []

    def add_row(self, cells):
        self.rows.append(cells)

    def render(self, agg_rows=None):
        out = []
        out.append(r"\begin{longtable}{%s}" % self.aligns)
        out.append(r"\caption{%s} \label{%s} \\" % (self.caption, self.label))
        out.append(r"\toprule")
        out.append(" & ".join(self.headers) + r" \\")
        out.append(r"\midrule")
        out.append(r"\endfirsthead")
        out.append(r"\multicolumn{%d}{c}{\tablename\ \thetable{} -- continuacion} \\" % len(self.headers))
        out.append(r"\toprule")
        out.append(" & ".join(self.headers) + r" \\")
        out.append(r"\midrule")
        out.append(r"\endhead")
        out.append(r"\midrule \multicolumn{%d}{r}{continua en la pagina siguiente} \\ \midrule" % len(self.headers))
        out.append(r"\endfoot")
        out.append(r"\bottomrule")
        out.append(r"\endlastfoot")
        for row in self.rows:
            out.append(" & ".join(str(c) for c in row) + r" \\")
        if agg_rows:
            out.append(r"\midrule")
            def is_median(row):
                return "mediana" in str(row[0])

            def is_total(row):
                return "TOTAL" in str(row[0]) and not is_median(row)

            for i, row in enumerate(agg_rows):
                starts_median_block = is_median(row) and (i == 0 or not is_median(agg_rows[i - 1]))
                if starts_median_block or (is_total(row) and len(agg_rows) > 1):
                    out.append(r"\midrule")
                out.append(" & ".join(str(c) for c in row) + r" \\")
        out.append(r"\end{longtable}")
        return "\n".join(out)


def build_gait_mocap_tables(master, bags):
    cols1 = ["left_spt", "left_sdt", "left_spl", "left_sdl", "right_spt", "right_sdt", "right_spl", "right_sdl"]
    nds1 = [3] * 8
    t1 = Table(
        "Punto 1 (mocap): tiempos y longitudes de paso/zancada por pierna, por bag",
        "tab:gait1",
        [("Bag", "l"), ("SpT$_L$ (s)", "r"), ("SdT$_L$ (s)", "r"), ("SpL$_L$ (m)", "r"), ("SdL$_L$ (m)", "r"),
         ("SpT$_R$ (s)", "r"), ("SdT$_R$ (s)", "r"), ("SpL$_R$ (m)", "r"), ("SdL$_R$ (m)", "r")])
    cols2 = ["nos", "tr_s", "d_m", "cad", "wv"]
    nds2 = [1, 2, 3, 2, 3]
    t2 = Table(
        "Punto 1 (mocap): parametros globales de marcha por bag",
        "tab:gait2",
        [("Bag", "l"), ("NoS", "r"), ("Tr (s)", "r"), ("d (m)", "r"), ("CAD (pasos/min)", "r"), ("WV (m/s)", "r")])
    raw1, raw2 = {}, {}
    for b in bags:
        g = master[b]["gait_mocap"]
        v1 = [g[c] for c in cols1]
        v2 = [g[c] for c in cols2]
        raw1[b], raw2[b] = v1, v2
        t1.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(v1, nds1)])
        t2.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(v2, nds2)])
    return (t1.render(build_group_agg_rows(bags, raw1, len(cols1), nds1)),
            t2.render(build_group_agg_rows(bags, raw2, len(cols2), nds2)))


GAIT_CMP_GROUPS = [
    ("spt", ["left_spt", "right_spt"], "Punto 2: Step Time (SpT), mocap vs andador real"),
    ("sdt", ["left_sdt", "right_sdt"], "Punto 2: Stride Time (SdT), mocap vs andador real"),
    ("spl", ["left_spl", "right_spl"], "Punto 2: Step Length (SpL), mocap vs andador real"),
    ("sdl", ["left_sdl", "right_sdl"], "Punto 2: Stride Length (SdL), mocap vs andador real"),
]
GAIT_CMP_GLOBAL = [
    ("nos_d", ["nos", "d_m"], "Punto 2: NoS y distancia recorrida, mocap vs andador real"),
    ("cad_wv", ["cad", "wv"], "Punto 2: Cadencia y velocidad, mocap vs andador real"),
]
FIELD_ND = {"left_spt": 3, "right_spt": 3, "left_sdt": 3, "right_sdt": 3,
            "left_spl": 3, "right_spl": 3, "left_sdl": 3, "right_sdl": 3,
            "nos": 1, "d_m": 3, "cad": 2, "wv": 3}


def build_gait_comparison_tables(master, bags):
    renders = []
    for key, fields, caption in GAIT_CMP_GROUPS + GAIT_CMP_GLOBAL:
        headers = [("Bag", "l")]
        nds = []
        for f in fields:
            label = f.replace("left_", "").replace("right_", "").upper()
            side = "L" if f.startswith("left_") else ("R" if f.startswith("right_") else "")
            hlabel = f"{label}$_{{{side}}}$" if side else label
            headers += [(f"{hlabel} mocap", "r"), (f"{hlabel} andador", "r"), (f"{hlabel} err\\%", "r")]
            nds += [FIELD_ND[f], FIELD_ND[f], 1]
        t = Table(caption, f"tab:cmp_{key}", headers)
        raw = {}
        for b in bags:
            cmp_ = master[b]["gait_comparison"]
            vals = []
            for f in fields:
                v = cmp_[f]
                vals += [v["mocap"], v["walker"], v["pct_err"]]
            raw[b] = vals
            t.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(vals, nds)])
        renders.append(t.render(build_group_agg_rows(bags, raw, len(nds), nds, with_median=True)))
    return renders


def build_cog_mocap_table(master, bags):
    t = Table("Punto 3: centro de gravedad estimado por mocap (modelo De Leva 1996)",
              "tab:cog_mocap",
              [("Bag", "l"), ("Sexo", "c"), ("Peso (kg)", "r"), ("Altura (cm)", "r"),
               ("CoM$_x$ (m)", "r"), ("CoM$_y$ (m)", "r"), ("CoM$_z$ (m)", "r"), ("$z$/estatura", "r")])
    cols = ["user_gender", "user_weight_kg", "user_height_cm", "com_x_mean", "com_y_mean",
            "com_z_mean", "com_height_frac_of_stature"]
    nds = [0, 0, 0, 3, 3, 3, 3]
    raw = {}
    for b in bags:
        c = master[b]["cog_mocap"]
        vals = [c[k] for k in cols]
        raw[b] = vals
        t.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(vals, nds)])
    return t.render(build_group_agg_rows(bags, raw, len(cols), nds))


def build_cog_comparison_table(master, bags):
    t = Table(
        "Punto 3/10: error CoM (mocap, De Leva) vs centroide de soporte real "
        r"(\texttt{/support\_centroid}) -- plano $x$/$y$ (unico comparable, ver \S\ref{sec:cog_cmp})",
        "tab:cog_cmp",
        [("Bag", "l"), ("$x$ MAE (m)", "r"), ("$x$ Bias (m)", "r"), ("$y$ MAE (m)", "r"), ("$y$ Bias (m)", "r"),
         ("Planar MAE (m)", "r"), ("Planar MedAE (m)", "r")])
    nds = [3] * 6
    raw = {}
    for b in bags:
        c = master[b]["cog_comparison"]
        if c is None:
            vals = [float("nan")] * 6
        else:
            x, y, p = c["x"], c["y"], c["planar_xy"]
            vals = [x["mae"], x["bias"], y["mae"], y["bias"], p["mae"], p["medae"]]
        raw[b] = vals
        t.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(vals, nds)])
    return t.render(build_group_agg_rows(bags, raw, len(nds), nds))


def build_posture_tables(master, bags):
    cols_lean = ["trunk_lean_total_deg_mean", "trunk_lean_total_deg_std",
                 "trunk_lean_sagittal_deg_mean", "trunk_lean_sagittal_deg_std",
                 "trunk_lean_frontal_deg_mean", "trunk_lean_frontal_deg_std"]
    nds_lean = [1] * 6
    t_lean = Table("Puntos 1 y 2 del plan: inclinacion de tronco (total/sagital/frontal)",
                    "tab:lean",
                    [("Bag", "l"), ("Total (\\textdegree)", "r"), ("$\\sigma$ Total", "r"),
                     ("Sagital (\\textdegree)", "r"), ("$\\sigma$ Sagital", "r"),
                     ("Frontal (\\textdegree)", "r"), ("$\\sigma$ Frontal", "r")])
    nds_stab = [3, 3, 1, 3, 3]
    t_stab = Table("Puntos 6, 7 y 10 del plan: ancho de paso, doble apoyo y margen de estabilidad lateral",
                    "tab:stab",
                    [("Bag", "l"), ("Ancho paso (m)", "r"), ("$\\sigma$", "r"),
                     ("Doble apoyo (\\%)", "r"), ("Margen lateral", "r"), ("$\\sigma$", "r")])
    raw_lean, raw_stab = {}, {}
    for b in bags:
        p = master[b]["posture_mocap"]
        v_lean = [p[c] for c in cols_lean]
        raw_lean[b] = v_lean
        t_lean.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(v_lean, nds_lean)])
        v_stab = [p["step_width_m_mean"], p["step_width_m_std"], p["double_support_fraction"] * 100,
                  p["lateral_stability_margin_mean"], p["lateral_stability_margin_std"]]
        raw_stab[b] = v_stab
        t_stab.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(v_stab, nds_stab)])
    return (t_lean.render(build_group_agg_rows(bags, raw_lean, len(cols_lean), nds_lean)),
            t_stab.render(build_group_agg_rows(bags, raw_stab, len(nds_stab), nds_stab)))


def build_elbow_wrist_tables(master, bags):
    nds_elb = [1, 1, 1, 1]
    t_elb = Table("Punto 3: angulo de codo (izquierdo/derecho)", "tab:elbow",
                   [("Bag", "l"), ("Codo$_L$ (\\textdegree)", "r"), ("$\\sigma$", "r"),
                    ("Codo$_R$ (\\textdegree)", "r"), ("$\\sigma$", "r")])
    nds_wr = [3, 3, 3, 3]
    t_wr = Table("Punto 4: altura de muñeca y distancia muñeca-tobillo", "tab:wrist",
                  [("Bag", "l"), ("Altura$_L$ (m)", "r"), ("Dist$_L$ (m)", "r"),
                   ("Altura$_R$ (m)", "r"), ("Dist$_R$ (m)", "r")])
    raw_elb, raw_wr = {}, {}
    for b in bags:
        e = master[b]["extra_mocap"]
        v_elb = [e["elbow_left_deg_mean"], e["elbow_left_deg_std"], e["elbow_right_deg_mean"], e["elbow_right_deg_std"]]
        raw_elb[b] = v_elb
        t_elb.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(v_elb, nds_elb)])
        v_wr = [e["wrist_left_height_m_mean"], e["wrist_left_dist_ankle_m_mean"],
                e["wrist_right_height_m_mean"], e["wrist_right_dist_ankle_m_mean"]]
        raw_wr[b] = v_wr
        t_wr.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(v_wr, nds_wr)])
    return (t_elb.render(build_group_agg_rows(bags, raw_elb, len(nds_elb), nds_elb)),
            t_wr.render(build_group_agg_rows(bags, raw_wr, len(nds_wr), nds_wr)))


def build_handle_table(master, bags):
    nds = [1, 1, 3, 3]
    t = Table("Punto 5: reparto de peso en las manetas (sensor real + calibracion)", "tab:handle",
              [("Bag", "l"), ("Fraccion peso (\\%)", "r"), ("$\\sigma$", "r"),
               ("Maneta$_L$ (kg)", "r"), ("Maneta$_R$ (kg)", "r")])
    raw = {}
    for b in bags:
        e = master[b]["extra_mocap"]
        vals = [e["handle_weight_fraction_mean"] * 100, e["handle_weight_fraction_std"] * 100,
                e["handle_left_kg_mean"], e["handle_right_kg_mean"]]
        raw[b] = vals
        t.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(vals, nds)])
    return t.render(build_group_agg_rows(bags, raw, len(nds), nds))


def build_yaw_com_instab_tables(master, bags):
    nds_yaw = [1, 1, 1, 1]
    t_yaw = Table("Punto 8: velocidad de giro (yaw) y radio de curvatura de la trayectoria", "tab:yaw",
                   [("Bag", "l"), ("$|\\dot\\psi|$ (\\textdegree/s)", "r"), ("$\\sigma$", "r"),
                    ("Radio curv. (m)", "r"), ("$\\sigma$", "r")])
    nds_com = [3, 3, 3]
    t_com = Table("Punto 9: oscilacion vertical del centro de masas", "tab:comz",
                   [("Bag", "l"), ("$\\sigma_z$ (m)", "r"), ("Rango$_z$ (m)", "r"), ("Media$_z$ (m)", "r")])
    nds_inst = [1, 2, 1, 1]
    t_inst = Table("Punto 11: indice compuesto de inestabilidad (exploratorio, sin ground truth Tinetti)", "tab:instab",
                    [("Bag", "l"), ("Sway frontal $\\sigma$ (\\textdegree)", "r"), ("CV tiempo de paso", "r"),
                     ("Doble apoyo (\\%)", "r"), ("Indice (0-100)", "r")])
    raw_yaw, raw_com, raw_inst = {}, {}, {}
    for b in bags:
        e = master[b]["extra_mocap"]
        v_yaw = [e["yaw_rate_abs_deg_s_mean"], e["yaw_rate_abs_deg_s_std"],
                 e["curvature_radius_m_mean"], e["curvature_radius_m_std"]]
        raw_yaw[b] = v_yaw
        t_yaw.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(v_yaw, nds_yaw)])
        v_com = [e["com_z_std_m"], e["com_z_range_m"], e["com_z_mean_m"]]
        raw_com[b] = v_com
        t_com.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(v_com, nds_com)])
        v_inst = [e["lateral_sway_frontal_std_deg"], e["step_time_cv"],
                  e["double_support_fraction"] * 100, e["instability_index"]]
        raw_inst[b] = v_inst
        t_inst.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(v_inst, nds_inst)])
    return (t_yaw.render(build_group_agg_rows(bags, raw_yaw, len(nds_yaw), nds_yaw)),
            t_com.render(build_group_agg_rows(bags, raw_com, len(nds_com), nds_com)),
            t_inst.render(build_group_agg_rows(bags, raw_inst, len(nds_inst), nds_inst)))


def _small_table(caption, label, col_specs, rows):
    """Tabla normal (no longtable) para resultados ya agregados por sujeto
    (p.ej. coeficientes de correlacion) -- unas pocas filas, no necesita
    salto de pagina."""
    headers = [h for h, _ in col_specs]
    aligns = "".join(a for _, a in col_specs)
    out = [r"\begin{table}[h]", r"\centering",
           r"\caption{%s} \label{%s}" % (caption, label),
           r"\begin{tabular}{%s}" % aligns, r"\toprule",
           " & ".join(headers) + r" \\", r"\midrule"]
    for row in rows:
        out.append(" & ".join(str(c) for c in row) + r" \\")
    out += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    return "\n".join(out)


def build_imu_tables(master, bags):
    nds_ap = [3, 3, 2, 2]
    t_ap = Table("Ampliacion, punto 1: RMS y Harmonic Ratio en el eje antero-posterior (AP), "
                  "IMU real del andador vs equivalente estimado por mocap", "tab:imu_ap",
                  [("Bag", "l"), ("RMS$_{AP}$ IMU", "r"), ("RMS$_{AP}$ mocap", "r"),
                   ("HR$_{AP}$ IMU", "r"), ("HR$_{AP}$ mocap", "r")])
    nds_ml = [3, 3, 2, 2]
    t_ml = Table("Ampliacion, punto 1: RMS y Harmonic Ratio en el eje medio-lateral (ML)", "tab:imu_ml",
                  [("Bag", "l"), ("RMS$_{ML}$ IMU", "r"), ("RMS$_{ML}$ mocap", "r"),
                   ("HR$_{ML}$ IMU", "r"), ("HR$_{ML}$ mocap", "r")])
    nds_reg = [2, 2, 2, 2]
    t_reg = Table("Ampliacion, punto 1: regularidad de paso (Ad1) y zancada (Ad2) por autocorrelacion",
                   "tab:imu_reg",
                   [("Bag", "l"), ("Ad1 IMU", "r"), ("Ad1 mocap", "r"), ("Ad2 IMU", "r"), ("Ad2 mocap", "r")])
    nds_v = [3, 2]
    t_v = Table("Ampliacion, punto 1: eje vertical (V), solo IMU real -- sin equivalente mocap fiable "
                 "(el chasis del andador apenas se desplaza en $z$)", "tab:imu_v",
                 [("Bag", "l"), ("RMS$_V$ IMU", "r"), ("HR$_V$ IMU", "r")])
    raw_ap, raw_ml, raw_reg, raw_v = {}, {}, {}, {}
    for b in bags:
        im = master[b].get("imu_stability")
        if im is None:
            v_ap, v_ml, v_reg = [float("nan")] * 4, [float("nan")] * 4, [float("nan")] * 4
            v_v = [float("nan")] * 2
        else:
            v_ap = [im["imu_rms_ap"], im["mocap_rms_ap"], im["imu_hr_ap"], im["mocap_hr_ap"]]
            v_ml = [im["imu_rms_ml"], im["mocap_rms_ml"], im["imu_hr_ml"], im["mocap_hr_ml"]]
            v_reg = [im["imu_ad1_step_regularity"], im["mocap_ad1_step_regularity"],
                     im["imu_ad2_stride_regularity"], im["mocap_ad2_stride_regularity"]]
            v_v = [im["imu_rms_v"], im["imu_hr_v"]]
        raw_ap[b], raw_ml[b], raw_reg[b], raw_v[b] = v_ap, v_ml, v_reg, v_v
        t_ap.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(v_ap, nds_ap)])
        t_ml.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(v_ml, nds_ml)])
        t_reg.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(v_reg, nds_reg)])
        t_v.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(v_v, nds_v)])
    return (t_ap.render(build_group_agg_rows(bags, raw_ap, len(nds_ap), nds_ap)),
            t_ml.render(build_group_agg_rows(bags, raw_ml, len(nds_ml), nds_ml)),
            t_reg.render(build_group_agg_rows(bags, raw_reg, len(nds_reg), nds_reg)),
            t_v.render(build_group_agg_rows(bags, raw_v, len(nds_v), nds_v)))


def build_handle_symmetry_tables(master, bags):
    nds = [1, 1, 1, 1, 1]
    t = Table("Ampliacion, punto 2: Symmetry Index (SI) de fuerza en manetas (sensor real) vs "
               "SI cinematico de paso (mocap)", "tab:handle_si",
               [("Bag", "l"), ("SI maneta (\\%)", "r"), ("SI SpT (\\%)", "r"),
                ("SI SdT (\\%)", "r"), ("SI SpL (\\%)", "r"), ("SI SdL (\\%)", "r")])
    raw = {}
    for b in bags:
        hs = master[b].get("handle_symmetry")
        vals = ([float("nan")] * 5 if hs is None else
                [hs["handle_si_pct"], hs["gait_si_spt_pct"], hs["gait_si_sdt_pct"],
                 hs["gait_si_spl_pct"], hs["gait_si_sdl_pct"]])
        raw[b] = vals
        t.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(vals, nds)])
    t_main = t.render(build_group_agg_rows(bags, raw, len(nds), nds))

    corr = master.get("_handle_symmetry_correlations", {})
    rows = []
    groups = sorted({g for key in corr for g in corr[key]} - {"TOTAL"}) + ["TOTAL"]
    for g in groups:
        row = [esc(g) if g != "TOTAL" else r"\textbf{TOTAL}"]
        for key in ("spt", "sdt", "spl", "sdl"):
            r, n = corr.get(key, {}).get(g, (float("nan"), 0))
            row.append(f"{fmt(r, 3)} (n={n})")
        rows.append(row)
    t_corr = _small_table(
        "Ampliacion, punto 2: correlacion de Pearson (SI maneta vs SI cinematico), por sujeto",
        "tab:handle_si_corr",
        [("Sujeto", "l"), ("r vs SpT", "r"), ("r vs SdT", "r"), ("r vs SpL", "r"), ("r vs SdL", "r")],
        rows)
    return t_main, t_corr


def build_lean_margin_table(master, bags):
    nds = [2, 3]
    t = Table("Ampliacion, punto 5: inclinacion sagital de tronco vs margen de estabilidad lateral",
               "tab:lean_margin",
               [("Bag", "l"), ("Lean sagital (\\textdegree)", "r"), ("Margen lateral", "r")])
    raw = {}
    for b in bags:
        lm = master[b].get("lean_vs_margin")
        vals = ([float("nan")] * 2 if lm is None else
                [lm["trunk_lean_sagittal_deg_mean"], lm["lateral_stability_margin_mean"]])
        raw[b] = vals
        t.add_row([esc(b)] + [fmt(v, nd) for v, nd in zip(vals, nds)])
    t_main = t.render(build_group_agg_rows(bags, raw, len(nds), nds))

    corr = master.get("_lean_vs_margin_correlation", {})
    rows = []
    groups = [g for g in corr if g != "TOTAL"] + (["TOTAL"] if "TOTAL" in corr else [])
    for g in groups:
        r, n = corr[g]
        label = esc(g) if g != "TOTAL" else r"\textbf{TOTAL}"
        rows.append([label, f"{fmt(r, 3)} (n={n})"])
    t_corr = _small_table(
        "Ampliacion, punto 5: correlacion de Pearson (lean sagital vs margen lateral), por sujeto",
        "tab:lean_margin_corr",
        [("Sujeto", "l"), ("r", "r")], rows)
    return t_main, t_corr


PREAMBLE = r"""
\documentclass[11pt,a4paper]{article}
\usepackage[utf8]{inputenc}
\usepackage[T1]{fontenc}
\usepackage[spanish]{babel}
\usepackage{amsmath,amssymb}
\usepackage{booktabs}
\usepackage{longtable}
\usepackage{geometry}
\usepackage{hyperref}
\usepackage{textcomp}
\geometry{margin=2.2cm}
\title{Validaci\'on del pipeline sensorial/software del andador rob\'otico WalKit\\
\large Comparaci\'on mocap vs. andador real sobre 47 rosbags etiquetados}
\author{walker\_mocap\_eval}
\date{\today}
\begin{document}
\maketitle
\tableofcontents
\newpage
"""

INTRO = r"""
\section{Introducci\'on y alcance}
Este informe valida los par\'ametros de marcha, carga y postura calculados
por el pipeline software del andador WalKit (\texttt{gait\_monitor\_speed},
\texttt{partial\_loads}, \texttt{walker\_centroid\_support},
\texttt{walker\_stability}) frente a una referencia independiente: captura
de movimiento (mocap) sincronizada, sobre 47 rosbags etiquetados del
dataset \texttt{soma13kp/datasets/labeled\_bags}. Cada bag corresponde a un
\textbf{sujeto} (identificado por el prefijo \texttt{CA}, \texttt{LA},
\texttt{MF} o \texttt{MT}) realizando una repetici\'on de un test de
marcha con el andador instrumentado, con 13 marcadores/keypoints
corporales etiquetados (tronco, hombros, codos, mu\~necas, caderas,
rodillas, tobillos) mas los \emph{rigid bodies} del propio andador. Hay 4
sujetos en total, con entre 7 y 20 repeticiones de test cada uno
(\texttt{CA}: 10, \texttt{LA}: 7, \texttt{MF}: 20, \texttt{MT}: 10).

Todas las estad\'isticas se calculan \textbf{unicamente dentro de la
ventana activa del test}, delimitada por los dos mensajes del topic
\texttt{/tag} (pulsaci\'on de bot\'on de inicio/fin) que ya usa
\texttt{merge\_rosbags.py} para sincronizar las fuentes: fuera de esa
ventana hay tiempo de calibraci\'on/colocaci\'on del sujeto que no es
marcha real y diluye sistem\'aticamente cadencia, velocidad y doble apoyo
si se incluye.

\textbf{Agregados por sujeto, no por el conjunto de bags}: cada tabla de
resultados desglosa primero las 47 repeticiones individuales y termina con
una fila de media\,$\pm$\,desviaci\'on t\'ipica \emph{por sujeto}
(\texttt{CA}/\texttt{LA}/\texttt{MF}/\texttt{MT}) m\'as una fila TOTAL
final. Los 4 sujetos difieren en sexo, peso y altura (ver tabla de la
\S\ref{sec:cog_mocap}), por lo que una \'unica media sobre los 47 bags
mezclar\'ia complexiones distintas y ocultar\'ia diferencias reales entre
sujetos; la fila TOTAL se mantiene solo como referencia de conjunto, no
como resumen principal.

El informe cubre 11 par\'ametros derivables de mocap identificados en la
fase de planificaci\'on, mas la recomputaci\'on/comparaci\'on de los
par\'ametros de marcha ya publicados por el nodo real
(\texttt{gait\_monitor\_speed}), organizados as\'i:
\begin{itemize}
  \item \textbf{Punto 1}: par\'ametros de marcha (SpT/SdT/SpL/SdL, NoS,
    distancia, cadencia, velocidad) recalculados desde mocap.
  \item \textbf{Punto 2}: comparaci\'on de esos mismos par\'ametros contra
    lo que publica el pipeline real (reproducci\'on de cada bag).
  \item \textbf{Puntos 3--11}: inclinaci\'on de tronco, \'angulo de codo,
    altura/distancia de mu\~neca, reparto de peso en manetas, ancho de
    paso, doble apoyo, curvatura/yaw de trayectoria, oscilaci\'on vertical
    del centro de masas (CoM), margen de estabilidad lateral / CoM vs
    centroide de soporte, e \'indice compuesto de inestabilidad
    (exploratorio).
\end{itemize}

Todos los CSV punto a punto (series temporales completas, mocap y andador,
mismo eje temporal en segundos de bag) se entregan junto a este informe
para poder graficar cualquier par\'ametro de cualquier bag por ambas
fuentes.
"""

METHOD = r"""
\section{Metodolog\'ia}

\subsection{Recorte a la ventana activa (\texttt{/tag})}
Cada bag trae exactamente 2 mensajes en \texttt{/tag}
(\texttt{builtin\_interfaces/Time}). Se usa el m\'inimo y el m\'aximo de
esos dos instantes como $[t_0, t_1]$, y tanto el lado mocap
(\texttt{/labeled\_markers}, \texttt{/rigid\_bodies}) como el lado andador
(inicio de la reproducci\'on del bag, \texttt{ros2 bag play
--start-offset}) se recortan a esa ventana antes de calcular cualquier
estad\'istica.

\subsection{Formulas -- Punto 1: par\'ametros de marcha (mocap)}
Puerto literal de \texttt{GaitMonitorSp} (\texttt{gait\_monitor\_speed.py})
sobre eventos de apoyo detectados en mocap:
\begin{itemize}
  \item \textbf{Evento de apoyo}: m\'inimo local de la velocidad horizontal
    del marcador de tobillo (izq./dcha.) por debajo de un umbral
    $v_{th}=0.25$~m/s, con separaci\'on m\'inima entre eventos de 0.3~s.
  \item \textbf{Step Time} $SpT = \overline{t_{apoyo,i}^{pierna} -
    t_{apoyo,\text{ultimo}}^{pierna\ opuesta}}$ (tiempo entre apoyos de
    piernas alternas).
  \item \textbf{Stride Time} $SdT = \overline{t_{apoyo,i}^{pierna} -
    t_{apoyo,i-1}^{pierna}}$ (tiempo entre apoyos de la misma pierna).
  \item \textbf{Step/Stride Length} ($SpL$/$SdL$): igual que arriba pero
    con la distancia eucl\'idea entre posiciones del evento en vez del
    tiempo.
  \item \textbf{NoS} $= N_{apoyos,L} + N_{apoyos,R}$.
  \item \textbf{Distancia} $d$ = longitud de arco del \emph{rigid body} del
    andador en mocap entre el primer y el \'ultimo evento de apoyo.
  \item \textbf{Cadencia} $CAD = 60 \cdot NoS / T_r$ (pasos/min).
  \item \textbf{Velocidad media} $WV = d / T_r$.
\end{itemize}
donde $T_r$ es el tiempo transcurrido entre el primer y el \'ultimo evento
de apoyo detectado (an\'alogo a \texttt{Tr} del nodo real).

\subsection{Formulas -- Punto 2: comparaci\'on con el andador real}
\label{sec:metodologia_p2}
Se reproduce cada bag con el pipeline real completo (\texttt{box\_filter}
$\to$ \texttt{detect\_steps} $\to$ \texttt{partial\_loads} $\to$
\texttt{mocap\_odom\_bridge} $\to$ \texttt{gait\_monitor\_speed}), se
recogen los mensajes acumulativos de \texttt{/left\_gait\_stats},
\texttt{/right\_gait\_stats} y \texttt{/global\_gait\_stats} (WalkStats) a
lo largo de la reproducci\'on, y se comparan con los valores de mocap del
mismo bag campo a campo (\'ultimo valor publicado = resumen acumulado de
todo el test, comparable con el resumen de mocap).

Dos precisiones que condicionan la lectura de los resultados:
\begin{itemize}
  \item \textbf{Ventana de comparaci\'on}: la grabaci\'on del lado andador
    se detiene por tiempo (duraci\'on de la ventana$/$rate$\,+3$~s), por lo
    que sus series llegan unos segundos m\'as all\'a del segundo
    \texttt{/tag}. Solo se usan filas con $t \le t_{fin}$ de la ventana
    (tanto el ``\'ultimo valor'' de los par\'ametros de marcha como las
    muestras del centroide); el tramo posterior, con el usuario ya parado,
    no corresponde a lo que mide mocap.
  \item \textbf{La odometr\'ia es de mocap}: los encoders de rueda de estos
    bags no son utilizables, as\'i que \texttt{/odom} que consume
    \texttt{gait\_monitor\_speed} se genera republicando el \emph{rigid
    body} del andador en mocap. Por tanto la distancia $d$ y la velocidad
    $WV$ del andador y las de mocap comparten, en la pr\'actica, la misma
    fuente de posici\'on: su acuerdo valida la aritm\'etica y la
    temporizaci\'on de \texttt{gait\_monitor\_speed}, \textbf{no la
    precisi\'on de la odometr\'ia del andador}.
\end{itemize}

\subsection{Formulas -- Puntos 3--11}
\begin{itemize}
  \item \textbf{Inclinaci\'on de tronco} (total/sagital/frontal): \'angulo
    entre el vector cadera\_media $\to$ hombro\_medio y la vertical, en
    cada frame con los 4 marcadores disponibles.
  \item \textbf{\'Angulo de codo}: \'angulo en el v\'ertice
    hombro--codo--mu\~neca (ley del coseno sobre los 3 marcadores).
  \item \textbf{Altura/distancia de mu\~neca}: altura ($z$) de la
    mu\~neca y distancia eucl\'idea mu\~neca--tobillo del mismo lado.
  \item \textbf{Reparto de peso en manetas}: $kg_L, kg_R$ del sensor real
    (\texttt{/left\_handle}, \texttt{/right\_handle}) calibrados
    (\texttt{handle\_calib.yaml}); fracci\'on $= (kg_L+kg_R)/peso_{usuario}$.
  \item \textbf{Ancho de paso}: distancia lateral (proyectada al eje del
    andador) entre cada evento de apoyo y el apoyo m\'as cercano en tiempo
    del pie contrario.
  \item \textbf{Doble apoyo (\%)}: fracci\'on de tiempo con solape entre
    los intervalos de apoyo de ambos pies, sobre la duraci\'on total de la
    ventana.
  \item \textbf{Curvatura/yaw}: velocidad angular $\dot\psi$ del
    \emph{rigid body} del andador (diferencia finita del yaw); radio de
    curvatura $R = v/|\dot\psi|$ (se descartan tramos casi rectos,
    $R>50$~m, como ``sin curvatura definida'').
  \item \textbf{Oscilaci\'on vertical del CoM}: $\sigma_z$, rango y media
    de $z_{CoM}(t)$ (ver estimaci\'on de CoM abajo).
  \item \textbf{Margen de estabilidad lateral}: punto medio de caderas
    (proxy de CoM) proyectado sobre el eje lateral del andador,
    normalizado entre los dos tobillos reales ($0$=borde, $1$=centro) --
    misma f\'ormula que \texttt{walker\_stability.py::y\_sta} pero con base
    de soporte (BoS) real en vez de estimada por tablas antropom\'etricas.
  \item \textbf{\'Indice compuesto de inestabilidad} (exploratorio, sin
    ground truth Tinetti disponible en este dataset): combinaci\'on
    normalizada 0--100 de la desviaci\'on t\'ipica del sway frontal, el
    coeficiente de variaci\'on del tiempo de paso y el \% de doble apoyo.
    Se documenta como \'indice \emph{de referencia relativa entre bags},
    no como escala cl\'inica validada.
\end{itemize}

\subsection{Estimaci\'on del centro de masas (CoM): modelo de De Leva (1996)}
Se usa el modelo antropom\'etrico segmentario de De Leva
(\emph{Adjustments to Zatsiorsky-Seluyanov's segment inertia parameters},
J.~Biomech. 29(9):1223--1230, 1996)\footnote{\url{https://ebm.ufabc.edu.br/wp-content/uploads/2013/12/Leva-1996.pdf}},
con tablas espec\'ificas por sexo verificadas contra el art\'iculo
original. Cada segmento corporal (tronco, brazo, antebrazo, muslo,
pantorrilla) se calcula interpolando entre sus dos articulaciones seg\'un
la fracci\'on de posici\'on del CoM de De Leva y se pondera por su
fracci\'on de masa corporal; los 3 segmentos sin marcador propio en el
esquema de 13 keypoints (cabeza, manos, pies) se aproximan concentrando su
masa (${\sim}8.5\%$ del total) en la articulaci\'on proximal disponible. La
altura del usuario no se usa para derivar posiciones (los segmentos tienen
longitud real medida por mocap en cada frame), solo como contraste
(\texttt{com\_height\_frac\_of\_stature}).

\subsection{F\'ormulas de error y precisi\'on}
\label{sec:error_formulas}
Para cada par (mocap $m_i$, andador $w_i$) de $n$ muestras u observaciones
comparables:
\begin{align}
  AE_i &= |w_i - m_i| &\text{(error absoluto)}\\
  MAE &= \frac{1}{n}\sum_i AE_i &\text{(error absoluto medio)}\\
  MedAE &= \mathrm{mediana}(AE_i) &\text{(mediana del error absoluto, robusta a outliers)}\\
  Bias &= \frac{1}{n}\sum_i (w_i - m_i) &\text{(error con signo medio, sesgo sistem\'atico)}\\
  RMSE &= \sqrt{\frac{1}{n}\sum_i (w_i-m_i)^2} &\text{(ra\'iz del error cuadr\'atico medio)}\\
  MAPE &= \frac{100}{n}\sum_i \frac{|w_i-m_i|}{|m_i|}\ \% &\text{(solo si } m_i \neq 0\text{)}\\
  MedAPE &= 100\cdot\mathrm{mediana}\!\left(\frac{|w_i-m_i|}{|m_i|}\right)\ \% &\text{(mediana del error porcentual, robusta a outliers)}
\end{align}
Se reportan la media (MAE, MAPE) y la mediana (MedAE, MedAPE) porque unos
pocos bags tienen errores muy grandes (p.ej. SpT del andador de 2--4~s
frente a $\sim$0.6--1.1~s de mocap, ver \S\ref{sec:limitaciones}); la
mediana no se ve distorsionada por ellos mientras que la media s\'i. Por
eso las tablas de comparaci\'on mocap-andador a\~naden, tras las filas de
media por sujeto y TOTAL, filas de \textbf{mediana} por sujeto y TOTAL. Las medias
``$\pm\sigma$'' por sujeto en cada tabla son media y desviaci\'on t\'ipica
simples sobre las repeticiones de test de ese sujeto (no error respecto a
ninguna referencia).

\subsection{Nota sobre la comparaci\'on de CoM vs. centroide de soporte}
\label{sec:cog_cmp}
\texttt{/support\_centroid} (\texttt{walker\_centroid\_support}) es el
centroide de la \textbf{base de soporte} (altura $\approx 0$, plano del
suelo/base del andador), mientras que el CoM de mocap es el del
\textbf{cuerpo completo} (altura $\approx 0.95$--$1.05$~m). Son magnitudes
f\'isicas distintas por construcci\'on, no el mismo punto en dos frames
distintos: comparar su componente $z$ (o la distancia eucl\'idea 3D) no
aporta informaci\'on \'util y se documenta como \emph{no comparable}. La
comparaci\'on v\'alida es en el plano $x$/$y$ (ambos ya proyectados al
mismo frame del andador mediante la calibraci\'on ICP
\texttt{mocap\_to\_base\_footprint.yaml}).
"""

AMPLIACION_INTRO = r"""
\section{Ampliaci\'on: nuevos par\'ametros con los sensores actuales del andador}
\label{sec:ampliacion}
Tras el informe inicial (puntos 1--11 de \S\ref{sec:ampliacion} en
adelante), se revis\'o la literatura buscando par\'ametros adicionales
medibles con sensores que el andador YA lleva (sin instrumentaci\'on
nueva) y corroborables con el mocap de este mismo dataset. Se identificaron
y verificaron -- contra los topics realmente grabados en los bags, no solo
contra el c\'odigo -- 4 candidatos, implementados en ese orden:

\begin{enumerate}
  \item Metricas de estabilidad por IMU real del andador (\texttt{/bwt901cl/imu\_filtered},
    grabado en todos los bags) vs. su equivalente estimado por mocap.
  \item \'Indice de simetr\'ia de marcha por fuerza en manetas (sensor real) vs. asimetr\'ia
    cinem\'atica de paso (mocap).
  \item Correlaci\'on inclinaci\'on de tronco vs. margen de estabilidad lateral (sin sensor
    nuevo, dos par\'ametros ya calculados).
  \item Validaci\'on de \texttt{walker\_stability.py} (\texttt{/user\_stability}) -- bloqueada,
    ver \S\ref{sec:walker_stability}.
\end{enumerate}
"""

AMPLIACION_IMU = r"""
\subsection{Punto 1: metricas de estabilidad por IMU}
\label{sec:ampliacion_imu}
El andador lleva un IMU real (WitMotion WT901, topic
\texttt{/bwt901cl/imu\_filtered}) montado en el chasis, grabado en los 47
bags pero actualmente solo usado en producci\'on para la velocidad angular
del EKF de localizaci\'on -- la aceleraci\'on lineal no se explota. Se
calculan aqu\'i tres \'indices est\'andar de la literatura de marcha con
IMU de tronco (\S\ref{sec:literatura}, aplicados aqu\'i al chasis del
andador en vez de al tronco del usuario -- ver limitaci\'on en
\S\ref{sec:limitaciones}), sobre DOS fuentes independientes: el IMU real, y
un equivalente estimado por mocap (segunda derivada num\'erica de la
posici\'on del \emph{rigid body} del andador, remuestreada a rejilla
uniforme y suavizada antes de derivar -- derivar dos veces sobre los
\emph{timestamps} irregulares de mocap sin este paso dispara el ruido por
$1/\Delta t^2$ y produce aceleraciones de decenas de m/s$^2$ sin relaci\'on
con el movimiento real, confirmado emp\'iricamente).

\textbf{RMS de aceleraci\'on}: $RMS = \sqrt{\frac{1}{N}\sum_i (a_i - \bar a)^2}$,
por eje (AP=antero-posterior, ML=medio-lateral, V=vertical).

\textbf{Harmonic Ratio} (Moe-Nilssen 1998): con $f_0=1/T_{zancada}$ la
frecuencia fundamental de zancada y $A_h$ la amplitud espectral en el
arm\'onico $h\cdot f_0$ (los primeros 10),
\begin{equation}
  HR_{AP,V} = \frac{\sum_{h=2,4,\dots,10} A_h}{\sum_{h=1,3,\dots,9} A_h}, \qquad
  HR_{ML} = \frac{\sum_{h=1,3,\dots,9} A_h}{\sum_{h=2,4,\dots,10} A_h}
\end{equation}
(en ML se invierte porque su periodicidad natural es de PASO, no de
zancada). Valores m\'as altos indican una marcha m\'as suave/r\'itmica.

\textbf{Regularidad de paso/zancada} (Moe-Nilssen \& Helbostad 2004): Ad1 y
Ad2 son el coeficiente de autocorrelaci\'on normalizada de la se\~nal en el
lag correspondiente al tiempo de paso y de zancada respectivamente,
\begin{equation}
  Ad_k = \max_{\text{lag} \approx \tau_k} \frac{\sum_i (a_i-\bar a)(a_{i+\text{lag}}-\bar a)}{\sum_i (a_i - \bar a)^2}
\end{equation}
con $\tau_1$=tiempo de paso, $\tau_2$=tiempo de zancada (de
\texttt{report\_gait\_mocap.json}); valores cercanos a 1 indican marcha muy
regular/peri\'odica.

No se compara el eje vertical (V) entre fuentes porque el chasis del
andador apenas se desplaza en $z$ (a diferencia de un tronco humano, donde
V es el eje con mayor se\~nal); se reporta el valor real del IMU como
referencia exploratoria (Tabla~\ref{tab:imu_v}).

Resultados sobre 46/47 bags (\texttt{MT\_test11} sin suficientes muestras
de IMU en la ventana \texttt{/tag}): RMS$_{AP}$ IMU y mocap del mismo orden
de magnitud en la mayor\'ia de bags (ambos t\'ipicamente 0.2--0.6 m/s$^2$),
con alguna excepci\'on puntual (p.ej. \texttt{MF\_test06}, mocap muy por
encima del IMU -- probablemente un salto de tracking mocap que sobrevive al
suavizado, ver \S\ref{sec:limitaciones}). HR y Ad1/Ad2 muestran la misma
escala entre fuentes pero con dispersi\'on considerable bag a bag --
esperable dado que son dos mediciones de naturaleza muy distinta (sensor
inercial real vs. doble derivada de posici\'on) del mismo fen\'omeno
f\'isico indirecto (vibraci\'on del chasis por la marcha transmitida a
trav\'es del andador, no medici\'on directa del tronco).
"""

AMPLIACION_HANDLE_SYMMETRY = r"""
\subsection{Punto 2: simetr\'ia de marcha por fuerza en manetas}
\label{sec:ampliacion_handle}
\texttt{/left\_handle}/\texttt{/right\_handle} (sensor real, ya calibrado
para el punto 5 original) permiten un \'indice de simetr\'ia de marcha SIN
mocap, disponible en vivo. Se usa el Symmetry Index (Robinson et al. 1987):
\begin{equation}
  SI = \frac{|X_L - X_R|}{0.5\,(X_L+X_R)} \times 100\ \%
\end{equation}
con $X$ la fuerza media calibrada de cada maneta (kg) durante la ventana
activa -- se promedia la fuerza ANTES de aplicar la f\'ormula (no se
promedia un SI por muestra): la fuerza calibrada puede dar valores
puntuales ligeramente negativos cerca de carga nula (extrapolaci\'on de la
curva de calibraci\'on fuera de rango), lo que distorsiona severamente un
SI calculado muestra a muestra (denominador casi nulo o negativo, llegando
a SI negativos sin sentido f\'isico -- encontrado y corregido durante esta
ampliaci\'on). El mismo SI se aplica a los pares cinem\'aticos de paso ya
calculados desde mocap (SpT, SdT, SpL, SdL izquierda-derecha).

Resultados (Tabla~\ref{tab:handle_si}): el SI de maneta es consistentemente
alto en el sujeto CA ($\sim$190\%, coherente con la asimetr\'ia de carga ya
documentada en \S\ref{sec:cog_mocap} para este sujeto) y mucho m\'as bajo y
variable en el resto. La correlaci\'on con la asimetr\'ia cinem\'atica
(Tabla~\ref{tab:handle_si_corr}) es \textbf{d\'ebil a moderada} en todos
los sujetos ($|r|<0.6$ en todos los casos, TOTAL $|r|<0.2$): la asimetr\'ia
de fuerza en manetas y la asimetr\'ia cinem\'atica del paso no parecen
corroborarse fuertemente entre s\'i en este dataset -- resultado honesto,
no un fallo del m\'etodo: son dos manifestaciones de asimetr\'ia
(carga sobre el andador vs. geometr\'ia del paso) que pueden responder a
mecanismos parcialmente independientes.
"""

AMPLIACION_LEAN_MARGIN = r"""
\subsection{Punto 5: inclinaci\'on de tronco vs. margen de estabilidad lateral}
\label{sec:ampliacion_lean}
Sin sensor nuevo: correlaci\'on de Pearson entre dos par\'ametros ya
calculados (\S\ref{sec:cog_mocap} y antes), inclinaci\'on sagital de tronco
(\texttt{trunk\_lean\_sagittal\_deg\_mean}) y margen de estabilidad lateral
(\texttt{lateral\_stability\_margin\_mean}). La literatura de andadores
instrumentados documenta que inclinarse hacia delante aumenta la
estabilidad \textbf{est\'atica} pero reduce la capacidad de respuesta ante
perturbaciones (\S\ref{sec:literatura}); \textbf{nota importante}: esa
literatura se refiere al margen \textbf{antero-posterior} (hacia delante),
mientras que \texttt{lateral\_stability\_margin} de este informe mide el
margen \textbf{lateral} (izquierda-derecha) -- esta correlaci\'on es, por
tanto, una prueba INDIRECTA/exploratoria de la hip\'otesis general, no una
validaci\'on directa sobre el mismo eje.

Resultados (Tablas~\ref{tab:lean_margin}, \ref{tab:lean_margin_corr}):
correlaci\'on d\'ebil y de signo variable seg\'un sujeto (CA $r=-0.475$, LA
$r=-0.298$, MF $r=-0.240$, MT $r\approx 0$; TOTAL $r=-0.033$) -- no se
observa una relaci\'on clara entre inclinaci\'on sagital y margen LATERAL
en este dataset, consistente con que son ejes mec\'anicamente distintos;
una prueba directa de la hip\'otesis de la literatura requerir\'ia un
margen de estabilidad antero-posterior, no calculado en este informe.
"""

AMPLIACION_WALKER_STABILITY = r"""
\subsection{Punto 3: validaci\'on de \texttt{walker\_stability.py} -- bloqueada}
\label{sec:walker_stability}
Se intent\'o a\~nadir \texttt{walker\_stability} (nodo real, publica
\texttt{/user\_stability}, \texttt{StabilityStamped.sta} $\in[0,1]$) a la
reproducci\'on para comparar su salida real contra el margen de
estabilidad calculado por mocap (\S\ref{sec:ampliacion_lean}), tal como se
dise\~n\'o originalmente \texttt{posture\_metrics.py} para permitir. Se
encontraron DOS bloqueos reales, verificados contra el c\'odigo y contra el
contenido real de los bags, no solo supuestos:

\begin{enumerate}
  \item \textbf{Bug de aridad en \texttt{get\_bos\_x()}}: en
    \texttt{walker\_stability.py::centroid\_callback()}, la llamada
    \texttt{self.get\_bos\_x(self.user\_gender, self.user\_age,
    self.user\_height, self.handle\_x, self.handle\_z)} pasa 5 argumentos,
    pero el m\'etodo solo acepta 4
    (\texttt{gender, age, user\_height\_m, handlebar\_height\_m}) --
    lanzar\'ia \texttt{TypeError} en CADA mensaje de centroide recibido. Es
    decir, \texttt{/user\_stability} probablemente nunca se ha publicado
    con \'exito en ninguna ejecuci\'on real de este nodo tal como est\'a.
  \item \textbf{Frames TF ausentes}: \texttt{centroid\_callback()} necesita
    tambi\'en \texttt{tf\_buffer.lookup\_transform(base\_footprint,
    left\_handle\_id, \dots)} y el equivalente derecho. Se comprob\'o
    \texttt{/tf\_static} de \texttt{CA\_test01} y NINGUNO de los 47 bags
    lleva publicados los frames \texttt{left\_handle\_id}/
    \texttt{right\_handle\_id} (solo \texttt{base\_link}, \texttt{laser},
    \texttt{wheel\_left/right}, \texttt{witmotion},
    \texttt{camera\_*}) -- aunque se corrigiera el bug anterior, el nodo
    seguir\'ia sin poder calcular nada con estos datos.
\end{enumerate}

\textbf{No se ha generado, por tanto, ning\'un dato sint\'etico ni se ha
forzado una reimplementaci\'on con TFs inventados para rellenar este
punto}: habr\'ia introducido supuestos no verificables sobre la geometr\'ia
real de las manetas, debilitando la fiabilidad del resto del informe. El
sustituto m\'as honesto disponible con los datos actuales es el margen de
estabilidad lateral ya calculado desde mocap
(Tabla~\ref{tab:stab}, \S\ref{sec:ampliacion_lean}) -- que adem\'as usa una
base de soporte MEDIDA directamente (tobillos reales en mocap) en vez de
estimada por tablas antropom\'etricas como hace
\texttt{walker\_stability.py::get\_bos\_x()}, por lo que es, si acaso, una
referencia m\'as fiable que lo que el nodo real producir\'ia incluso si
funcionase. \textbf{Recomendaci\'on}: corregir ambos bloqueos (aridad +
a\~nadir broadcast de los frames de maneta en el robot real) antes de
intentar esta validaci\'on en una futura campa\~na de grabaci\'on.
"""

LIMITACIONES = r"""
\section{Limitaciones conocidas y bugs corregidos durante el an\'alisis}
\label{sec:limitaciones}
Durante la validaci\'on se encontraron y corrigieron varios defectos en el
pipeline de producci\'on que afectaban directamente a los n\'umeros
reportados aqu\'i (documentados con detalle en el \texttt{readme.md} del
paquete \texttt{walker\_mocap\_eval}):
\begin{itemize}
  \item \texttt{gait\_monitor\_speed.py::getTravelledDist()} usaba una
    constante de prueba hardcodeada en vez de la distancia real acumulada
    (\texttt{self.travelled}); corregido.
  \item \texttt{km\_detect\_steps.cpp}, \texttt{detect\_steps.cpp} y
    \texttt{detect\_steps\_s.cpp} sellaban cada detecci\'on con el reloj de
    pared (\texttt{this->now()}) en vez del \emph{timestamp} del propio
    escaneo l\'aser -- cr\'itico al reproducir bags a \texttt{rate>1},
    donde ambos relojes divergen; corregido usando
    \texttt{scan->header.stamp}.
  \item \texttt{partial\_loads.launch.py} pasaba un par\'ametro
    \texttt{period} que el nodo nunca declara (solo \texttt{ms\_period}),
    causando un muestreo de carga 10$\times$ m\'as lento del previsto y un
    patr\'on de error bimodal seg\'un la cadencia del sujeto; corregido.
  \item \texttt{TrackLeg::predict\_step()} (EKF de seguimiento de pierna)
    usaba un $\Delta t$ de segundos-desde-el-epoch en su primera
    predicci\'on (centinela \texttt{t==0} mal gestionado), disparando la
    covarianza y produciendo saltos de cientos de metros en un bag
    (\texttt{CA\_test09}); corregido.
  \item El pipeline de reproducci\'on no usaba tiempo de simulaci\'on
    (\texttt{use\_sim\_time}) de forma consistente: los mensajes de
    \texttt{gait\_monitor\_speed} llevaban \emph{timestamps} de reloj de
    pared (hora de la reproducci\'on) en vez del tiempo del propio bag,
    lo que corrompe cualquier eje temporal usado para graficar series
    mocap-vs-andador. Corregido a\~nadiendo \texttt{use\_sim\_time:=true}
    y \texttt{ros2 bag play --clock}, y un campo \texttt{header} nuevo en
    \texttt{WalkStats.msg}.
  \item \texttt{walker\_stability.py::centroid\_callback()} llama a
    \texttt{get\_bos\_x()} con 5 argumentos cuando el m\'etodo solo acepta
    4 (ver \S\ref{sec:walker_stability}) -- lanzar\'ia \texttt{TypeError}
    en cada mensaje de centroide; \textbf{no corregido} (bloquea tambi\'en
    la falta de frames TF de maneta, ver limitaciones persistentes abajo,
    as\'i que arreglar solo este bug no habilitar\'ia el nodo en estos
    bags).
\end{itemize}

Limitaciones que persisten y se documentan expl\'icitamente en los
resultados:
\begin{itemize}
  \item El detector de pasos real (\texttt{detect\_steps}, Random Forest)
    conserva un error residual de detecci\'on del orden del 20--25\% en
    NoS incluso tras los fixes de \emph{timestamp}; se propaga a
    SpT/SdT/SpL/SdL.
  \item El \'indice de inestabilidad (punto 11) es exploratorio: no existe
    en este dataset una escala cl\'inica de referencia (p.ej. Tinetti) con
    la que validarlo directamente.
  \item La comparaci\'on de CoM vs. centroide de soporte usa \'unicamente
    el plano $x$/$y$ (\S\ref{sec:cog_cmp}); la aproximaci\'on de
    cabeza/manos/pies del modelo de De Leva introduce un peque\~no sesgo
    vertical en el CoM de mocap que no afecta a esta comparaci\'on plana.
  \item \texttt{/odom} del experimento de reproducci\'on procede de mocap
    (\S\ref{sec:metodologia_p2}): $d$ y $WV$ no validan la odometr\'ia real
    del andador.
  \item \textbf{Repetibilidad del replay (limitaci\'on abierta)}: la
    reproducci\'on del lado andador no es determinista. A rate 4,
    \texttt{partial\_loads} decide qu\'e pierna carga con un temporizador
    de reloj de pared, as\'i que el resultado depende del instante de bag
    en que cae cada decisi\'on (y de la carga de CPU). Repetir el
    experimento cambia los resultados del lado andador: el MAPE del SpT
    izquierdo fue 38\% en la primera ejecuci\'on (noche del 30 de septiembre al 1 de octubre de 2026) y 58\% en
    la del 6 de octubre (misma forma de c\'alculo; 53\% al acotar a la
    ventana \texttt{/tag}). Todas las cifras del lado andador de este
    informe (tablas de \S\ref{sec:resultados_p2} y cifras citadas en el
    texto) corresponden a \textbf{una sola ejecuci\'on, la de octubre de
    2026}; no se ha cuantificado la variabilidad entre repeticiones (p.ej.
    N repeticiones por bag y dispersi\'on del error), as\'i que diferencias
    de unos pocos puntos de MAPE entre dos ejecuciones no deben
    interpretarse como mejora o empeoramiento del pipeline. Los
    resultados calculados \'unicamente desde mocap y desde sensores
    grabados (puntos de marcha de mocap, postura, IMU, simetr\'ia de
    maneta) s\'i son reproducibles.
  \item Ejemplo del efecto en \texttt{CA\_test01} (ejecuci\'on de octubre de
    2026): tras t$\approx$44~s el pie derecho deja de registrar apoyos
    mientras el izquierdo sigue registrando 10 m\'as; cada ``paso''
    izquierdo se mide contra un apoyo derecho cada vez m\'as antiguo
    (1--8~s) y \texttt{gait\_monitor\_speed}, que promedia con
    \texttt{nanmean} sin rechazar valores at\'ipicos, sube el SpT medio de
    0.7 a 1.9~s. En la ejecuci\'on anterior este bag terminaba en 0.58~s.
  \item Los 47 bags NO llevan publicados los frames TF
    \texttt{left\_handle\_id}/\texttt{right\_handle\_id} que
    \texttt{walker\_stability.py} necesita, ni el fix del bug anterior
    bastar\'ia por s\'i solo -- \S\ref{sec:walker_stability}.
  \item El equivalente de aceleraci\'on del andador por mocap
    (\S\ref{sec:ampliacion_imu}) es sensible a saltos puntuales de tracking
    mocap que sobreviven al suavizado (p.ej. \texttt{MF\_test06}); afecta a
    RMS/HR de ese bag concreto, no al resto.
\end{itemize}
"""

LITERATURE = r"""
\section{Comparaci\'on con la literatura}
\label{sec:literatura}
No se dispone en este dataset de una segunda referencia externa (p.ej. un
sistema comercial ya validado) con la que triangular directamente; la
comparaci\'on que sigue usa como referencia estudios de validaci\'on
publicados de sensores port\'atiles (IMU, acelerometr\'ia de pie) frente a
mocap, que miden tareas conceptualmente similares (par\'ametros discretos
de marcha) aunque no sobre poblaci\'on de usuarios de andador.

\textbf{Nota sobre las cifras}: los porcentajes y rangos citados abajo y en
las conclusiones para el lado andador (MAPE, MedAPE, error planar del
centroide) est\'an escritos a mano para \textbf{la ejecuci\'on del replay de
octubre de 2026}; las tablas se calculan de los datos de esa misma
ejecuci\'on. El replay no es determinista (\S\ref{sec:limitaciones}), por lo
que otra ejecuci\'on dar\'a cifras algo distintas y habr\'a que revisar este
texto.

\begin{itemize}
  \item \textbf{Velocidad y distancia} (aqu\'i: MAPE $WV\approx3.9\%$,
    $d\approx1.4\%$; MedAPE 2.6\% y 0.05\%): \textbf{no son una validaci\'on
    de la odometr\'ia del andador}, y por eso no se comparan con la
    literatura de precisi\'on de sensores. El \texttt{/odom} del pipeline
    reproducido sale del propio mocap (\S\ref{sec:metodologia_p2}); una
    mediana de error de 0.05\% en $d$ es la firma de esa circularidad. Lo
    que s\'i muestran estos valores es que la aritm\'etica de
    \texttt{gait\_monitor\_speed} (distancia acumulada, $T_r$, velocidad) es
    correcta. Para referencia, estudios de validaci\'on de IMU vs.\ mocap
    reportan concordancia muy alta de la velocidad de marcha
    (ICC$\approx0.98$, MAE$\approx0.07$~m/s) y de cadencia/longitud de
    zancada (ICC 0.96--0.97)\footnote{\url{https://formative.jmir.org/2026/1/e80728}};
    validar la odometr\'ia real del andador (l\'aser + IMU) frente a mocap
    es trabajo aparte.
  \item \textbf{Cadencia y NoS} (aqu\'i: MAPE 25--26\%, MedAPE 23--24\%): muy por encima del
    MAPE de cadencia reportado para IMU de pie bien calibrados
    ($\approx4.1\%$ en un estudio de dispositivos portables, que reporta
    sin embargo un MAPE mucho peor -- 14.9--19.0\% -- para longitud de
    zancada y tiempo de contacto, par\'ametros derivados de detecci\'on
    discreta de eventos como NoS/SpT/SdT aqu\'i)\footnote{\url{https://0-bmcsportsscimedrehabil-biomedcentral-com.brum.beds.ac.uk/counter/pdf/10.1186/s13102-023-00792-3.pdf}}.
    Otro estudio con IMU de pie reporta ICC 0.92 (tiempo de paso) y 0.94
    (tiempo de zancada), con RMSE por debajo de 40~ms en poblaci\'on
    sana\footnote{\url{https://pmc.ncbi.nlm.nih.gov/articles/PMC8472002}}.
    En la literatura, los par\'ametros que dependen de detectar eventos
    discretos de paso son los de peor acuerdo; aqu\'i ocurre lo mismo con
    NoS, cadencia y tiempos/longitudes de paso.
  \item \textbf{SpT/SdT/SpL/SdL} (aqu\'i: MAPE 22--89\%, MedAPE 17--61\%): por encima del
    rango t\'ipico de los sistemas de referencia bien calibrados citados
    arriba (ICC$>$0.9, MAPE de un d\'igito para cadencia), consistente con
    que la causa ra\'iz identificada aqu\'i es un detector de pasos por
    l\'aser sin filtro de confianza robusto (\S\ref{sec:limitaciones}), no
    la metodolog\'ia de comparaci\'on.
  \item \textbf{Doble apoyo} (aqu\'i: 3--56\% seg\'un bag, tras recorte a
    la ventana activa): en marcha libre sana, el doble apoyo ocupa
    t\'ipicamente 20--30\% del ciclo a paso normal, aumentando sin
    asistencia a medida que la velocidad disminuye (hasta un 12\% del
    tiempo de zancada a baja velocidad, frente a un 5\% a 3~m/s)\footnote{\url{https://clinicalgaitanalysis.com/data/stance.html}},
    y las poblaciones de mayor edad muestran sistem\'aticamente una
    proporci\'on de doble apoyo mayor que adultos j\'ovenes\footnote{\url{https://par.nsf.gov/servlets/purl/10101238}}.
    El rango observado aqu\'i (hasta 56\% en algunos bags) es compatible
    con marcha asistida por andador, donde el apoyo bimanual permanente en
    las manetas favorece fases de doble apoyo m\'as largas que en marcha
    libre -- no indica un error de medici\'on.
  \item \textbf{CoM (De Leva) vs.\ centroide de soporte real}: el modelo
    antropom\'etrico segmentario de De Leva (1996) es un est\'andar amplia-
    mente usado en biomec\'anica para estimar par\'ametros
    inerciales de segmentos corporales a partir de posiciones
    articulares\footnote{\url{https://ebm.ufabc.edu.br/wp-content/uploads/2013/12/Leva-1996.pdf}},
    pero no se ha encontrado en la b\'usqueda realizada una cifra de
    precisi\'on del modelo en s\'i directamente comparable a la magnitud
    medida aqu\'i (error planar frente a un centroide de carga, no frente
    al propio CoM medido por una plataforma de fuerza). No se reporta, por
    tanto, una comparaci\'on cuantitativa directa con literatura para este
    punto concreto; se destaca \'unicamente que el modelo est\'a bien
    establecido y que el error planar encontrado (10--15~cm sobre una base
    de apoyo de $\sim$1~m) es del orden esperable al comparar dos puntos
    de equilibrio relacionados pero f\'isicamente distintos
    (\S\ref{sec:cog_cmp}).
  \item \textbf{Harmonic Ratio / RMS de aceleraci\'on} (\S\ref{sec:ampliacion_imu}):
    el Harmonic Ratio de la aceleraci\'on de tronco decae linealmente con
    la edad y distingue sujetos mayores inestables, y un RMS de
    aceleraci\'on m\'as bajo junto con mayor asimetr\'ia de tiempo de paso
    son marcadores establecidos de peor estabilidad en marcha de
    mayores\footnote{\url{https://pmc.ncbi.nlm.nih.gov/articles/PMC7348719}}\footnote{\url{https://documentserver.uhasselt.be/bitstream/1942/49006/1/sensors-26-02194.pdf}}.
    Aqu\'i el sensor est\'a en el chasis del andador, no en el tronco del
    usuario, as\'i que los valores absolutos no son directamente
    comparables a esa literatura (poblaci\'on y punto de medici\'on
    distintos); el valor de este punto es metodol\'ogico -- que la misma
    f\'ormula aplicada a dos fuentes independientes (IMU real y mocap) da
    resultados del mismo orden de magnitud, no la comparaci\'on de esos
    valores contra normas poblacionales externas.
  \item \textbf{Simetr\'ia de marcha por fuerza en manetas}
    (\S\ref{sec:ampliacion_handle}): hay precedente directo de un andador
    rob\'otico ("Smart Walker") que detecta asimetr\'ia de marcha
    analizando el par de interacci\'on usuario-andador en las
    manetas\footnote{\url{https://arxiv.org/html/2408.12005v1}}; la
    correlaci\'on d\'ebil encontrada aqu\'i entre el SI de maneta y el SI
    cinem\'atico no contradice ese precedente (que valida la asimetr\'ia de
    maneta como se\~nal en s\'i misma, no su correlaci\'on 1:1 con la
    asimetr\'ia cinem\'atica).
  \item \textbf{Inclinaci\'on hacia delante y estabilidad}
    (\S\ref{sec:ampliacion_lean}): la literatura de andadores instrumentados
    documenta que inclinarse hacia delante aumenta la estabilidad
    est\'atica pero reduce la capacidad de respuesta ante
    perturbaciones\footnote{\url{https://www.ati-ia.com/Library/documents/walker_forces.pdf}}
    -- sobre el margen ANTERO-POSTERIOR, no el lateral calculado aqu\'i
    (\S\ref{sec:ampliacion_lean}), de ah\'i la correlaci\'on d\'ebil
    observada.
\end{itemize}

\textbf{Conclusi\'on de la comparaci\'on}: los par\'ametros que dependen de
la detecci\'on discreta de pasos (cadencia, NoS, tiempos/longitudes de
paso) muestran un error (MAPE 22--89\%) muy superior al t\'ipico de la
literatura de sensores port\'atiles, lo que localiza d\'onde debe
enfocarse la siguiente mejora del pipeline: el detector de pasos y la
asignaci\'on de carga por pierna. La distancia y la velocidad no permiten
concluir nada sobre la precisi\'on de la odometr\'ia del andador, porque su
fuente de posici\'on en este experimento es el propio mocap. De los 4
par\'ametros nuevos de la ampliaci\'on, el de mayor potencial pr\'actico es
la simetr\'ia por fuerza en manetas (\S\ref{sec:ampliacion_handle}): es el
\'unico que no necesita mocap en absoluto para calcularse en vivo, con
precedente directo en la literatura de andadores inteligentes.
"""

CONCLUSION = r"""
\section{Conclusiones}
\begin{enumerate}
  \item Se ha construido y validado (autotest sint\'etico + comparaci\'on
    con datos reales) un conjunto reutilizable de herramientas
    (\texttt{walker\_mocap\_eval}) para recalcular desde mocap cualquiera
    de los 11 par\'ametros de este informe sobre nuevos bags etiquetados
    futuros, sin depender de la reproducci\'on del pipeline real salvo
    para el punto 2 y la comparaci\'on de CoM.
  \item Durante el proceso se identificaron y corrigieron 5 defectos reales
    en el pipeline de producci\'on (\S\ref{sec:limitaciones}), todos con
    impacto directo y cuantificado en los par\'ametros publicados por el
    andador.
  \item La distancia y la velocidad de \texttt{gait\_monitor\_speed} coinciden
    con mocap (MedAPE de $d$ del 0.05\%), pero eso valida su aritm\'etica y
    no la odometr\'ia del andador: el \texttt{/odom} del experimento sale
    de mocap. Los par\'ametros basados en detecci\'on discreta de eventos
    de paso conservan un error del 22--89\% (MAPE) seg\'un el campo
    (mediana 17--61\%), localizado en el detector de pasos y en la
    asignaci\'on de carga por pierna, no en el resto del pipeline.
  \item La estimaci\'on de CoM por mocap (De Leva) y el centroide de
    soporte real del andador sit\'uan el punto de equilibrio del usuario en
    la misma zona con un error planar del orden de 10--15~cm sobre una base
    de apoyo de ${\sim}1$~m, sin outliers tras los fixes del EKF.
  \item Los par\'ametros posturales nuevos (inclinaci\'on de tronco,
    \'angulo de codo, reparto de peso en manetas, ancho de paso, doble
    apoyo, curvatura de trayectoria, oscilaci\'on vertical del CoM) quedan
    caracterizados por primera vez sobre los 47 bags (4 sujetos) y
    disponibles como serie temporal completa en los CSV adjuntos, listos
    para cruzarse en trabajo futuro con variables de configuraci\'on del
    test (p.ej. altura de manetas) o con m\'etricas cl\'inicas de
    estabilidad si se incorporan a este dataset.
  \item De los 4 par\'ametros nuevos de la ampliaci\'on
    (\S\ref{sec:ampliacion}), 3 se han podido calcular y validar sobre los
    47 bags (IMU, simetr\'ia por maneta, correlaci\'on inclinaci\'on/margen);
    el cuarto (validaci\'on de \texttt{walker\_stability.py}) qued\'o
    bloqueado por un bug de aridad y por frames TF ausentes en el dataset
    (\S\ref{sec:walker_stability}) -- documentado como hallazgo en vez de
    forzarse con supuestos no verificables.
\end{enumerate}
"""

BIBLIOGRAPHY = r"""
\section{Referencias bibliogr\'aficas}
\begin{enumerate}
  \item Moe-Nilssen R. A new method for evaluating motor control in gait
    under real-life environmental conditions. Part 2: Gait analysis.
    \emph{Clinical Biomechanics} 1998;13:328--335. (Harmonic Ratio,
    \S\ref{sec:ampliacion_imu}).
  \item Moe-Nilssen R, Helbostad JL. Estimation of gait cycle
    characteristics by trunk accelerometry. \emph{Journal of Biomechanics}
    2004;37(1):121--126. (regularidad de paso/zancada por autocorrelaci\'on,
    \S\ref{sec:ampliacion_imu}).
  \item Robinson RO, Herzog W, Nigg BM. Use of force platform variables to
    quantify the effects of chiropractic manipulation on gait symmetry.
    \emph{Journal of Manipulative and Physiological Therapeutics}
    1987;10(4):172--176. (Symmetry Index, \S\ref{sec:ampliacion_handle}).
  \item de Leva P. Adjustments to Zatsiorsky-Seluyanov's segment inertia
    parameters. \emph{Journal of Biomechanics} 1996;29(9):1223--1230.
    \url{https://ebm.ufabc.edu.br/wp-content/uploads/2013/12/Leva-1996.pdf}
    (modelo de CoG, \S\ref{sec:cog_mocap}).
  \item Harmonic ratio y marcha en mayores (validaci\'on trunk
    accelerometry). \url{https://pmc.ncbi.nlm.nih.gov/articles/PMC7348719}
  \item RMS de aceleraci\'on de tronco y asimetr\'ia de marcha en mayores.
    \url{https://documentserver.uhasselt.be/bitstream/1942/49006/1/sensors-26-02194.pdf}
  \item Validaci\'on de velocidad/cadencia/longitud de zancada por IMU vs.
    mocap (ICC, MAE). \url{https://formative.jmir.org/2026/1/e80728}
  \item MAPE de cadencia/longitud de zancada/tiempo de contacto, sensores
    portables. \url{https://0-bmcsportsscimedrehabil-biomedcentral-com.brum.beds.ac.uk/counter/pdf/10.1186/s13102-023-00792-3.pdf}
  \item ICC y RMSE de tiempo de paso/zancada, IMU de pie.
    \url{https://pmc.ncbi.nlm.nih.gov/articles/PMC8472002}
  \item Normas de doble apoyo y velocidad de marcha.
    \url{https://clinicalgaitanalysis.com/data/stance.html}
  \item Doble apoyo en marcha de mayores vs. adultos j\'ovenes.
    \url{https://par.nsf.gov/servlets/purl/10101238}
  \item Evaluating Gait Symmetry with a Smart Robotic Walker (asimetr\'ia
    por fuerza/par de interacci\'on en manetas).
    \url{https://arxiv.org/html/2408.12005v1}
  \item Walker forces -- fuerzas e inclinaci\'on en andadores instrumentados.
    \url{https://www.ati-ia.com/Library/documents/walker_forces.pdf}
\end{enumerate}
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--master", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    with open(args.master) as f:
        master = json.load(f)
    bags = sorted(k for k in master.keys() if not k.startswith("_"))

    t_gait1, t_gait2 = build_gait_mocap_tables(master, bags)
    t_gait_cmp = build_gait_comparison_tables(master, bags)
    t_cog_mocap = build_cog_mocap_table(master, bags)
    t_cog_cmp = build_cog_comparison_table(master, bags)
    t_lean, t_stab = build_posture_tables(master, bags)
    t_elb, t_wr = build_elbow_wrist_tables(master, bags)
    t_handle = build_handle_table(master, bags)
    t_yaw, t_com, t_inst = build_yaw_com_instab_tables(master, bags)
    t_imu_ap, t_imu_ml, t_imu_reg, t_imu_v = build_imu_tables(master, bags)
    t_hsym_main, t_hsym_corr = build_handle_symmetry_tables(master, bags)
    t_leanmargin_main, t_leanmargin_corr = build_lean_margin_table(master, bags)

    doc = [PREAMBLE, INTRO, METHOD]
    doc.append(r"\section{Punto 1: par\'ametros de marcha desde mocap (resultados)}")
    doc.append(t_gait1)
    doc.append(t_gait2)
    doc.append(r"\section{Punto 2: comparaci\'on mocap vs. andador real (resultados)}\label{sec:resultados_p2}")
    doc.append(build_gait_error_summary(master))
    doc.extend(t_gait_cmp)
    doc.append(r"\section{Punto 3: centro de gravedad (De Leva) y comparaci\'on con el andador}")
    doc.append(r"\label{sec:cog_mocap}")
    doc.append(t_cog_mocap)
    doc.append(t_cog_cmp)
    doc.append(r"\section{Inclinaci\'on de tronco, ancho de paso, doble apoyo y margen de estabilidad}")
    doc.append(t_lean)
    doc.append(t_stab)
    doc.append(r"\section{\'Angulo de codo y altura/distancia de mu\~neca}")
    doc.append(t_elb)
    doc.append(t_wr)
    doc.append(r"\section{Reparto de peso en las manetas}")
    doc.append(t_handle)
    doc.append(r"\section{Curvatura/yaw, oscilaci\'on vertical del CoM e \'indice de inestabilidad}")
    doc.append(t_yaw)
    doc.append(t_com)
    doc.append(t_inst)
    doc.append(AMPLIACION_INTRO)
    doc.append(AMPLIACION_IMU)
    doc.append(t_imu_ap)
    doc.append(t_imu_ml)
    doc.append(t_imu_reg)
    doc.append(t_imu_v)
    doc.append(AMPLIACION_HANDLE_SYMMETRY)
    doc.append(t_hsym_main)
    doc.append(t_hsym_corr)
    doc.append(AMPLIACION_LEAN_MARGIN)
    doc.append(t_leanmargin_main)
    doc.append(t_leanmargin_corr)
    doc.append(AMPLIACION_WALKER_STABILITY)
    doc.append(LIMITACIONES)
    doc.append(LITERATURE)
    doc.append(CONCLUSION)
    doc.append(BIBLIOGRAPHY)
    doc.append(r"\end{document}")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(doc), encoding="utf-8")
    print(f"Escrito {out_path} ({len(bags)} bags, {len(group_bags(bags))} sujetos)")


if __name__ == "__main__":
    main()
