#!/usr/bin/env python3
"""Parametros restantes de la tabla del plan del paso 3 (puntos 3, 4, 5, 8,
9, 11 -- 1/2/6/7/10 ya estan en posture_metrics.py/cog_estimation.py):

  3. Angulo de codo (hombro-codo-muñeca) por lado.
  4. Altura de muñeca y distancia muñeca-tobillo del mismo lado.
  5. Descarga de peso en manetas (% del peso del usuario), directamente del
     sensor crudo de maneta (/left_handle, /right_handle) + la curva de
     calibracion de partial_loads (config/handle_calib.yaml) -- NO requiere
     replay: es la misma interpolacion que hace partial_loads.cpp, aplicada
     aqui offline sobre las lecturas crudas ya grabadas en el bag.
  8. Curvatura/velocidad angular de la trayectoria del andador
     (rigid_bodies[0]) -- yaw rate, y radio de curvatura instantaneo.
  9. Oscilacion vertical del CoM (necesita cog_estimation.py: dispersion y
     rango de com_z a lo largo del tiempo).
  11. Indice compuesto de inestabilidad, EXPLORATORIO: combinacion de
     variabilidad de balanceo lateral + variabilidad de tiempo de paso +
     doble apoyo, pensado para parecerse a lo que mide un Tinetti/POMA
     (mayor variabilidad e inestabilidad = mayor riesgo). Los bags de
     datasets/labeled_bags NO tienen una puntuacion Tinetti real con la que
     validar esto (test_config.txt no trae esa columna, ver
     replay_offline.launch.py) -- es un indice ilustrativo sin ground
     truth, no una recreacion validada del test clinico.
"""
import math

import numpy as np
from scipy.interpolate import interp1d


def angle_at_vertex(a, b, c):
    """Angulo en el vertice b, formado por los segmentos b->a y b->c, en
    grados -- usado para el codo (a=hombro, b=codo, c=muñeca): 180 grados
    es brazo extendido, angulos menores son mas flexion."""
    v1 = np.array(a) - np.array(b)
    v2 = np.array(c) - np.array(b)
    n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
    if n1 == 0 or n2 == 0:
        return float("nan")
    cos_a = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
    return math.degrees(math.acos(cos_a))


def elbow_angle_series(shoulder_track, elbow_track, wrist_track, max_dt_ns):
    """[(t_ns, angle_deg)], emparejando por tiempo (ver bag_io.indexed_nearest)."""
    from . import bag_io

    sh_q = bag_io.indexed_nearest(shoulder_track)
    wr_q = bag_io.indexed_nearest(wrist_track)
    out = []
    for t, elbow in elbow_track:
        sh = sh_q(t, max_dt_ns)
        wr = wr_q(t, max_dt_ns)
        if sh is None or wr is None:
            continue
        out.append((t, angle_at_vertex(sh[1], elbow, wr[1])))
    return out


def wrist_height_and_ankle_dist_series(wrist_track, ankle_track, max_dt_ns):
    """[(t_ns, wrist_z, dist_wrist_ankle_m)] -- altura de muñeca (z, mocap
    world frame) y distancia 3D muñeca-tobillo del MISMO lado, emparejando
    por tiempo. La distancia decae cuando el usuario suelta/baja la mano
    hacia el tobillo (irrelevante en marcha con andador salvo posturas muy
    atipicas) y aumenta con el brazo extendido hacia la maneta."""
    from . import bag_io

    ank_q = bag_io.indexed_nearest(ankle_track)
    out = []
    for t, wrist in wrist_track:
        ank = ank_q(t, max_dt_ns)
        if ank is None:
            continue
        dist = math.dist(wrist, ank[1])
        out.append((t, wrist[2], dist))
    return out


def yaw_rate_series(rb_track_xy_yaw):
    """[(t_ns, yaw_rate_deg_s, curvature_radius_m)] del propio rigid body
    del andador -- diferencias centradas de yaw (con wraparound a
    [-pi, pi]) entre muestras consecutivas, y radio de curvatura
    instantaneo = velocidad_lineal / |yaw_rate| (nan si yaw_rate~0, tramo
    recto)."""
    out = []
    for i in range(1, len(rb_track_xy_yaw) - 1):
        t0, (x0, y0, yaw0) = rb_track_xy_yaw[i - 1]
        t1, (x1, y1, yaw1) = rb_track_xy_yaw[i + 1]
        dt = (t1 - t0) / 1e9
        if dt <= 0:
            continue
        dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
        yaw_rate = math.degrees(dyaw / dt)
        lin_speed = math.hypot(x1 - x0, y1 - y0) / dt
        radius = lin_speed / abs(math.radians(yaw_rate)) if abs(yaw_rate) > 1e-6 else float("nan")
        t_mid, _ = rb_track_xy_yaw[i]
        out.append((t_mid, yaw_rate, radius))
    return out


def vertical_com_oscillation(com_series_xyz):
    """(std_m, range_m, mean_z_m) de la componente z de una serie de CoM
    (t_ns, (x,y,z)) -- amplitud del "rebote" vertical de la marcha
    (double-bump), mas alto en marcha vigorosa/menos amortiguada."""
    zs = [c[2] for _, c in com_series_xyz]
    if len(zs) < 2:
        return float("nan"), float("nan"), float("nan")
    return float(np.std(zs)), float(max(zs) - min(zs)), float(np.mean(zs))


def build_handle_calibration(handle_calib_yaml):
    """Curva de calibracion cruda->kg de walker_loads/config/
    handle_calib.yaml, exactamente la misma interpolacion lineal (con
    extrapolacion) que partial_loads.cpp/py aplican en vivo. Devuelve
    (interp_left, interp_right), cada uno callable(raw_adc) -> kg."""
    import yaml

    with open(handle_calib_yaml) as f:
        d = yaml.safe_load(f)
    params = d["partial_loads"]["ros__parameters"]
    weight_points = params["weight_points"]
    left = interp1d(params["left_handle_points"], weight_points, fill_value="extrapolate")
    right = interp1d(params["right_handle_points"], weight_points, fill_value="extrapolate")
    return left, right


def handle_weight_fraction_series(left_handle_msgs, right_handle_msgs, interp_left, interp_right, user_weight_kg):
    """[(t_ns, left_kg, right_kg, total_fraction)] -- fuerza calibrada (kg)
    de cada maneta, y fraccion del peso del usuario sostenida por AMBAS
    manetas en ese instante (msgs de /left_handle, /right_handle NO estan
    sincronizados 1:1 -- cada canal se calibra con su propio timestamp; la
    fraccion combinada usa el valor mas reciente conocido de la otra
    maneta en cada evento, igual que hace partial_loads.cpp con
    left_hand_msg_/right_hand_msg_)."""
    events = sorted(
        [(t, "left", m) for t, m in left_handle_msgs] + [(t, "right", m) for t, m in right_handle_msgs],
        key=lambda r: r[0])
    out = []
    last_left_kg = last_right_kg = 0.0
    have_left = have_right = False
    for t, side, msg in events:
        if side == "left":
            last_left_kg = float(interp_left(msg.force))
            have_left = True
        else:
            last_right_kg = float(interp_right(msg.force))
            have_right = True
        if have_left and have_right and user_weight_kg:
            frac = (last_left_kg + last_right_kg) / user_weight_kg
            out.append((t, last_left_kg, last_right_kg, frac))
    return out


def symmetry_index(a, b):
    """Symmetry Index (Robinson et al. 1987), en %: SI = |a-b| / (0.5*(a+b))
    * 100 -- 0% = simetria perfecta, mayor valor = mas asimetria entre lado
    izquierdo/derecho. Formula estandar en literatura de marcha, aplicada
    aqui tanto a fuerza en manetas (de sensor real) como a parametros
    cinematicos de paso (de mocap) para poder comparar ambas fuentes de
    asimetria entre si (punto 2 de la ampliacion del informe)."""
    denom = 0.5 * (a + b)
    if denom == 0 or math.isnan(a) or math.isnan(b):
        return float("nan")
    return abs(a - b) / denom * 100.0


def handle_force_symmetry_series(left_handle_msgs, right_handle_msgs, interp_left, interp_right):
    """[(t_ns, SI_pct)] -- Symmetry Index (ver symmetry_index()) entre la
    fuerza calibrada (kg) de cada maneta, emparejando por el ultimo valor
    conocido de cada canal (mismo criterio de sincronizacion que
    handle_weight_fraction_series()). Proxy de asimetria de marcha
    DISPONIBLE EN VIVO (sensor real de maneta, sin necesidad de mocap) --
    se valida aqui contra la asimetria cinematica de paso (SpT/SdT/SpL/SdL
    izquierda-derecha) ya calculada desde mocap.

    La fuerza calibrada puede salir ligeramente negativa cerca de carga
    nula (extrapolacion de la curva), lo que hace explotar/invertir el SI
    por muestra: se recorta a >=0 y, si L+R < 0.5 kg (denominador casi
    nulo), el SI de esa muestra es NaN. Devuelve [(t_ns, SI_pct, L_kg, R_kg)]
    con los kg ya recortados."""
    events = sorted(
        [(t, "left", m) for t, m in left_handle_msgs] + [(t, "right", m) for t, m in right_handle_msgs],
        key=lambda r: r[0])
    out = []
    last_left_kg = last_right_kg = 0.0
    have_left = have_right = False
    for t, side, msg in events:
        if side == "left":
            last_left_kg = float(interp_left(msg.force))
            have_left = True
        else:
            last_right_kg = float(interp_right(msg.force))
            have_right = True
        if have_left and have_right:
            lk, rk = max(0.0, last_left_kg), max(0.0, last_right_kg)
            si = symmetry_index(lk, rk) if lk + rk >= 0.5 else float("nan")
            out.append((t, si, lk, rk))
    return out


def composite_instability_index(lateral_sway_std_deg, step_time_cv, double_support_frac):
    """Indice EXPLORATORIO (0-100, mayor = mas inestable), sin validar
    contra ningun Tinetti/POMA real (no hay ground truth en estos bags --
    ver docstring del modulo): media de 3 componentes normalizados de forma
    aproximada/heuristica a un rango 0-100 tipico observado en este mismo
    dataset (no una escala clinica):
      - balanceo lateral (std de la inclinacion frontal de tronco, grados):
        /10 grados de std -> 100.
      - variabilidad del tiempo de paso (coef. de variacion = std/media):
        /0.5 de CV -> 100.
      - doble apoyo (%): ya es 0-100 directamente; mas doble apoyo suele
        asociarse a marcha mas cautelosa/menos estable en poblacion
        general, aunque en marcha con andador un doble apoyo alto puede
        tambien reflejar simplemente lentitud deliberada, no inestabilidad
        -- este componente es el mas discutible de los tres.
    """
    if any(math.isnan(v) for v in (lateral_sway_std_deg, step_time_cv, double_support_frac)):
        return float("nan")
    c1 = min(100.0, 100.0 * lateral_sway_std_deg / 10.0)
    c2 = min(100.0, 100.0 * step_time_cv / 0.5)
    c3 = min(100.0, 100.0 * double_support_frac)
    return (c1 + c2 + c3) / 3.0
