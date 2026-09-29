#!/usr/bin/env python3
"""Parametros posturales/de estabilidad derivables del mocap etiquetado,
potencialmente correlables con datos del andador (paso 3 del plan de
validacion -- ver walker_mocap_eval/readme.md).

Reutiliza la misma base que gait_metrics.py/contact_events.py (punto 1):
bag_io.py ya expone los 13 keypoints etiquetados (no solo tobillos), y los
eventos/intervalos de apoyo de contact_events.py sirven tanto para marcha
como para estos parametros posturales, sin duplicar deteccion.

Parametros implementados aqui (ver la tabla del plan para el resto):
  - Inclinacion de tronco (total, sagital -fore/aft-, frontal -lateral-):
    candidato a correlar con el reparto de carga manetas/pies (walker_loads/
    partial_loads) y con el centroide de soporte (walker_centroid_support).
  - Ancho de paso: no publicado hoy por ningun nodo del andador, pero
    derivable de las mismas posiciones de pie que ya usa el detector de
    pasos -- candidato "barato" (misma fuente que StepStamped.position).
  - Tiempo de doble apoyo (% del ciclo): idem, derivable de gait_monitor_
    speed.py sin sensorica nueva (is_left_load/is_right_load ya existen
    ahi, solo no se publican).
  - Margen de estabilidad lateral real (CoM real vs BoS real de tobillos):
    validacion directa de walker_stability.py, que hoy estima el BoS con
    tablas antropometricas en vez de medirlo.
"""
import math

import numpy as np


def midpoint(p1, p2):
    return ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0, (p1[2] + p2[2]) / 2.0)


def _angle_from_vertical_deg(vx, vy, vz):
    v = np.array([vx, vy, vz], dtype=float)
    n = np.linalg.norm(v)
    if n == 0:
        return float("nan")
    cos_a = np.clip(v[2] / n, -1.0, 1.0)
    return math.degrees(math.acos(cos_a))


def trunk_lean_at_frame(l_shoulder, r_shoulder, l_hip, r_hip):
    """Inclinacion de tronco en un instante: angulo (grados) entre el
    vector cadera_media->hombro_medio y la vertical (eje Z). 0 = de pie
    recto; aumenta con la inclinacion, sea hacia delante, atras o los
    lados. Tambien devuelve la componente sagital (fore/aft, signo + hacia
    delante en X) y frontal (lateral, signo + hacia la derecha en Y) via
    atan2, en grados, utiles por separado para correlar con asimetria de
    carga fore-aft vs izquierda-derecha.

    Devuelve (total_deg, sagittal_deg, frontal_deg).
    """
    shoulder_mid = midpoint(l_shoulder, r_shoulder)
    hip_mid = midpoint(l_hip, r_hip)
    v = (shoulder_mid[0] - hip_mid[0], shoulder_mid[1] - hip_mid[1], shoulder_mid[2] - hip_mid[2])

    total = _angle_from_vertical_deg(*v)
    sagittal = math.degrees(math.atan2(v[0], v[2])) if v[2] != 0 else float("nan")
    frontal = math.degrees(math.atan2(v[1], v[2])) if v[2] != 0 else float("nan")
    return total, sagittal, frontal


def trunk_lean_series(l_shoulder_track, r_shoulder_track, l_hip_track, r_hip_track, max_dt_ns):
    """Series de tiempo (una por marcador, ver bag_io.marker_track) ->
    [(t_ns, total_deg, sagittal_deg, frontal_deg)], emparejando cada
    muestra de l_shoulder con la mas cercana (dentro de max_dt_ns) de los
    otros tres marcadores. El etiquetado puede tener huecos por marcador,
    de ahi el emparejado por tiempo en vez de asumir frames alineados."""
    from . import bag_io

    # indexed_nearest() en vez de nearest(): esto consulta los 3 tracks una
    # vez por cada muestra de l_shoulder (miles a ~120Hz) -- con nearest()
    # (escaneo lineal) es O(n^2) y tarda minutos por bag.
    r_sh_q = bag_io.indexed_nearest(r_shoulder_track)
    l_hip_q = bag_io.indexed_nearest(l_hip_track)
    r_hip_q = bag_io.indexed_nearest(r_hip_track)

    out = []
    for t, l_sh in l_shoulder_track:
        r_sh = r_sh_q(t, max_dt_ns)
        l_hip = l_hip_q(t, max_dt_ns)
        r_hip = r_hip_q(t, max_dt_ns)
        if r_sh is None or l_hip is None or r_hip is None:
            continue
        total, sag, fro = trunk_lean_at_frame(l_sh, r_sh[1], l_hip[1], r_hip[1])
        out.append((t, total, sag, fro))
    return out


def _rotate_to_walker_frame(dx, dy, walker_yaw):
    """Rota un vector (dx, dy) del frame map al frame local del andador
    (heading = walker_yaw), para que la componente Y resultante sea
    "lateral" (izquierda/derecha del andador) y la X sea "longitudinal"
    (avance), sin depender de como este orientada la sala."""
    c, s = math.cos(-walker_yaw), math.sin(-walker_yaw)
    return c * dx - s * dy, s * dx + c * dy


def step_width_m(left_events, right_events, walker_track, max_pair_dt_ns, max_rb_dt_ns):
    """Ancho de paso: distancia lateral (perpendicular al avance del
    andador) entre cada evento de apoyo de un pie y el evento del pie
    contrario mas cercano en el tiempo (misma logica de emparejado que
    gait_metrics usa para Step length, pero mirando el eje lateral en vez
    de la distancia total).

    left_events/right_events: [(t_ns, (x,y,z))] en frame "map" (SIN
    proyectar al frame del andador -- aqui se hace directamente con
    walker_yaw, no hace falta la calibracion mocap->laser porque un ancho
    de paso es una distancia relativa, no una posicion absoluta).
    walker_track: bag_io.rigid_body_track_xy_yaw(rb_msgs) (para el yaw).

    Devuelve [(t_ns, width_m)], uno por cada evento (de cualquier pie) que
    tuviera un evento del pie contrario dentro de max_pair_dt_ns.
    """
    from . import bag_io

    out = []
    for own_events, opp_events in ((left_events, right_events), (right_events, left_events)):
        for t, p in own_events:
            opp = bag_io.nearest(opp_events, t, max_pair_dt_ns)
            rb = bag_io.nearest(walker_track, t, max_rb_dt_ns)
            if opp is None or rb is None:
                continue
            yaw = rb[1][2]
            dx, dy = p[0] - opp[1][0], p[1] - opp[1][1]
            _, lateral = _rotate_to_walker_frame(dx, dy, yaw)
            out.append((t, abs(lateral)))
    out.sort(key=lambda r: r[0])
    return out


def double_support_fraction(left_intervals, right_intervals):
    """Fraccion (0-1) del tiempo total (desde el primer inicio de intervalo
    hasta el ultimo final, de cualquier pie) en que AMBOS pies estan en
    apoyo simultaneamente (solape entre un intervalo izquierdo y uno
    derecho). Ver contact_events.detect_stance_intervals()."""
    if not left_intervals or not right_intervals:
        return float("nan")

    overlap = 0
    for ls, le in left_intervals:
        for rs, re in right_intervals:
            lo = max(ls, rs)
            hi = min(le, re)
            if hi > lo:
                overlap += hi - lo

    t_start = min(left_intervals[0][0], right_intervals[0][0])
    t_end = max(left_intervals[-1][1], right_intervals[-1][1])
    total = t_end - t_start
    if total <= 0:
        return float("nan")
    return overlap / total


def lateral_margin_at_event(t, hip_mid_track, l_ankle_track, r_ankle_track, walker_track,
                             max_dt_ns, max_rb_dt_ns):
    """Margen de estabilidad lateral REAL (0-1) en el instante t: posicion
    del punto medio de caderas (proxy de CoM) proyectada sobre el eje
    lateral del andador, normalizada entre las posiciones de los dos
    tobillos en ese mismo eje -- 0 en el borde de un pie o mas alla
    (inestable), 1 en el centro exacto entre ambos pies. Misma formula que
    walker_stability.py::centroid_callback() usa para su "y_sta", pero con
    el BoS REAL (tobillos del mocap) en vez de la anchura de manetas
    estimada por tablas antropometricas -- pensado para validar esa
    formula, no para sustituirla.

    Devuelve nan si falta algun marcador dentro de la tolerancia temporal."""
    from . import bag_io

    hip = bag_io.nearest(hip_mid_track, t, max_dt_ns)
    la = bag_io.nearest(l_ankle_track, t, max_dt_ns)
    ra = bag_io.nearest(r_ankle_track, t, max_dt_ns)
    rb = bag_io.nearest(walker_track, t, max_rb_dt_ns)
    if hip is None or la is None or ra is None or rb is None:
        return float("nan")

    yaw = rb[1][2]
    hip_p, la_p, ra_p = hip[1], la[1], ra[1]

    _, hip_lat = _rotate_to_walker_frame(hip_p[0], hip_p[1], yaw)
    _, la_lat = _rotate_to_walker_frame(la_p[0], la_p[1], yaw)
    _, ra_lat = _rotate_to_walker_frame(ra_p[0], ra_p[1], yaw)

    lo, hi = min(la_lat, ra_lat), max(la_lat, ra_lat)
    if hi - lo <= 0:
        return float("nan")
    margin = (hip_lat - lo) / (hi - lo)
    return float(np.clip(margin, 0.0, 1.0))
