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
  (El margen de estabilidad -- CoM real frente al poligono de apoyo real de
  los pies en apoyo y las manetas -- ya no esta aqui: lo calcula
  mocap_stability, el par mocap de walker_stability, y
  compute_posture_from_mocap.py lo usa. La funcion que habia aqui,
  lateral_margin_at_event, validaba la rampa lateral del modelo antiguo de
  walker_stability, que ya no existe.)
"""
import math

import numpy as np

# Movidos a mocap_loads (parametros de marcha derivados de los eventos de apoyo); se reexportan aqui.
from mocap_loads.gait_params import _rotate_to_walker_frame, double_support_fraction, step_width_m  # noqa: F401


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
