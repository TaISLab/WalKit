#!/usr/bin/env python3
"""Reimplementacion, en funciones puras (sin rclpy), del algoritmo de
walker_loads/walker_loads/scripts/gait_monitor_speed.py (nodo
`gait_monitor_speed`), para poder correrlo tanto sobre eventos en vivo
(StepStamped) como sobre eventos reconstruidos desde mocap
(contact_events.py) y comparar ambos (punto 2 del plan).

Replica deliberadamente, campo a campo, lo que hace GaitMonitorSp:
  - update_stats(): stride_time/length (mismo pie, eventos consecutivos) y
    step_time/length (pie contrario, evento mas reciente conocido), exactos
    a left_loads_cb/right_loads_cb + update_stats() del nodo.
  - summarize(): medias acumuladas desde el inicio (nanmean sobre todo el
    historial, igual que timer_callback()), Tr = desde el primer evento de
    cualquier pierna hasta el ultimo de cualquiera, NoS, CAD, WV -- mismos
    nombres de campo que walker_msgs/GaitStamped y walker_msgs/WalkStats
    (spt/sdt/spl/sdl, tr/nos/d/cad/wv) para que un futuro script de
    comparacion pueda cruzarlos 1:1.

Diferencia deliberada con el nodo real: `d` (distancia) no se calcula aqui
-- se recibe ya calculada (ver bag_io.arclength_m) porque
gait_monitor_speed.py NO usa su propio self.travelled (acumulado real desde
/odom) sino un valor de get_TravelledDist() con una constante de prueba
hardcodeada (test_time=14.48s, 8m, resultado capado a 8m). Ver README de
este paquete: es una discrepancia a tener en cuenta en el punto 2, no un
detalle que debamos reproducir "a proposito" en la referencia de mocap.

`get_difference()`/get_distance() tampoco transforma a un frame global
comun entre los dos instantes comparados (el propio codigo del nodo lo dice
explicitamente: "We don't need to cast positions to global frame"): calcula
distancia euclidea directa entre las dos posiciones tal como llegan. Aqui
hacemos lo mismo a proposito -- si las posiciones que se le pasan ya vienen
proyectadas al frame del andador en su propio instante (como hace
compute_gait_from_mocap.py via bag_io.project_map_to_laser), el resultado
es comparable al que publica el nodo real, con el mismo sesgo si lo hay.
"""
import math

import numpy as np


def _distance(xy1, xy2):
    return math.hypot(xy1[0] - xy2[0], xy1[1] - xy2[1])


class LegLog:
    """Historial de eventos de apoyo de una pierna, y las series derivadas
    que gait_monitor_speed.py acumula en self.left_/right_* ."""

    def __init__(self, name):
        self.name = name
        self.events = []  # [(t_ns, (x, y))]
        self.stride_times = []
        self.stride_lengths = []
        self.step_times = []
        self.step_lengths = []


def update_stats(t_ns, xy, own: LegLog, opposite: LegLog):
    """Puerto directo de GaitMonitorSp.update_stats(): se llama una vez por
    cada evento de apoyo de `own`, con acceso al historial ya visto de
    `opposite` en ese instante (igual que el callback del nodo real solo ve
    lo publicado hasta el momento en el otro topic)."""
    own.events.append((t_ns, xy))

    if len(own.events) > 1:
        (t_prev, xy_prev) = own.events[-2]
        (t_cur, xy_cur) = own.events[-1]
        own.stride_times.append((t_cur - t_prev) / 1e9)
        own.stride_lengths.append(_distance(xy_cur, xy_prev))

    if len(opposite.events) > 0:
        (t_opp, xy_opp) = opposite.events[-1]
        (t_cur, xy_cur) = own.events[-1]
        own.step_times.append((t_cur - t_opp) / 1e9)
        own.step_lengths.append(_distance(xy_cur, xy_opp))


def process_events(events):
    """events: iterable de (t_ns, leg, (x, y)) con leg en {"left","right"},
    en CUALQUIER orden -- se ordenan aqui por t_ns para garantizar el mismo
    orden de procesado cronologico que verian los callbacks de ROS.

    Devuelve (left_log, right_log)."""
    left_log = LegLog("left")
    right_log = LegLog("right")

    for t_ns, leg, xy in sorted(events, key=lambda e: e[0]):
        if leg == "left":
            update_stats(t_ns, xy, left_log, right_log)
        elif leg == "right":
            update_stats(t_ns, xy, right_log, left_log)
        else:
            raise ValueError(f"leg desconocida: {leg!r} (se esperaba 'left'/'right')")

    return left_log, right_log


def _nanmean_or_nan(values):
    return float(np.nanmean(values)) if values else float("nan")


def _avg_speed(lengths, times):
    """Media de (longitud_i / tiempo_i), no longitud_media/tiempo_medio --
    evita el sesgo de una razon de medias. No es un campo publicado por el
    nodo real (GaitStamped/WalkStats no tienen "step speed"), pero si forma
    parte de la especificacion original de gait_monitor_speed (ver
    docstring de GaitMonitorSp): "Average step speed (left/right/all)"."""
    ratios = [l / t for l, t in zip(lengths, times) if t and t > 0]
    return _nanmean_or_nan(ratios)


def summarize(left_log: LegLog, right_log: LegLog, distance_m):
    """Puerto de GaitMonitorSp.timer_callback(): misma condicion de guarda
    (>2 eventos en AMBAS piernas) y mismas formulas para Tr/NoS/CAD/WV.
    `distance_m` es la distancia recorrida en la ventana [Tr_start, Tr_end]
    (ver bag_io.arclength_m + clip_track para la version mocap).

    Devuelve None si aun no hay datos suficientes (como el nodo real, que
    simplemente no publica todavia), o un dict con las mismas claves que
    walker_msgs/GaitStamped (prefijadas left_/right_) y
    walker_msgs/WalkStats, mas avg_step_speed_{left,right,all} (derivado,
    no publicado por el nodo)."""
    if len(left_log.events) <= 2 or len(right_log.events) <= 2:
        return None

    l_spt = _nanmean_or_nan(left_log.step_times)
    l_sdt = _nanmean_or_nan(left_log.stride_times)
    l_nos = len(left_log.events)
    l_spl = _nanmean_or_nan(left_log.step_lengths)
    l_sdl = _nanmean_or_nan(left_log.stride_lengths)

    r_spt = _nanmean_or_nan(right_log.step_times)
    r_sdt = _nanmean_or_nan(right_log.stride_times)
    r_nos = len(right_log.events)
    r_spl = _nanmean_or_nan(right_log.step_lengths)
    r_sdl = _nanmean_or_nan(right_log.stride_lengths)

    nos = l_nos + r_nos

    # Tr: desde el primer evento de cualquiera de las dos piernas hasta el
    # ultimo de cualquiera de las dos (igual que get_secs_diff aplicado dos
    # veces en el nodo real para elegir start/end_stamp).
    t_start = min(left_log.events[0][0], right_log.events[0][0])
    t_end = max(left_log.events[-1][0], right_log.events[-1][0])
    tr_s = (t_end - t_start) / 1e9

    cad = 60.0 * nos / tr_s if tr_s > 0 else float("nan")
    wv = distance_m / tr_s if tr_s > 0 else float("nan")

    all_lengths = left_log.step_lengths + right_log.step_lengths
    all_times = left_log.step_times + right_log.step_times

    return {
        "left_spt": l_spt, "left_sdt": l_sdt, "left_nos": l_nos,
        "left_spl": l_spl, "left_sdl": l_sdl,
        "right_spt": r_spt, "right_sdt": r_sdt, "right_nos": r_nos,
        "right_spl": r_spl, "right_sdl": r_sdl,
        "tr_s": tr_s, "nos": nos, "d_m": distance_m, "cad": cad, "wv": wv,
        "avg_step_speed_left": _avg_speed(left_log.step_lengths, left_log.step_times),
        "avg_step_speed_right": _avg_speed(right_log.step_lengths, right_log.step_times),
        "avg_step_speed_all": _avg_speed(all_lengths, all_times),
    }
