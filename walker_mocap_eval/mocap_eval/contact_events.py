#!/usr/bin/env python3
"""Deteccion de eventos de apoyo (stance) por pie a partir de trayectorias
mocap, para poder alimentar gait_metrics.py con algo equivalente al flanco
de subida de walker_msgs/StepStamped.load (>0) que usa
walker_loads/gait_monitor_speed.py -- el mocap no mide carga, asi que aqui
se aproxima el instante de apoyo con la cinematica: durante la fase de
apoyo el marcador del tobillo esta (casi) quieto respecto al suelo,
mientras que en el balanceo se mueve rapido. Se busca, por cada episodio de
velocidad por debajo de un umbral, el instante de velocidad minima.

No pretende ser un detector de contacto de precision submilimetrica (para
eso se usaria la componente vertical y un umbral de altura, si el mocap
fuese fiable en Z); es la aproximacion mas simple que permite
recalcular Step/Stride time y length en el mismo formato que publica
gait_monitor_speed.py, para poder compararlos (punto 2 del plan).
"""
import math

import numpy as np


def horizontal_speeds(track):
    """Velocidad horizontal (m/s) en cada muestra interior de `track`
    ([(t_ns, (x,y,z))]), por diferencias centradas. Devuelve una lista
    alineada con track[1:-1] (una velocidad menos por cada extremo)."""
    speeds = []
    for i in range(1, len(track) - 1):
        t0, p0 = track[i - 1]
        t1, p1 = track[i + 1]
        dt = (t1 - t0) / 1e9
        if dt <= 0:
            speeds.append(float("nan"))
            continue
        d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        speeds.append(d / dt)
    return speeds


def detect_stance_events(track, speed_threshold=0.25, min_gap_s=0.3):
    """track: [(t_ns, (x,y,z))] de un marcador de tobillo, en frame map,
    ordenado por tiempo (ver bag_io.marker_track).

    Devuelve una lista de eventos [(t_ns, (x,y,z))], uno por episodio en el
    que la velocidad horizontal cae por debajo de `speed_threshold`
    (m/s) -- el evento se situa en el instante de velocidad minima de ese
    episodio, analogo al instante en que walker_msgs/StepStamped.load pasa
    a ser >0 en el nodo real.

    `min_gap_s` descarta un nuevo evento si esta a menos de ese tiempo del
    anterior ya aceptado, para no duplicar un mismo apoyo partido en dos
    episodios por ruido puntual del etiquetado.
    """
    if len(track) < 3:
        return []

    speeds = horizontal_speeds(track)
    n = len(speeds)
    events = []
    i = 0
    while i < n:
        if not (speeds[i] < speed_threshold):
            i += 1
            continue
        j = i
        while j < n and speeds[j] < speed_threshold:
            j += 1
        episode = speeds[i:j]
        best_k = i + int(np.nanargmin(episode))
        # speeds[k] esta calculado entre track[k] y track[k+2] -> el
        # instante correspondiente en track es k+1.
        t_evt, p_evt = track[best_k + 1]
        if not events or (t_evt - events[-1][0]) / 1e9 >= min_gap_s:
            events.append((t_evt, p_evt))
        i = j
    return events


def detect_stance_intervals(track, speed_threshold=0.25, min_duration_s=0.05):
    """Igual que detect_stance_events(), pero devuelve el episodio completo
    de baja velocidad como intervalo [(t_start_ns, t_end_ns), ...] en vez de
    solo su instante de velocidad minima -- lo que hace falta para calcular
    tiempo de doble apoyo (solape entre intervalos de ambos pies), que
    detect_stance_events() no puede dar por si solo.

    `min_duration_s` descarta episodios mas cortos que eso (ruido puntual
    del etiquetado que cruza el umbral un instante sin ser un apoyo real).
    """
    if len(track) < 3:
        return []

    speeds = horizontal_speeds(track)
    n = len(speeds)
    intervals = []
    i = 0
    while i < n:
        if not (speeds[i] < speed_threshold):
            i += 1
            continue
        j = i
        while j < n and speeds[j] < speed_threshold:
            j += 1
        # speeds[k] corresponde al instante track[k+1] (ver detect_stance_events).
        t_start = track[i + 1][0]
        t_end = track[j][0]
        if (t_end - t_start) / 1e9 >= min_duration_s:
            intervals.append((t_start, t_end))
        i = j
    return intervals
