"""Metricas de marcha (Step/Stride time y length, NoS, cadencia...) a partir de eventos de apoyo.

La implementacion vive en el paquete mocap_loads (mocap4w/mocap_loads), junto con la comparacion de walker_loads; este
modulo la reexporta para no cambiar los imports de los scripts.
"""
from mocap_loads.gait_metrics import *  # noqa: F401,F403
from mocap_loads.gait_metrics import LegLog, process_events, summarize, update_stats  # noqa: F401
