"""Deteccion de eventos de apoyo por pie a partir de trayectorias del mocap.

La implementacion vive en el paquete mocap_step_detector (mocap4w/mocap_step_detector), donde esta todo lo que se
obtiene de los pasos del mocap; este modulo la reexporta para no cambiar los imports de los scripts.
"""
from mocap_step_detector.contact_events import *  # noqa: F401,F403
from mocap_step_detector.contact_events import detect_stance_events, horizontal_speeds  # noqa: F401
