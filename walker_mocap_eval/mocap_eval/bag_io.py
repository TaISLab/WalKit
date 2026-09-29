#!/usr/bin/env python3
"""Lectura generica de los bags etiquetados de datasets/labeled_bags
(soma13kp) y proyeccion de puntos mocap (frame "map") al frame del andador.

Version generalizada de walker_step_detector/scripts/mocap_bag_io.py: ese
modulo vive dentro de walker_step_detector y esta pensado para sus propios
scripts de evaluacion del detector de pasos (frame "laser", solo
FOOT_KEYS/LEG_KEYS). Aqui reproducimos las mismas funciones de bajo nivel
(mismo formato de bag, mismo bug conocido de metadata.yaml, misma
calibracion mocap->base_footprint) para que walker_mocap_eval no dependa de
la ruta relativa de otro paquete, y podamos pedir cualquier marcador
etiquetado (no solo pies), pensando en los estudios futuros del plan
(inclinacion de tronco, angulos de codo, etc.).

Como con el modulo original: no forma parte del build de colcon. Son
herramientas offline de analisis, para ejecutarse con
`python3 scripts/xxx.py` tras sourcear /opt/ros/jazzy/setup.bash (necesitan
rosbag2_py/rclpy para deserializar).
"""
import bisect
import math
import re
from pathlib import Path

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# base_footprint -> laser (walker_description/urdf/walker_model.xacro:17-19,
# joint fijo rpy=0 0 0): valores por defecto de lidar_pos_{x,y,z}_prop.
# Igual que en walker_step_detector/scripts/mocap_bag_io.py.
LASER_IN_BASE_FOOTPRINT = (0.55, 0.0, 0.21)

FOOT_KEYS = ("l_ankle", "r_ankle")
LEG_KEYS = ("l_ankle", "r_ankle", "l_knee", "r_knee")

# Los 13 keypoints de soma_net.schema.KEYPOINTS, copiados literalmente para
# no depender del paquete soma13kp (no es una dependencia de WalKit). Si el
# esquema cambia alli, hay que actualizarlo aqui tambien.
ALL_KEYPOINTS = (
    "torso",
    "l_shoulder", "r_shoulder",
    "l_elbow", "r_elbow",
    "l_wrist", "r_wrist",
    "l_hip", "r_hip",
    "l_knee", "r_knee",
    "l_ankle", "r_ankle",
)


def find_labeled_bags(root):
    root = Path(root)
    return sorted(p.parent for p in root.glob("*/metadata.yaml"))


def bag_duration_seconds(bag_dir):
    """Duracion grabada del bag (metadata.yaml), sin necesidad de abrirlo."""
    meta_path = Path(bag_dir) / "metadata.yaml"
    with open(meta_path) as f:
        d = yaml.safe_load(f)
    return d["rosbag2_bagfile_information"]["duration"]["nanoseconds"] / 1e9


def _read_topic(reader, type_map, wanted_topics):
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(wanted_topics)))
    out = {t: [] for t in wanted_topics}
    while reader.has_next():
        topic, data, t = reader.read_next()
        msg = deserialize_message(data, get_message(type_map[topic]))
        out[topic].append((t, msg))
    return out


def repair_bag_metadata(bag_dir):
    """soma13kp/bag_labeler/label_rosbag.py escribe topic_metadata de
    /labeled_markers con offered_qos_profiles como el string literal "[]" y
    sin type_description_hash -- rompe rosbag2_py/`ros2 bag` en cualquier
    bag etiquetado. Bug del otro subproyecto, se repara aqui in-place (solo
    el .yaml de texto, nunca el .db3), igual que en
    walker_step_detector/scripts/mocap_bag_io.py."""
    meta_path = Path(bag_dir) / "metadata.yaml"
    text = meta_path.read_text()
    fixed = text.replace("offered_qos_profiles: '[]'", "offered_qos_profiles: []")
    fixed = re.sub(
        r"(name: /labeled_markers\n\s+type: mocap4r2_msgs/msg/Markers\n"
        r"\s+serialization_format: cdr\n\s+offered_qos_profiles: \[\]\n)"
        r"(?!\s+type_description_hash)",
        r"\1      type_description_hash: ''\n",
        fixed,
    )
    if fixed != text:
        meta_path.write_text(fixed)


def _storage_id(bag_dir):
    """labeled_bags (soma13kp) son sqlite3; un bag grabado con `ros2 bag
    record` hoy en dia por defecto es mcap -- se lee del propio
    metadata.yaml en vez de asumir uno de los dos."""
    meta_path = Path(bag_dir) / "metadata.yaml"
    with open(meta_path) as f:
        d = yaml.safe_load(f)
    return d["rosbag2_bagfile_information"].get("storage_identifier", "sqlite3")


def _open_reader(bag_dir):
    bag_dir = str(bag_dir)
    repair_bag_metadata(bag_dir)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id=_storage_id(bag_dir)),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def bag_topics(bag_dir):
    """dict topic -> tipo, sin leer mensajes (para autodescubrir topics)."""
    reader = _open_reader(bag_dir)
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def load_bag(bag_dir, topics=("/labeled_markers", "/rigid_bodies"), require_all=True):
    """Devuelve dict topic -> [(t_ns, msg), ...] ordenado por tiempo de bag.
    Si require_all es False, los topics ausentes simplemente no aparecen en
    el dict devuelto (en vez de lanzar ValueError)."""
    reader = _open_reader(bag_dir)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    present = [t for t in topics if t in type_map]
    missing = [t for t in topics if t not in type_map]
    if missing and require_all:
        raise ValueError(f"{bag_dir}: faltan topics {missing}")
    return _read_topic(reader, type_map, present)


def nearest(t_list, t_query, max_dt_ns=None):
    """O(n) por llamada (escaneo lineal) -- bien para una consulta suelta,
    pero NO para buscar repetidamente contra el mismo t_list (p.ej. una
    consulta por cada muestra de otro marcador, con miles de muestras a
    ~120Hz): en ese caso usar indexed_nearest(t_list) una vez y llamar al
    callable que devuelve, que es O(log n) por consulta via bisect."""
    if not t_list:
        return None
    best = min(t_list, key=lambda x: abs(x[0] - t_query))
    if max_dt_ns is not None and abs(best[0] - t_query) > max_dt_ns:
        return None
    return best


def indexed_nearest(t_list):
    """Indexa t_list (debe venir ordenado por tiempo, como devuelven
    marker_track()/load_bag()) una sola vez con bisect, y devuelve una
    funcion query(t_query, max_dt_ns=None) equivalente a nearest(t_list,
    ...) pero O(log n) por consulta en vez de O(n) -- necesario cuando se
    hacen muchas consultas contra el mismo track (p.ej.
    posture_metrics.trunk_lean_series(), que consulta 3 tracks una vez por
    cada muestra de otro, miles de veces por bag: con nearest() eso es
    O(n^2) y tarda minutos por bag; con esto, segundos -- el resto es
    deserializar los mensajes del bag, no busquedas)."""
    times = [t for t, _ in t_list]

    def query(t_query, max_dt_ns=None):
        if not times:
            return None
        i = bisect.bisect_left(times, t_query)
        candidates = [k for k in (i - 1, i) if 0 <= k < len(times)]
        if not candidates:
            return None
        best_i = min(candidates, key=lambda k: abs(times[k] - t_query))
        if max_dt_ns is not None and abs(times[best_i] - t_query) > max_dt_ns:
            return None
        return t_list[best_i]

    return query


def rigid_body_pose_xy_yaw(rb_msg, index=0):
    """rigidbodies[index].pose -> (x, y, yaw) en frame map. index=0 es el andador."""
    pose = rb_msg.rigidbodies[index].pose
    x, y = pose.position.x, pose.position.y
    q = pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return x, y, yaw


def rigid_body_track_xy(rb_msgs, index=0):
    """[(t_ns, (x, y))] de rigidbodies[index].pose, en frame map, sin filtrar."""
    out = []
    for t, msg in rb_msgs:
        p = msg.rigidbodies[index].pose.position
        out.append((t, (p.x, p.y)))
    return out


def rigid_body_track_xy_yaw(rb_msgs, index=0):
    """[(t_ns, (x, y, yaw))] de rigidbodies[index].pose, en frame map, sin
    filtrar -- para lo que necesite el heading del andador en cada
    instante (p.ej. posture_metrics.py, para proyectar posiciones al eje
    lateral/longitudinal del andador), a diferencia de
    rigid_body_track_xy() que solo da posicion (suficiente para
    arclength_m())."""
    out = []
    for t, msg in rb_msgs:
        x, y, yaw = rigid_body_pose_xy_yaw(msg, index)
        out.append((t, (x, y, yaw)))
    return out


def arclength_m(xy_track):
    """Distancia recorrida = suma de desplazamientos entre muestras
    consecutivas (equivalente mocap de walker_loads/gait_monitor_speed.py:
    self.travelled, acumulado a partir de /odom -- pero medido con el rigid
    body del andador en el mocap en vez de la odometria de las ruedas)."""
    d = 0.0
    for (_, p0), (_, p1) in zip(xy_track, xy_track[1:]):
        d += math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    return d


def clip_track(track, t_start_ns, t_end_ns):
    return [(t, p) for t, p in track if t_start_ns <= t <= t_end_ns]


def marker_by_name(markers_msg, name):
    for m in markers_msg.markers:
        if m.marker_name == name:
            return (m.translation.x, m.translation.y, m.translation.z)
    return None


def markers_by_name(markers_msg, names=LEG_KEYS):
    out = {}
    for m in markers_msg.markers:
        if m.marker_name in names:
            out[m.marker_name] = (m.translation.x, m.translation.y, m.translation.z)
    return out


def marker_track(mk_msgs, name):
    """[(t_ns, (x, y, z))] de un marcador concreto, en frame map, solo en
    los frames donde esta presente (el etiquetado puede tener huecos)."""
    out = []
    for t, msg in mk_msgs:
        p = marker_by_name(msg, name)
        if p is not None:
            out.append((t, p))
    return out


def load_calib(path):
    """config/mocap_to_base_footprint.yaml -> (dx, dy, dyaw), del ICP de
    walker_step_detector/scripts/calibrate_mocap_to_laser.py. Reutilizamos
    ese fichero (fuente unica de la calibracion), no lo duplicamos aqui."""
    with open(path) as f:
        d = yaml.safe_load(f)
    return float(d["dx"]), float(d["dy"]), float(d["dyaw"])


def project_map_to_laser(px, py, rb_x, rb_y, rb_yaw, calib):
    """Proyecta un punto (px, py) en frame "map" (p.ej. un marcador de
    /labeled_markers) al frame "laser", combinando:
      - rigid_body[0].pose (rb_x, rb_y, rb_yaw): andador en frame map
      - calib = (dx, dy, dyaw): rigid_body -> base_footprint (ICP, ver
        walker_step_detector/config/mocap_to_base_footprint.yaml)
      - LASER_IN_BASE_FOOTPRINT: base_footprint -> laser (fijo, xacro)
    """
    dx, dy, dyaw = calib
    c, s = math.cos(rb_yaw), math.sin(rb_yaw)
    bx = rb_x + c * dx - s * dy
    by = rb_y + s * dx + c * dy
    byaw = rb_yaw + dyaw
    c2, s2 = math.cos(-byaw), math.sin(-byaw)
    lx_bf = c2 * (px - bx) - s2 * (py - by)
    ly_bf = s2 * (px - bx) + c2 * (py - by)
    ox, oy, _ = LASER_IN_BASE_FOOTPRINT
    return lx_bf - ox, ly_bf - oy
