#!/usr/bin/env python3
"""Lectura minima de los bags de datasets/labeled_bags (soma13kp) para evaluar
walker_step_detector contra ground truth de mocap.

No forma parte del build de colcon: son herramientas offline de analisis,
pensadas para ejecutarse con `python3 scripts/xxx.py` tras sourcear
/opt/ros/jazzy/setup.bash (necesitan rosbag2_py/rclpy para deserializar).

Cada bag trae, entre otros topics:
  /scan             sensor_msgs/msg/LaserScan   frame_id="laser"
  /labeled_markers  mocap4r2_msgs/msg/Markers   frame_id="map", marker_name
                     en soma_net.schema.KEYPOINTS (l_ankle, r_ankle, l_knee,
                     r_knee, ...)
  /rigid_bodies     mocap4r2_msgs/msg/RigidBodies  rigidbodies[0] = el andador
                     (ver bag_checker/unify_bags.py), pose en frame "map"

No hay map->base_footprint en /tf: hay que combinar rigid_bodies[0].pose con
una calibracion fija rigid_body->base_footprint (ver calibrate_mocap_to_laser.py)
y con base_footprint->laser (fijo, del xacro de walker_description).
"""
import math
import re
from pathlib import Path

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# base_footprint -> laser (walker_description/urdf/walker_model.xacro:17-19,
# joint fijo rpy=0 0 0): valores por defecto de lidar_pos_{x,y,z}_prop.
LASER_IN_BASE_FOOTPRINT = (0.55, 0.0, 0.21)

FOOT_KEYS = ("l_ankle", "r_ankle")
LEG_KEYS = ("l_ankle", "r_ankle", "l_knee", "r_knee")


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
    """soma13kp/bag_labeler/label_rosbag.py:193-202 escribe la entrada de
    topic_metadata de /labeled_markers con offered_qos_profiles como el
    string literal "[]" (en vez de una lista YAML) y sin type_description_hash,
    a diferencia del resto de topics del fichero -- rompe el parser de
    rosbag2_py/ros2 bag en cualquier bag que ese script haya tocado. Es un
    bug del otro subproyecto (etiquetado SOMA-13KP), no de aqui, y esa tarea
    sigue corriendo en paralelo regenerando bags -- CA_test01 volvio a salir
    corrupto tras haberlo reparado antes en esta misma sesion. Hasta que se
    arregle alli, reparamos la copia de metadata.yaml in-place (solo texto,
    nunca el .db3) para poder leer estos bags con las herramientas estandar
    de ROS2 (rosbag2_py Y el binario `ros2 bag play`, que no pasa por este
    modulo -- quien lance `ros2 bag play` directamente debe llamar a esto
    primero, ver run_multibag_eval.py).
    """
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


def load_bag(bag_dir, topics=("/scan", "/rigid_bodies", "/labeled_markers"), require_all=True):
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
    if not t_list:
        return None
    best = min(t_list, key=lambda x: abs(x[0] - t_query))
    if max_dt_ns is not None and abs(best[0] - t_query) > max_dt_ns:
        return None
    return best


def rigid_body_pose_xy_yaw(rb_msg, index=0):
    """rigidbodies[index].pose -> (x, y, yaw) en frame map. index=0 es el andador."""
    pose = rb_msg.rigidbodies[index].pose
    x, y = pose.position.x, pose.position.y
    q = pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return x, y, yaw


def markers_by_name(markers_msg, names=LEG_KEYS):
    out = {}
    for m in markers_msg.markers:
        if m.marker_name in names:
            out[m.marker_name] = (m.translation.x, m.translation.y, m.translation.z)
    return out


def scan_to_cartesian(scan_msg, max_range=3.0):
    """Devuelve lista (x, y) en frame laser, rangos validos y < max_range."""
    pts = []
    a = scan_msg.angle_min
    rmin, rmax = scan_msg.range_min, scan_msg.range_max
    for r in scan_msg.ranges:
        if rmin < r < rmax and r < max_range:
            pts.append((math.cos(a) * r, math.sin(a) * r))
        a += scan_msg.angle_increment
    return pts


def load_calib(path):
    """config/mocap_to_base_footprint.yaml -> (dx, dy, dyaw), del ICP de
    calibrate_mocap_to_laser.py."""
    with open(path) as f:
        d = yaml.safe_load(f)
    return float(d["dx"]), float(d["dy"]), float(d["dyaw"])


def project_map_to_laser(px, py, rb_x, rb_y, rb_yaw, calib):
    """Proyecta un punto (px, py) en frame "map" (p.ej. un marcador de
    /labeled_markers) al frame "laser", combinando:
      - rigid_body[0].pose (rb_x, rb_y, rb_yaw): andador en frame map
      - calib = (dx, dy, dyaw): rigid_body -> base_footprint (ICP, ver
        calibrate_mocap_to_laser.py / config/mocap_to_base_footprint.yaml)
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
