#!/usr/bin/env python3
"""Nombres con los que los scripts de walker_mocap_eval leen los bags y trabajan con pistas del mocap.

Ya no hay implementacion aqui: leer un bag es de `walker_replay.bagio` y lo que se hace con los mensajes del mocap
(pistas de rigid bodies y marcadores, el mas cercano en el tiempo, proyeccion `map` -> laser) es de
`mocap_compare_common.mocap_tracks` (mocap4w). Este modulo solo mantiene los nombres antiguos para no tocar cada script.

No forma parte del build de colcon: son herramientas offline de analisis, para ejecutarse con `python3 scripts/xxx.py`
tras sourcear el workspace (necesitan rosbag2_py/rclpy para deserializar).
"""
from mocap_compare_common.mocap_tracks import (  # noqa: F401
    ALL_KEYPOINTS, FOOT_KEYS, LASER_IN_BASE_FOOTPRINT, LEG_KEYS, arclength_m, bag_duration_seconds, bag_start_time_ns,
    bag_topics, clip_track, find_labeled_bags, indexed_nearest, load_bag, load_calib, marker_by_name, marker_track,
    markers_by_name, nearest, project_map_to_laser, read_tag_window, repair_bag_metadata, rigid_body_pose_xy_yaw,
    rigid_body_track_xy, rigid_body_track_xy_yaw)
from walker_replay.bagio import replay_launch_args  # noqa: F401
