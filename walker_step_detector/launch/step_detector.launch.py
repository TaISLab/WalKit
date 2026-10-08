#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import launch
import os

step_detector_path = get_package_share_directory('walker_step_detector')
forest_file_path = os.path.join( step_detector_path, "config", "trained_step_detector_res_0.33.yaml")

def generate_launch_description():

    ld = LaunchDescription()

    # Parametros validados contra mocap sobre 47 bags (docs/
    # detect_steps_algorithm.md, eval_results/). Los de antes no estaban
    # validados, y varios de los que se pasaban (fixed_frame,
    # detect_distance_frame_id, use_scan_header_stamp_for_tfs, plot_*,
    # fixed_frame_active_area_*) no los declara detect_steps.cpp: se
    # ignoraban en silencio. detection_threshold y max_detected_clusters
    # activan el filtro de confianza del clasificador (sus defaults, -1.0/-1,
    # lo desactivan). No hay kalman_enabled: el EKF esta siempre activo.
    # publish_clusters=False: solo activa is_debug (log DEBUG + CSV en el
    # cwd), no cambia el tracking.
    detect_steps_node = Node(
            package="walker_step_detector",
            executable="detect_steps",
            name="detect_steps",
            parameters= [
                {"scan_topic" : "/scan_filtered"},
                {"forest_file" : forest_file_path},
                {"detected_steps_topic_name": "/detected_step"},
                {"kalman_model_d0": 0.001}, {"kalman_model_a0": 0.001},
                {"kalman_model_f0": 0.001}, {"kalman_model_p0": 0.001},
                {"detection_threshold": 0.01},
                {"cluster_dist_euclid": 0.13},
                {"max_detect_distance": 1.25},
                {"max_detected_clusters": 2},
                {"min_points_per_cluster": 3},
                {"publish_clusters": False},
            ],
    )
 
    ld.add_action(detect_steps_node)
    
    return ld 
 
