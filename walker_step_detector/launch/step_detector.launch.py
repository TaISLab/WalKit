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

    # filter laser data to get only legs area
    laser_filter2_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="step_laser_filter",
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory("walker_step_detector"),
                "config", "laser_box_filter.yaml",
            ])],
        remappings=[ ("scan", "scan_filtered"),
                     ("scan_filtered", "scan_filtered2")]
    )

    # Launching detect_steps node
    detect_steps_node = Node(
            package="walker_step_detector",
            executable="detect_steps",
            name="detect_steps",
            parameters= [
                {"scan_topic" : "/scan_filtered2"},
                {"fixed_frame" : "laser"},
                {"forest_file" : forest_file_path},
                {"detection_threshold": 0.01},
                {"cluster_dist_euclid": 0.13},
                {"min_points_per_cluster":  3},
                {"detect_distance_frame_id": "base_link"},
                {"max_detect_distance": 0.45},
                {"use_scan_header_stamp_for_tfs": False},
                {"max_detected_clusters": 2}
            ]
    )

    ld.add_action(laser_filter2_node)
    ld.add_action(detect_steps_node)

    return ld 
 
