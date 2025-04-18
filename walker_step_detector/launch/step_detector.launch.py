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
                {"scan_topic" : "/scan_filtered"},
                #{"scan_topic" : "/scan_filtered2"},
                {"fixed_frame" : "base_link"},
                {"forest_file" : forest_file_path},
                {"detection_threshold": 0.001},
                {"cluster_dist_euclid": 0.02},
                {"min_points_per_cluster":  5},
                {"detect_distance_frame_id": "base_link"},
                {"max_detect_distance": 0.45},
                {"use_scan_header_stamp_for_tfs": False},
                {"max_detected_clusters": 4},
                {"plot_all_clusters": False},
                {"plot_leg_kalman": False},
                {"plot_leg_clusters": False},
                {"fixed_frame_active_area_x": [-0.75, 0.4]},
                {"fixed_frame_active_area_y": [-0.4, 0.4]}
            ],
        # Cheap way to reuse node with recorded rosbags
        #remappings=[ ("detected_step_left", "new_detected_step_left"),
        #             ("detected_step_right", "new_detected_step_right")]
    )

    #ld.add_action(laser_filter2_node)
    ld.add_action(detect_steps_node)
    
    return ld 
 
