#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from os.path import join


step_detector_path = get_package_share_directory('walker_step_detector')

def generate_launch_description():

    ld = LaunchDescription()

    # Launching detect_steps node
    detect_steps_node = Node(
            package="walker_step_detector",
            executable="km_detect_steps",
            name="detect_steps_km",
            parameters= [
                {"scan_topic": "/scan_filtered"},
                {"detected_steps_topic_name": "/detected_step"},
                {"detected_steps_frame": "/base_link"},
                {"kalman_enabled": False},
                {"kalman_model_d0": 0.001},
                {"kalman_model_a0": 0.001},
                {"kalman_model_f0": 0.001},
                {"kalman_model_p0": 0.001},
                {"plot_leg_kalman": True},
                {"plot_leg_clusters": False},
                {"use_scan_header_stamp_for_tfs": False},
                {"fixed_frame_active_area_x": [-0.75, 0.4]},
                {"fixed_frame_active_area_y": [-0.4, 0.4]}    
            ],
        # Cheap way to reuse node with recorded rosbags
        #remappings=[ ("detected_step_left", "new_detected_step_left"),
        #             ("detected_step_right", "new_detected_step_right")]
    )

    ld.add_action(detect_steps_node)
    
    return ld 
 

