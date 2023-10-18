#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from os.path import join


step_detector_path = get_package_share_directory('walker_step_detector')
laser_segmentation_path = get_package_share_directory('laser_segmentation')
laser_segmentation_cfg_file = join( step_detector_path, "config", "laser_segmentation.yml")

def generate_launch_description():

    ld = LaunchDescription()

    # Laser segmentation node
    laser_segmentation_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                join(laser_segmentation_path, 'launch', 'segmentation.launch.py')),
            launch_arguments={
                'params_file': laser_segmentation_cfg_file          
            }.items()
        )
    ])
    ld.add_action(laser_segmentation_launch)

    # Launching detect_steps node
    detect_steps_node = Node(
            package="walker_step_detector",
            executable="detect_steps_s",
            name="detect_steps",
            parameters= [
                {"scan_topic" : "/scan_filtered"},
                {"fixed_frame" : "base_link"},
                {"detect_distance_frame_id": "base_link"},
                {"use_scan_header_stamp_for_tfs": False},
                {"plot_leg_kalman": True},
                {"plot_leg_clusters": False}
            ],
        # Cheap way to reuse node with recorded rosbags
        #remappings=[ ("detected_step_left", "new_detected_step_left"),
        #             ("detected_step_right", "new_detected_step_right")]
    )

    ld.add_action(detect_steps_node)
    
    return ld 
 
