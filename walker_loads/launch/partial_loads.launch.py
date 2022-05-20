#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import launch
import os

walker_loads_path = get_package_share_directory('walker_loads')
handle_calibration_file_path = os.path.join( walker_loads_path, "config", "params.yaml")

def generate_launch_description():

    ld = LaunchDescription()

    # Launching walker_loads node
    walker_loads_node = Node(
            package="walker_loads",
            executable="partial_loads",
            name="partial_loads",
            parameters= [
                 {'handle_calibration_file': handle_calibration_file_path},
                 {'left_handle_topic_name': '/left_handle'},
                 {'right_handle_topic_name': '/right_handle'},
                 {'user_desc_topic_name': '/user_desc'},
                 {'period': 0.05},
                 {'speed_delta': 0.05},
                 {'left_steps_topic_name': '/detected_step_left'},
                 {'right_steps_topic_name': '/detected_step_right'},
                 {'left_loads_topic_name': '/left_loads'},
                 {'right_loads_topic_name': '/right_loads'}
            ],
            # Cheap way to reuse node with recorded rosbags
            remappings=[ ("detected_step_left", "new_detected_step_left"),
                         ("detected_step_right", "new_detected_step_right"),
                         ("left_loads", "new_left_loads"),
                         ("right_loads", "new_right_loads")]
    )

    ld.add_action(walker_loads_node)
    
    return ld 
 

