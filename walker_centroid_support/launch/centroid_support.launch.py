#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import launch
import os

walker_loads_path = get_package_share_directory('walker_loads')
handle_calibration_file_path = os.path.join( walker_loads_path, "config", "handle_calib.yaml")

def generate_launch_description():

    ld = LaunchDescription()

    # Launching centroid_support node
    walker_centroid_support_node = Node(
            package="walker_centroid_support",
            executable="centroid_support",
            name="centroid_support",
            parameters= [
                 {'handle_calibration_file': handle_calibration_file_path},
                 {'left_handle_topic_name': '/left_handle'},
                 {'right_handle_topic_name': '/right_handle'},
                 {'user_desc_topic_name': '/user_desc'},
                 {'period': 0.05},
                 {'speed_delta': 0.05},
                 {'left_loads_topic_name': '/left_loads'},
                 {'right_loads_topic_name': '/right_loads'},
                 {'centroid_topic_name': '/support_centroid'}
            ],
            # Cheap way to reuse node with recorded rosbags
            #remappings=[
            #             ("left_loads", "new_left_loads"),
            #             ("right_loads", "new_right_loads")]
    )

    ld.add_action(walker_centroid_support_node)
    
    return ld 
 

                 
