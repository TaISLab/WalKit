#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import launch
import os

def generate_launch_description():

    ld = LaunchDescription()

    # Launching walker_loads node
    walker_stability_node = Node(
            package="walker_stability",
            executable="walker_stability.py",
            name="walker_stability_node",
            parameters= [
                 {'stability_topic_name': '/user_stability'},
                 {'centroid_topic_name': '/support_centroid'},
                 {'user_desc_topic_name': '/user_desc'},
                 {'base_footprint_frame': 'base_footprint'},
                 {'left_handle_frame': 'left_handle_id'},
                 {'right_handle_frame': 'right_handle_id'}
            ],
    )

    ld.add_action(walker_stability_node)
    
    return ld 
 

