#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import launch
import os


def generate_launch_description():

    ld = LaunchDescription()

    # Launching gait_monitor_speed node
    gait_monitor_speed_node = Node(
            package="walker_loads",
            executable="gait_monitor_speed.py",
            name="gait_stats",
            parameters= [
                 {'period': 0.05},
                 {'left_loads_topic_name': '/left_loads'},
                 {'right_loads_topic_name': '/right_loads'},
                 {'left_gait_stats_topic_name': '/left_gait_stats'},
                 {'right_gait_stats_topic_name': '/right_gait_stats'},
                 {'global_gait_stats_topic_name': '/global_gait_stats'},
                 {'odom_topic_name': '/odom'},
                 {'global_frame_id': '/odom'}
            ],
    )

    ld.add_action(gait_monitor_speed_node)
    
    return ld 
 



