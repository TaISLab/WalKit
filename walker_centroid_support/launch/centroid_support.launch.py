#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    # Launching centroid_support node
    walker_centroid_support_node = Node(
            package='walker_centroid_support',
            executable='centroid_support',
            name='centroid_support',
            parameters=[
                 {'left_hand_loads_topic_name': '/left_hand_loads'},
                 {'right_hand_loads_topic_name': '/right_hand_loads'},
                 {'period': 0.05},
                 {'left_loads_topic_name': '/left_loads'},
                 {'right_loads_topic_name': '/right_loads'},
                 {'centroid_topic_name': '/support_centroid'}
            ],
            # Cheap way to reuse node with recorded rosbags
            # remappings=[
            #             ('left_loads', 'new_left_loads'),
            #             ('right_loads', 'new_right_loads')]
    )

    ld.add_action(walker_centroid_support_node)

    return ld
