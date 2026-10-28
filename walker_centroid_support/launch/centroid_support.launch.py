#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # stamp_ordered (false): an output for every input message; true: inputs in stamp order, so the same bag gives the
    # same output at any replay rate (see the README)
    ld = LaunchDescription([
        DeclareLaunchArgument('stamp_ordered', default_value='false',
                              description='inputs in stamp order, output on a grid of data time (reproducible)')])

    # Launching centroid_support node
    walker_centroid_support_node = Node(
            package='walker_centroid_support',
            executable='centroid_support',
            name='centroid_support',
            parameters=[
                 {'stamp_ordered': ParameterValue(LaunchConfiguration('stamp_ordered'), value_type=bool)},
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
