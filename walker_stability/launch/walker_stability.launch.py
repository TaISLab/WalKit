#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """User stability index (/user_stability) from /support_centroid and /user_desc.

    Arguments: sim_time (false: use_sim_time, true when replaying a bag), debug (false: DEBUG log level)."""
    ld = LaunchDescription([
        DeclareLaunchArgument('sim_time', default_value='false', description='use_sim_time'),
        DeclareLaunchArgument('debug', default_value='false', description='DEBUG log level'),
    ])

    ld.add_action(Node(
        package='walker_stability',
        executable='user_stability.py',
        name='user_stability',
        parameters=[
            {'use_sim_time': ParameterValue(LaunchConfiguration('sim_time'), value_type=bool)},
            {'centroid_topic_name': '/support_centroid'},
            {'user_desc_topic_name': '/user_desc'},
            {'stability_topic_name': '/user_stability'},
            {'base_footprint_frame': 'base_footprint'},
            {'left_handle_frame': 'left_handle_id'},
            {'right_handle_frame': 'right_handle_id'},
        ],
        arguments=['--ros-args', '--log-level',
                   PythonExpression(["'debug' if '", LaunchConfiguration('debug'), "' == 'true' else 'info'"])],
    ))
    return ld
