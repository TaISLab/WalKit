#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """User stability (/user_stability): margin of the user's centre of mass in the support polygon of the feet and
    handles, from the leg and handle loads (walker_loads).

    Arguments: sim_time (false: use_sim_time, true when replaying a bag), debug (false: DEBUG log level), stamp_ordered
    (false: an output for every input message; true: inputs in stamp order, so the same bag gives the same output at
    any replay rate, see the README).
    The model's parameters (min_foot_load_kg, com_tau_s, margin_ref_m, foot_*_m) are documented in the README."""
    ld = LaunchDescription([
        DeclareLaunchArgument('sim_time', default_value='false', description='use_sim_time'),
        DeclareLaunchArgument('debug', default_value='false', description='DEBUG log level'),
        DeclareLaunchArgument('stamp_ordered', default_value='false',
                              description='inputs in stamp order, output on a grid of data time (reproducible)'),
    ])

    ld.add_action(Node(
        package='walker_stability',
        executable='user_stability.py',
        name='user_stability',
        parameters=[
            {'use_sim_time': ParameterValue(LaunchConfiguration('sim_time'), value_type=bool)},
            {'stamp_ordered': ParameterValue(LaunchConfiguration('stamp_ordered'), value_type=bool)},
            {'left_loads_topic_name': '/left_loads'},
            {'right_loads_topic_name': '/right_loads'},
            {'left_hand_loads_topic_name': '/left_hand_loads'},
            {'right_hand_loads_topic_name': '/right_hand_loads'},
            {'user_desc_topic_name': '/user_desc'},
            {'stability_topic_name': '/user_stability'},
            {'polygon_topic_name': '/support_polygon'},
        ],
        arguments=['--ros-args', '--log-level',
                   PythonExpression(["'debug' if '", LaunchConfiguration('debug'), "' == 'true' else 'info'"])],
    ))
    return ld
