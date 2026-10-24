#!/usr/bin/python3
"""Gait quality (/gait_quality) from the leg loads. Arguments: sim_time (false: use_sim_time, true when replaying a bag), debug
(false: DEBUG log level), window_s (20), min_step_interval_s (0.8), stance_load_fraction (0.15: the share of the user's weight a leg
carries to be in stance)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim_time', default_value='false', description='use_sim_time'),
        DeclareLaunchArgument('debug', default_value='false', description='DEBUG log level'),
        DeclareLaunchArgument('window_s', default_value='20.0', description='data time the numbers cover; 0 = since the first onset'),
        DeclareLaunchArgument('min_step_interval_s', default_value='0.8', description='refractory period of each leg'),
        DeclareLaunchArgument('stance_load_fraction', default_value='0.15', description="share of the user's weight a leg carries to be in stance"),
        Node(package='walker_loads', executable='gait_quality.py', name='gait_quality', output='screen',
             parameters=[{'use_sim_time': ParameterValue(LC('sim_time'), value_type=bool),
                          'window_s': ParameterValue(LC('window_s'), value_type=float),
                          'min_step_interval_s': ParameterValue(LC('min_step_interval_s'), value_type=float),
                          'stance_load_fraction': ParameterValue(LC('stance_load_fraction'), value_type=float),
                          'left_loads_topic_name': '/left_loads', 'right_loads_topic_name': '/right_loads',
                          'gait_quality_topic_name': '/gait_quality'}],
             arguments=['--ros-args', '--log-level', PythonExpression(["'debug' if '", LC('debug'), "' == 'true' else 'info'"])]),
    ])
