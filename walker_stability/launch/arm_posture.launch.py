#!/usr/bin/python3
"""Arm reach to the handles (/arm_posture) from /user_stability. Arguments: sim_time (false: use_sim_time, true when replaying a bag),
debug (false: DEBUG log level), arm_reach_offset_m (0.108)."""
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
        DeclareLaunchArgument('arm_reach_offset_m', default_value='0.108', description='handle distance minus reach'),
        Node(package='walker_stability', executable='arm_posture.py', name='arm_posture', output='screen',
             parameters=[{'use_sim_time': ParameterValue(LC('sim_time'), value_type=bool),
                          'arm_reach_offset_m': ParameterValue(LC('arm_reach_offset_m'), value_type=float),
                          'stability_topic_name': '/user_stability', 'arm_posture_topic_name': '/arm_posture',
                          'left_handle_frame': 'left_handle_id', 'right_handle_frame': 'right_handle_id'}],
             arguments=['--ros-args', '--log-level', PythonExpression(["'debug' if '", LC('debug'), "' == 'true' else 'info'"])]),
    ])
