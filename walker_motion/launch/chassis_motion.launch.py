"""Chassis motion (/chassis_motion) from the IMU. Arguments: sim_time (false: use_sim_time, true when replaying a bag), debug (false: DEBUG
log level), window_s (20), period_s (1)."""
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
        DeclareLaunchArgument('window_s', default_value='20.0', description='data time the numbers cover'),
        DeclareLaunchArgument('period_s', default_value='1.0', description='how often they are published, in data time'),
        Node(package='walker_motion', executable='chassis_motion', name='chassis_motion', output='screen',
             parameters=[{'use_sim_time': ParameterValue(LC('sim_time'), value_type=bool),
                          'window_s': ParameterValue(LC('window_s'), value_type=float),
                          'period_s': ParameterValue(LC('period_s'), value_type=float)}],
             arguments=['--ros-args', '--log-level', PythonExpression(["'debug' if '", LC('debug'), "' == 'true' else 'info'"])]),
    ])
