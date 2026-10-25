"""The walker's whole scan chain: /scan -> /scan_filtered (self_filter) -> /scan_nav (nav_filter) and /scan_feet (feet_filter).

Arguments: sim_time (false), debug (false), scan_topic (/scan: the raw scan; the stages after the first take /scan_filtered)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC

from walker_laser_filter.launching import include_stage


def generate_launch_description():
    common = {'sim_time': LC('sim_time'), 'debug': LC('debug')}
    return LaunchDescription([
        DeclareLaunchArgument('sim_time', default_value='false', description='use_sim_time (replaying a bag)'),
        DeclareLaunchArgument('debug', default_value='false', description='DEBUG log level'),
        DeclareLaunchArgument('scan_topic', default_value='/scan', description='the raw scan'),
        include_stage('self_filter.launch.py', in_topic=LC('scan_topic'), **common),
        include_stage('nav_filter.launch.py', **common),
        include_stage('feet_filter.launch.py', **common),
    ])
