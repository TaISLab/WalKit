"""The launch description of one scan filter: a `laser_filters` scan_to_scan_filter_chain with the parameter file of one stage of the
walker's scan chain. The three launchers of this package (self_filter, nav_filter, feet_filter) are this with their own file and topics.

Arguments of every launcher: sim_time (use_sim_time, false), debug (DEBUG log level, false), config (the parameter file), in_topic and
out_topic. The node lives in the namespace `walker_laser_filter` and the topics are absolute, so the namespace only keeps its parameters
and name apart from the other nodes of the robot.

The log level of `rcl.logging_rosout` is raised to error: every `LaserScanBoxFilter` is a lifecycle node that `laser_filters`
names "laser_scan_box_filter" whatever its name in the chain, so a chain with several of them makes rcl warn once per extra one that
their logs share a publisher. Nothing is lost: the filters' own messages go through their loggers, not that one.

To include more than one stage in a launch file use `include_stage`: the arguments given to an include (config, in_topic, ...) and
the defaults the included launcher declares stay in the launch context afterwards, and a stage included later takes them instead
of its own, silently (it was measured: all three stages ran with the first one's configuration and topics).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

NAMESPACE = 'walker_laser_filter'


def config_file(name):
    return os.path.join(get_package_share_directory('walker_laser_filter'), 'config', name)


def chain_launch_description(node_name, config_name, in_topic, out_topic):
    """node_name: name of the node; config_name: file in this package's config/; in_topic -> out_topic: the defaults."""
    log_level = PythonExpression(["'debug' if '", LC('debug'), "'.lower() == 'true' else 'info'"])
    return LaunchDescription([
        DeclareLaunchArgument('sim_time', default_value='false', description='use_sim_time (replaying a bag)'),
        DeclareLaunchArgument('debug', default_value='false', description='DEBUG log level'),
        DeclareLaunchArgument('config', default_value=config_file(config_name), description='parameter file of the filter chain'),
        DeclareLaunchArgument('in_topic', default_value=in_topic, description='scan to filter'),
        DeclareLaunchArgument('out_topic', default_value=out_topic, description='filtered scan'),
        Node(package='laser_filters', executable='scan_to_scan_filter_chain', name=node_name, namespace=NAMESPACE,
             parameters=[LC('config'), {'use_sim_time': ParameterValue(LC('sim_time'), value_type=bool)}],
             remappings=[('scan', LC('in_topic')), ('scan_filtered', LC('out_topic'))],
             arguments=['--ros-args', '--log-level', log_level, '--log-level', 'rcl.logging_rosout:=error'],
             output='screen'),
    ])


def include_stage(launcher, **arguments):
    """The launcher `launcher` of this package (e.g. 'nav_filter.launch.py') included with `arguments`, in a scope of its own so that
    nothing of it reaches whatever is included next."""
    path = os.path.join(get_package_share_directory('walker_laser_filter'), 'launch', launcher)
    return GroupAction(scoped=True, forwarding=True, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=arguments.items())])
