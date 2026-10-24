"""rviz2 with one of the walker's views, and the markers node that feeds it.

Arguments: view (walker), sim_time (false), debug (false), markers (true: also start walker_markers), rviz_config (empty = the
file of `view`). Views (walker_rviz/rviz/<view>.rviz):
  walker         everything the walker measures, live: scans, feet, markers of loads, centroid and stability, support polygon
  robot          the robot model with the scan and the odometry (what walker_description had)
  bringup_demo   the view of the bringup demo (scans, odometry, clusters), with the markers
  vicon          the same against a Vicon world
  step_detector  the step detector's work: scans, detected feet, segments
  navigation     the navigation stack's view: map, planners, particle cloud
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

VIEWS = ('walker', 'robot', 'bringup_demo', 'vicon', 'step_detector', 'navigation')


def _rviz(context):
    share = get_package_share_directory('walker_rviz')
    config = LC('rviz_config').perform(context).strip() or os.path.join(share, 'rviz', LC('view').perform(context).strip() + '.rviz')
    return [Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', config],
                 parameters=[{'use_sim_time': ParameterValue(LC('sim_time'), value_type=bool)}])]


def generate_launch_description():
    markers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('walker_rviz'), 'launch', 'walker_markers.launch.py')),
        launch_arguments={'sim_time': LC('sim_time'), 'debug': LC('debug')}.items(), condition=IfCondition(LC('markers')))
    return LaunchDescription([
        DeclareLaunchArgument('view', default_value='walker', description='one of: ' + ', '.join(VIEWS)),
        DeclareLaunchArgument('sim_time', default_value='false', description='use_sim_time'),
        DeclareLaunchArgument('debug', default_value='false', description='DEBUG log level'),
        DeclareLaunchArgument('markers', default_value='true', description='also start walker_markers'),
        DeclareLaunchArgument('rviz_config', default_value='', description='full path of an .rviz file, instead of `view`'),
        markers,
        OpaqueFunction(function=_rviz),
    ])
