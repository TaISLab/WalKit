# This file is a customization of :
# https://github.com/cra-ros-pkg/robot_localization/blob/foxy-devel/launch/ekf.launch.py
#

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("walker_bringup"), 'config', 'odom_ekf.yaml')],
           ),
])