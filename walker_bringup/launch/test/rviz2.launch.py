#!/usr/bin/env python3
 
'''
    Launches rviz2 visualization for walker

    TODO: 
        lidar?
        save an actual rviz description file at description package with name 
'''

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

from launch_ros.actions import Node

import os

from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Default filenames and where to find them
    description_dir = get_package_share_directory('walker_description')
    default_walker_rviz2_file = os.path.join( description_dir, 'rviz2', 'walker.rviz')
    

    ld = LaunchDescription()

    # Set env vars
    # to print messages to stdout immediately
    log_config = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    ld.add_action(log_config)
   
    # Create the launch configuration variables: 
    log_level = LaunchConfiguration('log_level')
    walker_rviz2_file = LaunchConfiguration( 'walker_rviz2_file', default=default_walker_rviz2_file)
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Map these variables to Arguments: can be set from the command line or a default will be used    
    log_level_launch_arg = DeclareLaunchArgument( "log_level", default_value=TextSubstitution(text="DEBUG") )
    walker_rviz2_file_arg = DeclareLaunchArgument('walker_rviz2_file', default_value=default_walker_rviz2_file)
    use_sim_time_launch_arg = DeclareLaunchArgument("use_sim_time",default_value='false', description='Use simulation (Gazebo) clock if true')

    # Declare defined launch options 
    ld.add_action(log_level_launch_arg)
    ld.add_action(walker_rviz2_file_arg)
    ld.add_action(use_sim_time_launch_arg)


    # Nodes and launchers  ...........................................................

    # rviz2 ..........................................................................
    #   left
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', walker_rviz2_file, '--ros-args', '--log-level', log_level]
    )
    ld.add_action(rviz2_node)
    
    return ld


if __name__ == '__main__':
    generate_launch_description()