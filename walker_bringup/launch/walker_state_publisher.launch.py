#!/usr/bin/env python3

'''
    Creates robot state publisher topic.
'''

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
import xacro

from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Default filenames and where to find them
    description_dir = get_package_share_directory('walker_description')
    xacro_file = os.path.join(description_dir, 'urdf', 'walker_model.xacro')
    
    ld = LaunchDescription()

    # Set env vars
    # to print messages to stdout immediately
    log_config = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    ld.add_action(log_config)
   
    # Create the launch configuration variables: 
    log_level = LaunchConfiguration('log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')
   
    # Map these variables to Arguments: can be set from the command line or a default will be used    
    log_level_launch_arg = DeclareLaunchArgument( "log_level", default_value=TextSubstitution(text="DEBUG") )
    use_sim_time_launch_arg = DeclareLaunchArgument("use_sim_time",default_value='false', description='Use simulation (Gazebo) clock if true')
   
    # Declare defined launch options 
    ld.add_action(log_level_launch_arg)
    ld.add_action(use_sim_time_launch_arg)
   
    # Spawning the walker  ...........................................................
    
    # create robot description from xacro file
    robot_description_config = xacro.process_file(xacro_file) 

    robot_desc = robot_description_config.toxml()
 
    # create state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='walker_state_publisher',
        output='screen', 
        parameters=[{'use_sim_time': use_sim_time},
                    {'robot_description': robot_desc}], 
        #remappings=[ ("robot_description", "walker_description") ]
        )
    ld.add_action(robot_state_publisher_node)

    return ld


if __name__ == '__main__':
    generate_launch_description()