#!/usr/bin/env python3
 
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, OpaqueFunction

import os
import time
    
def launch_setup(context, *args, **kwargs):
    storage_folder = os.path.join(os.getenv("HOME"), 'bagsFolder')

    try:
        os.mkdir(storage_folder)        
    except FileExistsError:
        pass

    # Config
    topic_list = LaunchConfiguration('topic_list').perform(context)
    bag_name = LaunchConfiguration('bag_name').perform(context)

    rosbag_node = LaunchDescription([ ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-o', os.path.join(storage_folder, bag_name)] + topic_list.split(' '),
                output='screen'
            )
    ])
    
    return [rosbag_node]
    
def generate_launch_description():    
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value=TextSubstitution(text="DEBUG")),
        DeclareLaunchArgument('topic_list', default_value=TextSubstitution(text='/left_handle /odom /odom_lscm /right_handle /scan_filtered /sensor/bwt901cl/Angle /sensor/bwt901cl/Imu /sensor/bwt901cl/MagneticField /sensor/bwt901cl/Temperature /tf /tf_static /detected_step_left /detected_step_right /left_loads /right_loads /user_desc /handle_height')),  # /left_wheel /right_wheel 
        DeclareLaunchArgument('bag_name', default_value=TextSubstitution(text= time.strftime("%Y%m%d-%H%M%S-bag"))),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        OpaqueFunction(function = launch_setup)
        ])    