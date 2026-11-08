#!/usr/bin/env python3

'''
Records one rosbag of the walker, in <bags_folder>/<campaign>_tests_walker/<bag_name>.

A test is recorded as two rosbags, one on each computer, with the same campaign and bag_name. The definitive-bag pipeline
(soma13kp/merge_definitive.py) pairs the two folders by that name and lines their clocks up with /tag, the button press that
both bags must carry; it also needs /scan in this one. Run it once per test with a new bag_name, and stop it with Ctrl-C.

https://github.com/IntelRealSense/realsense-ros/issues/315
We need to avoid compressed topics from realsense as they use incompatible formats ...
ros2 bag record -a -x "(.*)theora(.*)|(.*)compressed(.*)"

'''

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, OpaqueFunction

import os
import time

def launch_setup(context, *args, **kwargs):
    bags_folder = os.path.expanduser(LaunchConfiguration('bags_folder').perform(context))
    campaign_folder = os.path.join(bags_folder, LaunchConfiguration('campaign').perform(context) + '_tests_walker')

    os.makedirs(campaign_folder, exist_ok=True)

    # Config
    topic_list = LaunchConfiguration('topic_list').perform(context)
    bag_name = LaunchConfiguration('bag_name').perform(context)

    rosbag_node = LaunchDescription([ ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-o', os.path.join(campaign_folder, bag_name)] + topic_list.split(),
                output='screen'
            )
    ])

    return [rosbag_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value=TextSubstitution(text="DEBUG")),
        DeclareLaunchArgument('bags_folder', default_value=TextSubstitution(text='~/bagsFolder')),
        DeclareLaunchArgument('campaign', default_value=TextSubstitution(text=time.strftime("%Y_%m"))),
        DeclareLaunchArgument('topic_list', default_value=TextSubstitution(text='/image_raw /left_handle /odom /left_wheel /right_wheel /right_handle /scan /scan_filtered /bwt901cl/imu /bwt901cl/magnetometer /bwt901cl/temperature /tf /tf_static /detected_step_left /detected_step_right /left_loads /right_loads /user_desc /handle_height /tag')),  # /clock /odom_lscm
        DeclareLaunchArgument('bag_name', default_value=TextSubstitution(text= time.strftime("%Y%m%d-%H%M%S-bag"))),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        OpaqueFunction(function = launch_setup)
        ])
