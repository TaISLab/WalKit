#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

'''


'''
    
def generate_launch_description():    
   
    ld = LaunchDescription()

    
    # Find out what we need and use after
    #ld.add_action(image_proc_launcher)

    mono_odometer_node = Node(
                        package='viso2_ros', 
                        executable='mono_odometer', 
                        output='screen', 
                        remappings=[("/image", "camera/image_raw"), 
                                    ("camera_info", "camera/camera_info" )],
                        parameters=[{
                            "odom_frame_id": "odom",
                            "base_link_frame_id": "camera",
                            "camera_height": 1.00, 
                            "camera_pitch": 0.00}]
                    )
    ld.add_action(mono_odometer_node)

    return ld


#######################################
