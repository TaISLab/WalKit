# It's almost https://github.com/ros-drivers/usb_cam/blob/ros2/launch/demo_launch.py

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    package_dir = get_package_share_directory('walker_usb_cameras')

    # color camera
    right_params_path = os.path.join(
        package_dir,
        'config',
        'est_right_params.yaml'
    )

    # infrarred camera
    left_params_path = os.path.join(
        package_dir,
        'config',
        'est_left_params.yaml'
    )
    
    ld.add_action(Node(
        package='usb_cam', 
        executable='usb_cam_node_exe', 
        output='screen',
        name='est_right_node',
        namespace='camera/right',
        parameters=[right_params_path]
        ))

    ld.add_action(Node(
        package='usb_cam', 
        executable='usb_cam_node_exe', 
        output='screen',
        name='est_left_node',
        namespace='camera/left',
        parameters=[left_params_path]
        ))
    
    return ld
