


from launch import LaunchDescription
from launch_ros.actions import Node

import os
import sys

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    bringup_dir = get_package_share_directory('walker_bringup')

    # get path to params file
    params_path = os.path.join(
        bringup_dir,
        'config',
        'left_cam.yaml'
    )

    ld.add_action(Node(
        package='usb_cam', executable='usb_cam_node_exe', output='screen',
        name='left_cam_node',
        namespace='left_cam',
        #remappings=[ ("image_raw", "left_image_raw")],
        parameters=[params_path]
        ))


    return ld
