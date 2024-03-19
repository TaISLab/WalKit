
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from os.path import join


def generate_launch_description():

    remappings=[
          ('left/image_rect', '/camera/infra1/image_rect_raw'),
          ('left/camera_info', '/camera/infra1/camera_info'),
          ('right/image_rect', '/camera/infra2/image_rect_raw'),
          ('right/camera_info', '/camera/infra2/camera_info')]


    return LaunchDescription([
        
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen', 
            namespace='visual',
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("walker_bringup"),
                    "config", "stereo_odom.yaml",
                ])],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', 'warn']
            ),


    ])
