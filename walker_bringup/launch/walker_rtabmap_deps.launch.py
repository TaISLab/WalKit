
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from os.path import join


def generate_launch_description():

    realsense_dir = get_package_share_directory('realsense2_camera')
    realsense_launch_file = join( realsense_dir, 'launch', 'rs_launch.py')

    rplidar_ros2_dir = get_package_share_directory('rplidar_ros2')
    rplidar_ros2_launch_file = join( rplidar_ros2_dir, 'launch', 'rplidar_launch.py')

    walker_bringup_dir = get_package_share_directory('walker_bringup')
    nav_laser_filter_launch_file = join( walker_bringup_dir, 'launch', 'nav_laser_filter.launch.py')
    
    wit_config = join(walker_bringup_dir, 'config', 'wt901.yml')
    

    return LaunchDescription([
        
        # Depencencies ..................................................................................
        # camera
        # ros2 launch realsense2_camera rs_launch.py enable_sync:=true device_type:=d435 publish_tf:=true global_time_enabled:=false  initial_reset:=true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file),
                                          launch_arguments={
                                          'enable_sync': 'True',
                                          'device_type': 'd435',
                                          'publish_tf': 'True',
                                          'global_time_enabled':'False',
                                           'initial_reset': 'True'
                                          }.items(),
        ),

        # laser
        # ros2 launch rplidar_ros2  rplidar_launch.py inverted:=true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_ros2_launch_file),
                                          launch_arguments={
                                          'inverted': 'True'
                                          }.items(),
        ),

        # ros2 launch walker_bringup nav_laser_filter.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_laser_filter_launch_file)
        ),

        # IMU
        # ros2 run bwt901cl_pkg imu_bwt901cl        
        Node( package = 'witmotion_ros', executable = 'witmotion_ros_node',
                    parameters = [wit_config]
        ),



        # tf 
        # joining camera_link + laser frames + imu
        Node(package = 'tf2_ros', executable = "static_transform_publisher", name = 'laser2cam_tf',
                       arguments = ["0.1", "0.04", "0", "0", "0", "0", "laser", "camera_link"]
        ),
        Node(package = 'tf2_ros', executable = "static_transform_publisher", name = 'laser2imu_tf',
                       arguments = ["0.08", "0", "0", "0", "0", "0", "laser", "witmotion"]
        ),


    ])
