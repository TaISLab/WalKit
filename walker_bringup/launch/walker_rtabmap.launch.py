
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from os.path import join


def generate_launch_description():
    parameters=[{
          'frame_id':'base_footprint',
          'subscribe_depth':False,
          'subscribe_rgbd': False, 
          'subscribe_stereo':True,
          'subscribe_odom_info':False,
          'approx_sync':True,
          'approx_sync_max_interval': 0.02,
          'wait_for_transform': 0.02,
          'queue_size': 100,
          'qos_imu':0,
          'publish_tf': True,
          'wait_imu_to_init':False}]

    remappings=[
          ('imu', '/imu/data'),
          ('left/image_rect', '/camera/infra1/image_rect_raw'),
          ('left/camera_info', '/camera/infra1/camera_info'),
          ('right/image_rect', '/camera/infra2/image_rect_raw'),
          ('right/camera_info', '/camera/infra2/camera_info'),          
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/depth/image_rect_raw'),
          ('odom', '/ekf_odom')  # /visual/odom
          
          ]


    return LaunchDescription([
        


        Node(
           package='rtabmap_slam', executable='rtabmap', output='screen',
           parameters=parameters,
           remappings=remappings,
           arguments=['-d', '--ros-args', '--log-level', 'info']
        ),

    #    Node(
    #        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
    #        parameters=parameters,
    #        remappings=remappings),
        
    ])
