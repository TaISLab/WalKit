
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
          'frame_id':'camera_link',
          'subscribe_depth':True,
          'subscribe_odom_info':False,
          'subscribe_rgbd': False, # rgbd_image
          'approx_sync':True,
          'approx_sync_max_interval': 0.02,
          'wait_for_transform': 0.02,
          'queue_size': 100,
          'qos_imu':0,
          'publish_tf': True,
          'wait_imu_to_init':False}]

    remappings=[
          ('imu', '/bwt901cl/imu'),
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/depth/image_rect_raw')]


    return LaunchDescription([
        
        # Nodes to launch       
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
           package='rtabmap_slam', executable='rtabmap', output='screen',
           parameters=parameters,
           remappings=remappings,
           arguments=['-d']),

    #    Node(
    #        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
    #        parameters=parameters,
    #        remappings=remappings),
        
        # Because of this issue: https://github.com/IntelRealSense/realsense-ros/issues/2564
        # Generate point cloud from not aligned depth
        # Node(
        #     package='rtabmap_util', executable='point_cloud_xyz', output='screen',
        #     parameters=[{'approx_sync':False}],
        #     remappings=[('depth/image',       '/camera/depth/image_rect_raw'),
        #                 ('depth/camera_info', '/camera/depth/camera_info'),
        #                 ('cloud',             '/camera/cloud_from_depth')]),
        
        # Generate aligned depth to color camera from the point cloud above       
        # Node(
        #     package='rtabmap_util', executable='pointcloud_to_depthimage', output='screen',
        #     parameters=[{ 'decimation':2,
        #                   'fixed_frame_id':'camera_link',
        #                   'fill_holes_size':1}],
        #     remappings=[('camera_info', '/camera/color/camera_info'),
        #                 ('cloud',       '/camera/cloud_from_depth'),
        #                 ('image_raw',   '/camera/depth/image_raw')]),
        
        # Compute quaternion of the IMU
#        Node(
#            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
#            parameters=[{'use_mag': False, 
#                         'world_frame':'enu', 
#                         'publish_tf':False}],
#            remappings=[('imu/data_raw', '/camera/imu')]),
        
        # The IMU frame is missing in TF tree, add it:
#        Node(
#            package='tf2_ros', executable='static_transform_publisher', output='screen',
#            arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']),


    ])
