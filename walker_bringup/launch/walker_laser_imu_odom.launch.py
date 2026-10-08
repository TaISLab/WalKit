#!/usr/bin/env python3

"""Odometria laser-inercial (lidar + IMU WitMotion) para sustituir a la
odometria de encoders de ruedas (walker_diff_odom), que ha dejado de
funcionar por rotura de hardware de los encoders.

Este launcher lanza SOLO dos cosas; el resto lo lanza el robot real:

    /scan_nav --[walker_lsm.launch.py, walker_lsm.yaml: ros2_laser_scan_matcher
                 (CSM / PL-ICP)]--> /scan_odom (nav_msgs/Odometry, pose+twist
                 con covarianza fija, ver laser_scan_matcher.cpp)

    /scan_odom + /bwt901cl/imu_filtered --[robot_localization ekf_node,
        walker_ekf.yaml: odom0 (x,y,yaw abs + vx,vy) + imu0 (vroll,vpitch,vyaw
        only, ver comentarios en walker_ekf.yaml sobre por que no se fusiona
        yaw absoluto del IMU)]--> /odom (remap de /odometry/filtered) + TF
        odom -> base_footprint

Entradas que ha de proporcionar el resto del sistema (NO se lanzan aqui):
  - /scan_nav: scan recortado de nav_laser_filter.launch.py (a su vez alimentado
    por /scan_filtered de laser_filter.launch.py, sobre el /scan de rplidar_ros).
  - /bwt901cl/imu_filtered: walker_witmotion.launch.py (witmotion_ros +
    imu_complementary_filter, use_mag:false).
  - TF: walker_state_publisher (walker_description), que da
    base_footprint->laser y base_footprint->witmotion.

    ros2 launch walker_bringup walker_laser_imu_odom.launch.py

Argumentos (los ficheros permiten ajustar parametros sin tocar los de produccion,
ver launch/test/walker_laser_imu_odom_test.launch.py):
  use_sim_time      true al reproducir un rosbag.
  lsm_params_file   parametros del scan matcher (por defecto config/walker_lsm.yaml).
  ekf_params_file   parametros del EKF (por defecto config/walker_ekf.yaml).

Validado offline contra rosbags con ground-truth de mocap (rigid_bodies[0] =
el andador) en bagsFolder_unified -- ver launch/test/walker_laser_imu_odom_test.launch.py
y walker_mocap_eval/scripts/eval_odom_against_mocap.py.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from os.path import join


def generate_launch_description():
    bringup_dir = get_package_share_directory('walker_bringup')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='true al reproducir un rosbag (ver launch/test/walker_laser_imu_odom_test.launch.py)'))
    ld.add_action(DeclareLaunchArgument(
        'lsm_params_file', default_value=join(bringup_dir, 'config', 'walker_lsm.yaml'),
        description='parametros de ros2_laser_scan_matcher'))
    ld.add_action(DeclareLaunchArgument(
        'ekf_params_file', default_value=join(bringup_dir, 'config', 'walker_ekf.yaml'),
        description='parametros del EKF de robot_localization'))

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))

    # --- Odometria laser: CSM / PL-ICP scan matching -> /scan_odom ----------
    lsm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(bringup_dir, 'launch', 'walker_lsm.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'params_file': LaunchConfiguration('lsm_params_file')}.items())
    ld.add_action(lsm_launch)

    # --- Fusion: EKF (robot_localization) de /scan_odom + /bwt901cl/imu_filtered
    # -> /odom + TF odom->base_footprint -------------------------------------
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='walker_ekf_node',
        output='screen',
        remappings=[('/odometry/filtered', '/odom')],
        parameters=[LaunchConfiguration('ekf_params_file')],
    )
    ld.add_action(ekf_node)

    return ld
