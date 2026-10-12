#!/usr/bin/env python3

"""Odometria laser-inercial (lidar + IMU WitMotion) para sustituir a la
odometria de encoders de ruedas (walker_diff_odom), que ha dejado de
funcionar por rotura de hardware de los encoders.

Este launcher lanza SOLO dos cosas; el resto lo lanza el robot real:

    /scan_nav --[walker_lsm.launch.py, walker_lsm.yaml: ros2_laser_scan_matcher
                 (CSM / PL-ICP)]--> /scan_odom (nav_msgs/Odometry, pose+twist
                 con covarianza fija, ver laser_scan_matcher.cpp)

    /scan_odom + /bwt901cl/imu (cruda) --[robot_localization ekf_node,
        walker_ekf.yaml: odom0 (x,y,yaw abs + vx,vy) + imu0 (vyaw
        only, ver comentarios en walker_ekf.yaml sobre por que no se fusiona
        yaw absoluto del IMU)]--> /odom (remap de /odometry/filtered) + TF
        odom -> base_footprint

Entradas que ha de proporcionar el resto del sistema (NO se lanzan aqui):
  - /scan_nav: scan recortado de nav_laser_filter.launch.py (a su vez alimentado
    por /scan_filtered de laser_filter.launch.py, sobre el /scan de rplidar_ros).
  - /bwt901cl/imu: walker_witmotion.launch.py (witmotion_ros, IMU cruda; NO la
    filtrada: su estimador de sesgo corrompe el gyro, ver walker_ekf.yaml).
  - TF: walker_state_publisher (walker_description), que da
    base_footprint->laser y base_footprint->witmotion.

    ros2 launch walker_bringup walker_laser_imu_odom.launch.py

Argumentos (los ficheros permiten ajustar parametros sin tocar los de produccion,
ver launch/test/walker_laser_imu_odom_test.launch.py):
  use_sim_time      true al reproducir un rosbag.
  lsm_params_file   parametros del scan matcher (por defecto config/walker_lsm.yaml).
  ekf_params_file   parametros del EKF (por defecto config/walker_ekf.yaml).
  supervise_imu     true (por defecto): lanza scripts/odom_supervisor.py, que vigila el gyro y conmuta las
                    covarianzas que ve el EKF (el rumbo del laser pasa a pesar mas si el gyro falla). El EKF se
                    alimenta entonces de /scan_odom_ekf y /imu_ekf, no de /scan_odom ni /bwt901cl/imu.

Validado offline contra rosbags con ground-truth de mocap (rigid_bodies[0] =
el andador) en bagsFolder_unified -- ver launch/test/walker_laser_imu_odom_test.launch.py
y walker_mocap_eval/scripts/eval_odom_against_mocap.py.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
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

    ld.add_action(DeclareLaunchArgument(
        'supervise_imu', default_value='true',
        description='vigilar el gyro y conmutar las covarianzas del EKF (scripts/odom_supervisor.py)'))

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))

    # --- Odometria laser: CSM / PL-ICP scan matching -> /scan_odom ----------
    lsm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(bringup_dir, 'launch', 'walker_lsm.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'params_file': LaunchConfiguration('lsm_params_file')}.items())
    ld.add_action(lsm_launch)

    # --- Fusion: EKF (robot_localization) de /scan_odom + /bwt901cl/imu
    # -> /odom + TF odom->base_footprint -------------------------------------
    # Con supervisor el EKF se alimenta de sus salidas; el YAML sigue apuntando a /scan_odom y /bwt901cl/imu
    # para que sea valido por si solo (sin supervisor).
    ekf_args = dict(
        package='robot_localization',
        executable='ekf_node',
        name='walker_ekf_node',
        output='screen',
        remappings=[('/odometry/filtered', '/odom')],
    )
    ld.add_action(Node(
        **ekf_args, condition=IfCondition(LaunchConfiguration('supervise_imu')),
        # Con supervisor el gate de Mahalanobis de odom0 se relaja (5 -> 100 sigma): al conmutar, el rumbo del
        # laser entra con varianza 0.0012 y, si el gyro malo ya desvio el EKF mas de ~10 deg, un gate de 5 sigma
        # RECHAZARIA esa correccion hasta que el error diera la vuelta (medido: signo invertido 48 deg con 5, 24 con
        # 1000). Los outliers del ICP ya los acota max_linear_correction/max_angular_correction del scan matcher.
        parameters=[LaunchConfiguration('ekf_params_file'),
                    {'odom0': '/scan_odom_ekf', 'imu0': '/imu_ekf',
                     'odom0_pose_rejection_threshold': 100.0,
                     'odom0_twist_rejection_threshold': 100.0}]))
    ld.add_action(Node(
        **ekf_args, condition=UnlessCondition(LaunchConfiguration('supervise_imu')),
        parameters=[LaunchConfiguration('ekf_params_file')]))

    ld.add_action(Node(
        package='walker_bringup',
        executable='odom_supervisor.py',
        name='odom_supervisor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('supervise_imu')),
        parameters=[{'base_frame': 'base_footprint'}],
    ))

    return ld
