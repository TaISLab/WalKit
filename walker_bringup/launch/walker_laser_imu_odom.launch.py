#!/usr/bin/env python3

"""Odometria laser-inercial (lidar + IMU WitMotion) para sustituir a la
odometria de encoders de ruedas (walker_diff_odom), que ha dejado de
funcionar por rotura de hardware de los encoders.

Cadena completa (todos los nodos ya existian por separado en walker_bringup,
este launcher solo los compone en el orden correcto):

    /scan --[laser_filter.launch.py, laser_filter_new.yaml]--> /scan_filtered
          --[nav_laser_filter.launch.py, nav_laser_filter.yaml]--> /scan_nav
          --[walker_lsm.launch.py, walker_lsm.yaml: ros2_laser_scan_matcher
             (CSM / PL-ICP)]--> /scan_odom (nav_msgs/Odometry, pose+twist con
             covarianza fija, ver laser_scan_matcher.cpp)

    /bwt901cl/imu --[walker_witmotion.launch.py: witmotion_ros +
                     imu_complementary_filter, use_mag:false]--> /bwt901cl/imu_filtered

    /scan_odom + /bwt901cl/imu_filtered --[robot_localization ekf_node,
        walker_ekf.yaml: odom0 (x,y,yaw abs + vx,vy) + imu0 (vroll,vpitch,vyaw
        only, ver comentarios en walker_ekf.yaml sobre por que no se fusiona
        yaw absoluto del IMU)]--> /odom (remap de /odometry/filtered) + TF
        odom -> base_footprint

No lanza: robot_state_publisher (walker_description/walker_state_publisher.launch.py,
ya publica TF base_footprint->laser y base_footprint->witmotion desde el
xacro), el driver del lidar (rplidar_ros) ni el driver de ruedas -- este
launcher es solo "el modulo de odometria", pensado para sustituir al pane
`sensors1: ros2 launch walker_diff_odom walker_diff_odom.launch.py` +
`sensors2: ros2 launch ros2_laser_scan_matcher demo_lsm.launch.py` del tmule
(WalKit/walker_bringup/tmule/walker_leap.yaml) por una unica linea:

    ros2 launch walker_bringup walker_laser_imu_odom.launch.py

Validado offline contra rosbags con ground-truth de mocap (rigid_bodies[0] =
el andador) en bagsFolder_unified -- ver
walker_bringup/launch/test_laser_imu_odom.launch.py y
walker_step_detector/scripts/eval_odom_against_mocap.py.
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

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='true al reproducir un rosbag (ver test_laser_imu_odom.launch.py)')
    ld.add_action(use_sim_time_arg)
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Propaga use_sim_time a TODOS los nodos de este arbol de lanzamiento,
    # incluidos los de los launch files incluidos abajo -- nav_laser_filter.yaml
    # trae use_sim_time:false fijo y ni laser_filter.launch.py, walker_lsm.launch.py
    # (usa RewrittenYaml pero para params_file, no para esto) ni
    # walker_witmotion.launch.py exponen el parametro por su cuenta.
    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))

    # --- IMU: witmotion_ros + filtro complementario (solo giroscopio+accel,
    # sin magnetometro) -> /bwt901cl/imu_filtered ------------------------------
    witmotion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(bringup_dir, 'launch', 'walker_witmotion.launch.py')))
    ld.add_action(witmotion_launch)

    # --- Laser: dos etapas de recorte (quitan el propio chasis del andador
    # del scan) antes del scan matcher ----------------------------------------
    laser_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(bringup_dir, 'launch', 'laser_filter.launch.py')))
    ld.add_action(laser_filter_launch)

    nav_laser_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(bringup_dir, 'launch', 'nav_laser_filter.launch.py')))
    ld.add_action(nav_laser_filter_launch)

    # --- Odometria laser: CSM / PL-ICP scan matching -> /scan_odom ----------
    lsm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(bringup_dir, 'launch', 'walker_lsm.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())
    ld.add_action(lsm_launch)

    # --- Fusion: EKF (robot_localization) de /scan_odom + /bwt901cl/imu_filtered
    # -> /odom + TF odom->base_footprint -------------------------------------
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='walker_ekf_node',
        output='screen',
        remappings=[('/odometry/filtered', '/odom')],
        parameters=[join(bringup_dir, 'config', 'walker_ekf.yaml')],
    )
    ld.add_action(ekf_node)

    return ld
