#!/usr/bin/python3

"""Reproduce un rosbag con ground-truth de mocap (bagsFolder_unified,
rigid_bodies[0] = el andador) y corre encima la odometria laser-inercial
(walker_laser_imu_odom.launch.py) para poder evaluarla offline sin hardware,
igual que walker_step_detector/launch/test_step_detector.launch.py hace para
los detectores de pasos.

Del bag solo se reproducen /scan, /bwt901cl/imu y /rigid_bodies -- NO /tf ni
/tf_static (los publica en vivo walker_state_publisher desde el xacro de
walker_description, para no duplicar/pisar el arbol TF con lo que hubiera
grabado ese bag en concreto) y NO se lanza witmotion_ros_node (necesita el
puerto serie real del IMU): en su lugar se corre directamente el
imu_complementary_filter que normalmente cuelga detras de el (mismos
parametros que walker_witmotion.launch.py), consumiendo el /bwt901cl/imu ya
grabado en el bag.

Para grabar la salida y evaluarla contra mocap:
    ros2 launch walker_bringup test_laser_imu_odom.launch.py \\
        bag_path:=/home/mfcarmona/bagsFolder_unified/MF_test01 rate:=1.0 &
    ros2 bag record -o /tmp/odom_eval_MF_test01 /odom /rigid_bodies
    # (parar ambos cuando el bag termine)
    python3 walker_step_detector/scripts/eval_odom_against_mocap.py \\
        --eval-bag /tmp/odom_eval_MF_test01
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from os.path import join

DEFAULT_BAG = "/home/mfcarmona/bagsFolder_unified/MF_test01"


def generate_launch_description():
    bringup_dir = get_package_share_directory('walker_bringup')
    description_dir = get_package_share_directory('walker_description')

    ld = LaunchDescription()

    bag_path_arg = DeclareLaunchArgument(
        'bag_path', default_value=DEFAULT_BAG,
        description='carpeta de un rosbag2 de bagsFolder_unified (o compatible: /scan, /bwt901cl/imu, /rigid_bodies)')
    rate_arg = DeclareLaunchArgument(
        'rate', default_value='1.0', description='velocidad de reproduccion del bag')
    ld.add_action(bag_path_arg)
    ld.add_action(rate_arg)

    bag_path = LaunchConfiguration('bag_path')
    rate = LaunchConfiguration('rate')

    # use_sim_time para TODO el arbol (scan matcher, filtros, EKF, robot_state_publisher):
    # el bag se reproduce con --clock, así que /clock manda la hora, no el reloj de pared.
    ld.add_action(SetParameter(name='use_sim_time', value=True))

    # TF real del andador (base_footprint -> laser, -> witmotion, etc.) desde
    # el xacro -- no se reproduce el /tf_static del bag para no tener dos
    # publicadores de la misma transformada estatica.
    walker_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(description_dir, 'launch', 'walker_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())
    ld.add_action(walker_tf)

    # IMU: solo el filtro complementario (sin el driver witmotion_ros_node,
    # que necesita el puerto serie real) sobre el /bwt901cl/imu ya grabado --
    # mismos parametros que walker_witmotion.launch.py.
    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ],
        remappings=[('imu/data_raw', '/bwt901cl/imu'),
                    ('imu/data', '/bwt901cl/imu_filtered'),
                    ('imu/mag', '/bwt901cl/magnetometer')])
    ld.add_action(imu_filter_node)

    # Laser: mismas dos etapas de recorte + scan matcher que en produccion.
    laser_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(bringup_dir, 'launch', 'laser_filter.launch.py')))
    ld.add_action(laser_filter_launch)

    nav_laser_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(bringup_dir, 'launch', 'nav_laser_filter.launch.py')))
    ld.add_action(nav_laser_filter_launch)

    lsm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(bringup_dir, 'launch', 'walker_lsm.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())
    ld.add_action(lsm_launch)

    # Fusion EKF -> /odom + TF odom->base_footprint.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='walker_ekf_node',
        output='screen',
        remappings=[('/odometry/filtered', '/odom')],
        parameters=[join(bringup_dir, 'config', 'walker_ekf.yaml')],
    )
    ld.add_action(ekf_node)

    # Reproduce el bag: /scan y /bwt901cl/imu alimentan la cadena de arriba,
    # /rigid_bodies se deja pasar sin mas para poder grabarlo junto a /odom y
    # evaluar despues con eval_odom_against_mocap.py.
    rosbag_cmd = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '-r', rate, '--clock', '--topics',
             '/scan', '/bwt901cl/imu', '/rigid_bodies'],
        output='screen')
    ld.add_action(rosbag_cmd)

    return ld
