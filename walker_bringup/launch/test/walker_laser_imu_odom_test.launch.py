#!/usr/bin/python3

"""Banco de pruebas offline de walker_laser_imu_odom.launch.py: reproduce un
rosbag con ground-truth de mocap (bagsFolder_unified, rigid_bodies[0] = el
andador) como si fuera el robot real, para ajustar los parametros de la
odometria sin el robot.

Que lanza:
  1. unified_bag_replay.launch.py: reproduce del bag solo
       - /scan,
       - /tf y /tf_static (el /tf_static del bag trae el arbol del andador,
         asi que no se lanza walker_state_publisher),
       - /bwt901cl/imu_filtered (lo que en el robot da walker_witmotion.launch.py),
       - /rigid_bodies (ground truth, para grabarlo junto a /odom).
     No se reproducen /scan_filtered, /scan_nav, /odom ni nada que vaya a
     regenerar este test.
  2. La cadena de scan del robot, siempre completa y con los filtros actuales
     (los bags solo traen /scan_filtered en 20 de 47 casos y con el filtro con
     el que se grabaron):
        /scan -> laser_filter.launch.py -> /scan_filtered
              -> nav_laser_filter.launch.py -> /scan_nav
     /scan_filtered no tiene las autocolisiones del laser con la estructura
     del andador; /scan_nav ademas no tiene los pies del usuario.
  3. walker_laser_imu_odom.launch.py: LO QUE SE PRUEBA (odometria laser + EKF),
     sin cambios, con use_sim_time:=true. Consume /scan_nav (laser_scan_topic en
     walker_lsm.yaml).
  No se lanza witmotion: en el robot real viene de fuera, y aqui
  /bwt901cl/imu_filtered sale del bag.

Argumentos:
  bag_path          carpeta del rosbag2 (por defecto MF_test07).
  rate              velocidad de reproduccion.
  extra_topics      resto de topics del bag a reproducir (ademas de /scan).
  lsm_params_file   parametros del scan matcher a probar.
  ekf_params_file   parametros del EKF a probar.

Ejemplo de ajuste (grabar la salida y puntuarla contra mocap):
    ros2 launch walker_bringup walker_laser_imu_odom_test.launch.py \\
        bag_path:=/home/mfcarmona/bagsFolder_unified/CA_test01 rate:=2.0 &
    ros2 bag record -o /tmp/odom_CA_test01 /odom /scan_odom /rigid_bodies
    python3 walker_mocap_eval/scripts/eval_odom_against_mocap.py --eval-bag /tmp/odom_CA_test01
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import SetParameter

from os.path import join


def generate_launch_description():
    bringup_dir = get_package_share_directory('walker_bringup')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'bag_path', default_value='/home/mfcarmona/bagsFolder_unified/MF_test07',
        description='carpeta de un rosbag2 de bagsFolder_unified'))
    ld.add_action(DeclareLaunchArgument(
        'rate', default_value='1.0', description='velocidad de reproduccion del bag'))
    ld.add_action(DeclareLaunchArgument(
        'extra_topics', default_value='/tf /tf_static /bwt901cl/imu_filtered /rigid_bodies',
        description='otros topics del bag a reproducir, separados por espacios (ademas de /scan)'))
    ld.add_action(DeclareLaunchArgument(
        'lsm_params_file', default_value=join(bringup_dir, 'config', 'walker_lsm.yaml'),
        description='parametros del scan matcher a probar'))
    ld.add_action(DeclareLaunchArgument(
        'ekf_params_file', default_value=join(bringup_dir, 'config', 'walker_ekf.yaml'),
        description='parametros del EKF a probar'))

    # use_sim_time para todo lo de este test (el yaml de nav_laser_filter lo trae a false)
    ld.add_action(SetParameter(name='use_sim_time', value=True))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bringup_dir, 'launch', 'test', 'unified_bag_replay.launch.py')),
        launch_arguments={'bag_path': LaunchConfiguration('bag_path'),
                          'rate': LaunchConfiguration('rate'),
                          'topics': PythonExpression(["'/scan ' + '", LaunchConfiguration('extra_topics'), "'"]),
                          'publish_walker_tf': 'false'}.items()))

    # /scan -> /scan_filtered -> /scan_nav (en el robot real lo lanzan otros launch)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bringup_dir, 'launch', 'laser_filter.launch.py'))))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bringup_dir, 'launch', 'nav_laser_filter.launch.py'))))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bringup_dir, 'launch', 'walker_laser_imu_odom.launch.py')),
        launch_arguments={'use_sim_time': 'true',
                          'lsm_params_file': LaunchConfiguration('lsm_params_file'),
                          'ekf_params_file': LaunchConfiguration('ekf_params_file')}.items()))

    return ld
