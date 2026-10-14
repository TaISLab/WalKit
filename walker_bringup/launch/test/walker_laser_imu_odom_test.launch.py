#!/usr/bin/python3

"""Banco de pruebas offline de walker_laser_imu_odom.launch.py: reproduce un
rosbag definitivo con ground-truth de mocap (rigid_bodies[0] = el andador)
como si fuera el robot real, para ajustar los parametros de la odometria sin
el robot.

Que lanza:
  1. walker_replay (replay_actions): reproduce del bag solo
       - /scan,
       - /bwt901cl/imu (IMU cruda; lo que en el robot da walker_witmotion.launch.py),
       - /rigid_bodies (ground truth, para grabarlo junto a /odom),
     con /clock y use_sim_time:=true para todo lo que se lanza despues. Publica
     ademas /user_desc, /handle_height y el TF del andador (walker_state_publisher
     desde el xacro actual, igual que en el robot; los bags definitivos no traen
     /tf ni /tf_static, y el que traian los originales llevaba el witmotion
     girado 180 grados).
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
  /bwt901cl/imu sale del bag.

Argumentos:
  bag_folder        carpeta base de los bags definitivos y de test_config.txt.
  bag_name          bag a reproducir (por defecto MF_test07).
  rate              velocidad de reproduccion.
  extra_topics      resto de topics del bag a reproducir (ademas de /scan).
  lsm_params_file   parametros del scan matcher a probar.
  ekf_params_file   parametros del EKF a probar.
  sim_time          true (por defecto, es un banco de pruebas de bags): use_sim_time
                    para todo lo del test. Con false no hay /clock ni tiempo de bag.
  debug             false por defecto: log en debug del reproductor.

Ejemplo de ajuste (grabar la salida y puntuarla contra mocap):
    ros2 launch walker_bringup walker_laser_imu_odom_test.launch.py \\
        bag_name:=CA_test01 rate:=2.0 &
    ros2 bag record -o /tmp/odom_CA_test01 /odom /scan_odom /rigid_bodies
    python3 walker_mocap_eval/scripts/eval_odom_against_mocap.py --eval-bag /tmp/odom_CA_test01
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from walker_replay.launch_utils import replay_actions

from os.path import expanduser, join


def generate_launch_description():
    bringup_dir = get_package_share_directory('walker_bringup')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'bag_folder', default_value=expanduser('~/bagsFolder/definitive'),
        description='carpeta base de los bags definitivos (y de test_config.txt)'))
    ld.add_action(DeclareLaunchArgument(
        'bag_name', default_value='MF_test07', description='bag definitivo a reproducir'))
    ld.add_action(DeclareLaunchArgument(
        'rate', default_value='1.0', description='velocidad de reproduccion del bag'))
    ld.add_action(DeclareLaunchArgument(
        'extra_topics', default_value='/bwt901cl/imu /rigid_bodies',
        description='otros topics del bag a reproducir, separados por espacios (ademas de /scan)'))
    ld.add_action(DeclareLaunchArgument(
        'lsm_params_file', default_value=join(bringup_dir, 'config', 'walker_lsm.yaml'),
        description='parametros del scan matcher a probar'))
    ld.add_action(DeclareLaunchArgument(
        'ekf_params_file', default_value=join(bringup_dir, 'config', 'walker_ekf.yaml'),
        description='parametros del EKF a probar'))

    ld.add_action(DeclareLaunchArgument(
        'sim_time', default_value='true',
        description='use_sim_time para todo lo del test (en un banco de pruebas de bags, true)'))
    ld.add_action(DeclareLaunchArgument(
        'debug', default_value='false', description='log en debug del reproductor del bag'))

    # Reproduccion del bag + use_sim_time para todo lo que viene despues
    # (el yaml de nav_laser_filter lo trae a false)
    for action in replay_actions(
            bag_folder=LaunchConfiguration('bag_folder'),
            bag_name=LaunchConfiguration('bag_name'),
            rate=LaunchConfiguration('rate'),
            topics=PythonExpression(["'/scan ' + '", LaunchConfiguration('extra_topics'), "'"]),
            sim_time=LaunchConfiguration('sim_time'),
            debug=LaunchConfiguration('debug')):
        ld.add_action(action)

    # /scan -> /scan_filtered -> /scan_nav (en el robot real lo lanzan otros launch)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bringup_dir, 'launch', 'laser_filter.launch.py'))))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bringup_dir, 'launch', 'nav_laser_filter.launch.py'))))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bringup_dir, 'launch', 'walker_laser_imu_odom.launch.py')),
        launch_arguments={'use_sim_time': LaunchConfiguration('sim_time'),
                          'lsm_params_file': LaunchConfiguration('lsm_params_file'),
                          'ekf_params_file': LaunchConfiguration('ekf_params_file')}.items()))

    return ld
