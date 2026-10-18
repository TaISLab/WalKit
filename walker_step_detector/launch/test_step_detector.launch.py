#!/usr/bin/python3

"""Prueba step_detector.launch.py SIN robot: reproduce un bag definitivo y
genera /scan_filtered como lo haria el robot real.

    walker_replay (replay_actions)
        reproduce /scan (y los topics extra que se pidan) como si los
        publicase el robot, con /clock y use_sim_time:=true para todo lo de
        este launcher, y publica el TF del andador (URDF).
    laser_filter.launch.py (walker_bringup)
        el mismo filtro del robot real: /scan -> /scan_filtered.
    step_detector.launch.py (este paquete)
        el launcher del robot, tal cual: es lo unico que se evalua. Publica
        /detected_step_left y /detected_step_right.

Asi, step_detector.launch.py queda preconfigurado para el robot (solo lanza
su detector, el filtro y el resto vienen del robot) y este launcher solo
aporta lo que en el robot ya existe. step_detector solo se suscribe a
/scan_filtered: no hacen falta handles ni configuracion de usuario.

Para comparar contra mocap hay que reproducir tambien el ground truth de
los bags definitivos (/labeled_markers, /rigid_bodies), que no esta en el
default de `topics`:
    ros2 launch walker_step_detector test_step_detector.launch.py \\
        bag_name:=CA_test01 rate:=4.0 \\
        topics:='/scan /labeled_markers /rigid_bodies'
mocap4w/mocap_step_detector/scripts/run_multibag_eval.py --launch-file test_step_detector.launch.py ...
lo hace sobre todos los bags (ver docs/detect_steps_algorithm.md).

Argumentos: bag_folder / bag_name (bag definitivo), rate, topics, delay
(espera antes de reproducir, para que el filtro y el detector, que carga el
random forest, ya esten suscritos y no se pierdan los primeros scans),
sim_time (true: es un banco de pruebas de bags), debug.

Para comparar las tres variantes entre si: test_step_detector_variants.launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from walker_replay.launch_utils import replay_actions

from os.path import expanduser, join


def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        "bag_folder", default_value=expanduser("~/bagsFolder/definitive"),
        description="carpeta base de los bags definitivos (y de test_config.txt)"))
    ld.add_action(DeclareLaunchArgument(
        "bag_name", default_value="CA_test01", description="bag definitivo a reproducir"))
    ld.add_action(DeclareLaunchArgument(
        "rate", default_value="1.0", description="velocidad de reproduccion del bag"))
    ld.add_action(DeclareLaunchArgument(
        "topics", default_value="/scan",
        description="topics del bag a reproducir, separados por espacios "
                    "(ver walker_replay)"))
    ld.add_action(DeclareLaunchArgument(
        "delay", default_value="3.0",
        description="segundos de espera antes de empezar a reproducir el bag, "
                    "para que el filtro y el detector (que carga el random "
                    "forest) ya esten suscritos y no se pierdan los primeros scans"))
    ld.add_action(DeclareLaunchArgument(
        "shutdown_on_end", default_value="false", description="apagar todo al acabar el bag"))
    ld.add_action(DeclareLaunchArgument(
        "sim_time", default_value="true",
        description="use_sim_time para todo lo del test (en un banco de pruebas de bags, true)"))
    ld.add_action(DeclareLaunchArgument(
        "debug", default_value="false", description="log en debug del reproductor del bag"))

    bringup = get_package_share_directory('walker_bringup')

    # El bag (y use_sim_time para lo que viene despues) primero; el
    # reproductor espera `delay` segundos, asi los nodos de abajo ya estan
    # suscritos cuando llega el primer scan.
    for action in replay_actions(
            bag_folder=LaunchConfiguration("bag_folder"),
            bag_name=LaunchConfiguration("bag_name"),
            rate=LaunchConfiguration("rate"),
            topics=LaunchConfiguration("topics"),
            delay=LaunchConfiguration("delay"),
            sim_time=LaunchConfiguration("sim_time"),
            debug=LaunchConfiguration("debug"),
            shutdown_on_end=LaunchConfiguration("shutdown_on_end")):
        ld.add_action(action)

    # Lo que en el robot real ya existe: el filtro que da /scan_filtered
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bringup, 'launch', 'laser_filter.launch.py'))))

    # Lo que se evalua: el launcher del robot, sin tocar
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('walker_step_detector'),
                 'launch', 'step_detector.launch.py'))))

    return ld
