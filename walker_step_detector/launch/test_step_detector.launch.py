#!/usr/bin/python3

"""Prueba step_detector.launch.py SIN robot: reproduce un bag y genera
/scan_filtered como lo haria el robot real.

    unified_bag_replay.launch.py (walker_bringup/launch/test)
        reproduce /scan, /tf, /tf_static (y los topics extra que se pidan)
        como si los publicase el robot, y lanza el TF del andador.
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
los bags etiquetados (datasets/labeled_bags), que no esta en el default de
`topics`:
    ros2 launch walker_step_detector test_step_detector.launch.py \\
        bag_path:=<labeled_bags/CA_test01> rate:=4.0 \\
        topics:='/scan /tf /tf_static /labeled_markers /rigid_bodies'
scripts/run_multibag_eval.py --launch-file test_step_detector.launch.py ...
lo hace sobre todos los bags (ver docs/detect_steps_algorithm.md).

Para comparar las tres variantes entre si: test_step_detector_variants.launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from os.path import join

DEFAULT_BAG = "/home/mfcarmona/workspace/walker_ws/src/soma13kp/datasets/labeled_bags/CA_test01"


def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        "bag_path", default_value=DEFAULT_BAG,
        description="carpeta de un rosbag2 (walkit o datasets/labeled_bags)"))
    ld.add_action(DeclareLaunchArgument(
        "rate", default_value="1.0", description="velocidad de reproduccion del bag"))
    ld.add_action(DeclareLaunchArgument(
        "topics", default_value="/scan /tf /tf_static",
        description="topics del bag a reproducir, separados por espacios "
                    "(ver unified_bag_replay.launch.py)"))
    ld.add_action(DeclareLaunchArgument(
        "publish_walker_tf", default_value="true",
        description="lanzar walker_state_publisher (ver unified_bag_replay.launch.py)"))
    ld.add_action(DeclareLaunchArgument(
        "replay_delay", default_value="3.0",
        description="segundos de espera antes de empezar a reproducir el bag, "
                    "para que el filtro y el detector (que carga el random "
                    "forest) ya esten suscritos y no se pierdan los primeros scans"))

    bringup = get_package_share_directory('walker_bringup')

    # Lo que en el robot real ya existe: scan, tf y el filtro que da /scan_filtered
    laser_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bringup, 'launch', 'laser_filter.launch.py')))
    ld.add_action(laser_filter)

    # Lo que se evalua: el launcher del robot, sin tocar
    step_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('walker_step_detector'),
                 'launch', 'step_detector.launch.py')))
    ld.add_action(step_detector)

    # El bag se reproduce el ultimo y con retraso: los nodos de arriba ya
    # estan suscritos cuando llega el primer scan.
    replay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(bringup, 'launch', 'test', 'unified_bag_replay.launch.py')),
        launch_arguments={
            'bag_path': LaunchConfiguration('bag_path'),
            'rate': LaunchConfiguration('rate'),
            'topics': LaunchConfiguration('topics'),
            'publish_walker_tf': LaunchConfiguration('publish_walker_tf'),
        }.items())
    ld.add_action(TimerAction(period=LaunchConfiguration('replay_delay'), actions=[replay]))

    return ld
