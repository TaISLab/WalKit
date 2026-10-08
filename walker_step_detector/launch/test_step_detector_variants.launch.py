#!/usr/bin/python3

"""(Antes test_step_detector.launch.py; renombrado al convertirse ese nombre
en el launcher de test de step_detector.launch.py via unified_bag_replay.
Este sigue siendo el que compara las variantes entre si: scripts/
run_multibag_eval.py lo usa por defecto.)

Lanza las TRES variantes de deteccion de pasos (detect_steps,
detect_steps_s, km_detect_steps) a la vez contra el mismo bag, cada una
publicando en un topic de salida distinto, para poder compararlas
directamente entre si y contra el ground truth de mocap.

Antes de este fix: se creaba un Node para `detect_steps` (Random Forest)
pero la variable se sobreescribia con el de `km_detect_steps` antes de
anadirse a la LaunchDescription -- solo se lanzaba km_detect_steps, nunca
las otras dos variantes, pese a que el nombre del fichero sugiere que
prueba "el" detector de pasos en general.

Bags: por defecto usa los de datasets/labeled_bags (soma13kp) -- traen
/scan crudo (sin pasar por walker_laser_filter) + ground truth de mocap
(/labeled_markers, /rigid_bodies). Aqui mismo se recorta ese /scan crudo a
/scan_feet con box_filter (walker_laser_filter, config/box_filter_eval.yaml
-- misma caja que inverted_box_filter.yaml, solo cambia el topic de
entrada) antes de pasarselo a las tres variantes: sin este recorte,
km_detect_steps (que no filtra por distancia en su propio codigo, ver
km_detect_steps.cpp) hace k-means sobre TODA la sala en vez de sobre las
piernas -- probado en vivo contra CA_test01, con /scan crudo publicaba
pies a 12-15 metros del robot. detect_steps_s necesita ademas /segments,
que aqui genera laser_segmentation en vivo a partir de /scan_feet (ver
config/laser_segmentation_eval.yml).

Ademas lanza detect_steps_fused, que fusiona los candidatos CRUDOS de las
tres variantes de arriba (topics /leg_candidates_rf|km|seg, publicados por
cada una antes de entrar a su propio LegsTracker) en un unico tracker --
aditivo, no sustituye a las tres variantes originales (ver
detect_steps_fused.cpp para la calibracion de confianza de km/segmentos).

Para evaluar contra ground truth: graba /detected_step_{rf,km,seg,fused}_*
junto con /labeled_markers y /rigid_bodies (p.ej. `ros2 bag record -o
eval_out /detected_step_rf_left /detected_step_rf_right ... /labeled_markers
/rigid_bodies`, o usa scripts/run_multibag_eval.py que ya lo hace) mientras
este launcher corre, y pasa ese bag de salida a scripts/eval_against_mocap.py
junto con config/mocap_to_base_footprint.yaml.

kalman_model_a0/f0: se probo a subirlos de 0.001 (practicamente cero) a
valores reales (a0=0.15m, f0=0.6Hz, medianas de 47 bags de mocap, ver
scripts/estimate_gait_params.py) para intentar arreglar la inobservabilidad
de frecuencia/fase diagnosticada en track_leg.cpp (dpos/df = 2*pi*a*cos(p)
~0 cuando a~0). Descartado: medido limpiamente en 47 bags (ROS_DOMAIN_ID
aislado, eval_results/result_2026-09-28_baseline_isolated.json vs
result_2026-09-28_gait_init_isolated.json), match_rate de rf EMPEORA
(93.7/93.8% -> 88.9/86.7%) mientras mean_err/swap_rate quedan casi iguales.
Motivo probable: p0 se dejo en 0.001 (fase arbitraria) -- con a0/f0 ya
realistas, el termino periodico SI oscila con amplitud/frecuencia reales
desde el primer instante, pero con la fase equivocada (no hay forma de
saber la fase real de la zancada al arrancar), asi que inyecta un error de
prediccion que antes no existia (cuando a0~0 el modelo era casi estatico,
sin oscilacion que pudiera desfasarse). Revertido a 0.001 en las cuatro
variantes. Arreglarlo de verdad necesitaria estimar la fase inicial de cada
pista a partir de sus primeras medidas reales, no fijarla a priori.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from os.path import join

step_detector_path = get_package_share_directory('walker_step_detector')
forest_file_path = join(step_detector_path, "config", "trained_step_detector_res_0.33.yaml")
segmentation_cfg_eval = join(step_detector_path, "config", "laser_segmentation_eval.yml")
box_filter_cfg_eval = join(step_detector_path, "config", "box_filter_eval.yaml")
rviz2_config_path = join(step_detector_path, "config", "test_step_detector.rviz")

DEFAULT_BAG = "/home/mfcarmona/workspace/walker_ws/src/soma13kp/datasets/labeled_bags/CA_test01"


def generate_launch_description():

    ld = LaunchDescription()

    bag_path_arg = DeclareLaunchArgument(
        "bag_path", default_value=DEFAULT_BAG,
        description="carpeta de un rosbag2 de datasets/labeled_bags (soma13kp)")
    rate_arg = DeclareLaunchArgument(
        "rate", default_value="0.3", description="velocidad de reproduccion del bag")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="false")
    ld.add_action(bag_path_arg)
    ld.add_action(rate_arg)
    ld.add_action(use_rviz_arg)

    bag_path = LaunchConfiguration("bag_path")
    rate = LaunchConfiguration("rate")

    # TF real del andador (base_footprint -> laser, etc.), desde el xacro de
    # walker_description -- no hardcodear el offset del laser aqui.
    walker_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('walker_description'),
                 'launch', 'walker_state_publisher.launch.py')))
    ld.add_action(walker_tf)

    # Recorta /scan (crudo) a /scan_feet -- la zona donde pueden aparecer
    # las piernas -- antes de pasarselo a cualquier detector (ver nota
    # arriba: km_detect_steps no es usable sin esto).
    box_filter_node = Node(
        package="walker_laser_filter", executable="box_filter", name="step_laser_filter",
        parameters=[box_filter_cfg_eval])
    ld.add_action(box_filter_node)

    # Segmentacion laser en vivo para detect_steps_s, ya sobre /scan_feet
    # (ver config/laser_segmentation_eval.yml).
    segmentation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('laser_segmentation'),
                 'launch', 'segmentation.launch.py')),
        launch_arguments={'params_file': segmentation_cfg_eval}.items())
    ld.add_action(segmentation_launch)

    # Reproduce el bag: /scan para los detectores (via box_filter arriba),
    # /tf* para el arbol propio del andador, /labeled_markers +
    # /rigid_bodies para poder grabarlos junto a las salidas y evaluar
    # despues.
    rosbag_cmd = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '-r', rate, '--topics',
             '/scan', '/tf', '/tf_static', '/labeled_markers', '/rigid_bodies'],
        output='screen')
    ld.add_action(rosbag_cmd)

    # --- Las tres variantes, cada una a un topic de salida distinto -------

    detect_steps_rf = Node(
        package="walker_step_detector", executable="detect_steps", name="detect_steps_rf",
        parameters=[
            {"scan_topic": "/scan_feet"},
            {"forest_file": forest_file_path},
            {"detected_steps_topic_name": "/detected_step_rf"},
            {"kalman_model_d0": 0.001}, {"kalman_model_a0": 0.001},
            {"kalman_model_f0": 0.001}, {"kalman_model_p0": 0.001},
            {"detection_threshold": 0.01},
            {"cluster_dist_euclid": 0.13},
            {"max_detect_distance": 1.25},
            {"max_detected_clusters": 2},
            {"min_points_per_cluster": 3},
            {"publish_clusters": True},
        ])
    ld.add_action(detect_steps_rf)

    detect_steps_km = Node(
        package="walker_step_detector", executable="km_detect_steps", name="detect_steps_km",
        parameters=[
            {"scan_topic": "/scan_feet"},
            {"detected_steps_topic_name": "/detected_step_km"},
            {"kalman_model_d0": 0.001}, {"kalman_model_a0": 0.001},
            {"kalman_model_f0": 0.001}, {"kalman_model_p0": 0.001},
            {"kalman_enabled": False},
            {"fit_ellipse": False},
            {"is_debug": True},
        ])
    ld.add_action(detect_steps_km)

    detect_steps_seg = Node(
        package="walker_step_detector", executable="detect_steps_s", name="detect_steps_seg",
        parameters=[
            {"segments_topic": "/segments"},
            {"detected_steps_topic_name": "/detected_step_seg"},
            {"kalman_model_d0": 0.001}, {"kalman_model_a0": 0.001},
            {"kalman_model_f0": 0.001}, {"kalman_model_p0": 0.001},
            {"kalman_enabled": False},
            {"is_debug": True},
        ])
    ld.add_action(detect_steps_seg)

    # --- Fusion: un unico tracker alimentado por los candidatos crudos de
    # las tres variantes de arriba (topics /leg_candidates_*, publicados por
    # ellas mismas antes de entrar a su propio LegsTracker) -- ver
    # detect_steps_fused.cpp. Aditivo: no sustituye a las tres de arriba.
    detect_steps_fused = Node(
        package="walker_step_detector", executable="detect_steps_fused", name="detect_steps_fused",
        parameters=[
            {"detected_steps_topic_name": "/detected_step_fused"},
            {"kalman_model_d0": 0.001}, {"kalman_model_a0": 0.001},
            {"kalman_model_f0": 0.001}, {"kalman_model_p0": 0.001},
            {"detection_threshold": 0.01},
            {"is_debug": True},
        ])
    ld.add_action(detect_steps_fused)

    # Visualizacion opcional (apagada por defecto: para grabar/evaluar no
    # hace falta rviz corriendo).
    rviz_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz2_config_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')))
    ld.add_action(rviz_cmd)

    return ld
