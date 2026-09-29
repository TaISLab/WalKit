#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import launch
import os

walker_loads_path = get_package_share_directory('walker_loads')
handle_calibration_file_path = os.path.join( walker_loads_path, "config", "handle_calib.yaml")

def generate_launch_description():

    ld = LaunchDescription()

    # Launching walker_loads node
    walker_loads_node = Node(
            package="walker_loads",
            executable="partial_loads",
            name="partial_loads",
            parameters= [
                 {'handle_calibration_file': handle_calibration_file_path},
                 {'left_handle_topic_name': '/left_handle'},
                 {'right_handle_topic_name': '/right_handle'},
                 {'user_desc_topic_name': '/user_desc'},
                 # ms_period (no "period"): partial_loads.cpp declara el parametro
                 # de su timer de calculo/publicacion como "ms_period" (int,
                 # milisegundos, default 500 = 2Hz) -- "period" no existe, asi que
                 # este valor (pensado como 0.05s = 50ms = 20Hz) nunca llegaba a
                 # aplicarse: el nodo corria siempre al default de 2Hz. Encontrado
                 # investigando por que gait_monitor_speed sigue infracontando NoS
                 # incluso tras corregir el timestamp de km_detect_steps (ver
                 # replay_offline.launch.py): con datos que llegan bien (steps a
                 # ~6Hz, manetas a ~2.4Hz, sin huecos > data_timeout_s), muestrear
                 # la decision de "que pierna carga" solo 2 veces por segundo
                 # sub-muestrea un ciclo de marcha real de ~2-3.4s (SpT/SdT de
                 # mocap, ver walker_mocap_eval) -- y en replay a --rate 4.0 ese
                 # timer de reloj de pared (create_wall_timer, no afectado por la
                 # velocidad del bag) corresponde a 2s de tiempo de bag entre
                 # muestras, mas de un ciclo de marcha completo. Con esto puesto a
                 # 50ms (=200ms de tiempo de bag a rate=4.0): spot-check en
                 # MT_test18 (NoS 21->178, mocap 153) y CA_test01 (NoS ->88,
                 # mocap 87) -- de infracontar severamente a sobrecontar un
                 # poco, por el ruido de km_detect_steps (sin filtro de
                 # confianza, ver km_detect_steps.cpp::getCentroids, que
                 # siempre fuerza 2 clusters aunque no haya pierna real).
                 # VALIDADO despues sobre los 47 bags completos
                 # (walker_mocap_eval/scripts/compare_gait_mocap_vs_walker.py,
                 # ROS_DOMAIN_ID aislado): NoS MAE 84 pasos/84% MAPE (antes de
                 # este fix Y del fix de timestamp de km_detect_steps,
                 # numeros del readme.md) -> MAE=23.5 pasos/25.1% MAPE. Los 3
                 # bags que antes no producian NINGUN GaitStamped
                 # (CA_test03, LA_test2, LA_test6) ahora si. d/wv siguen
                 # bien (MAPE 1.3%/13.2%). SpT/SdT/SpL/SdL siguen con error
                 # alto (MAPE 30-230%): heredan el ~25% de error de NoS que
                 # queda (ruido de km_detect_steps, mismo mecanismo de
                 # arriba) -- no arreglado aqui, ver walker_mocap_eval/readme.md.
                 {'ms_period': 50},
                 {'speed_delta': 0.05},
                 {'speed_delta_ratio': 0.3},
                 {'speed_delta_max': 1.0},
                 {'data_timeout_s': 1.0},
                 {'left_steps_topic_name': '/detected_step_left'},
                 {'right_steps_topic_name': '/detected_step_right'},
                 {'left_loads_topic_name': '/left_loads'},
                 {'debug_output': False},
                 {'right_loads_topic_name': '/right_loads'}                 
            ],
            # Cheap way to reuse node with recorded rosbags
#            remappings=[ ("detected_step_left", "new_detected_step_left"),
#                         ("detected_step_right", "new_detected_step_right"),
#                         ("left_loads", "new_left_loads"),
#                         ("right_loads", "new_right_loads")]
    )

    ld.add_action(walker_loads_node)
    
    return ld 
 

