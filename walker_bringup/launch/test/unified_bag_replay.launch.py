#!/usr/bin/python3

"""
Reproduce un rosbag de forma que el sistema walkit cree que hay walker funcionando.
Esencialmente reproduce los topics provenientes del robot real grabados en el bag.

Argumentos:
  bag_path           carpeta del rosbag2.
  rate               velocidad de reproduccion.
  topics             topics a reproducir, separados por espacios (p.ej.
                     "/scan /tf /tf_static /bwt901cl/imu_filtered"). Vacio
                     (por defecto) = todos los del bag. Util para no
                     reproducir topics grabados que otro nodo del test va a
                     volver a generar (p.ej. /scan_nav, /odom).
  clock_hz           frecuencia de /clock (por defecto 200, ver su descripcion).
  publish_walker_tf  true (por defecto): lanza ademas walker_state_publisher
                     (TF del xacro). Ponlo a false si reproduces /tf_static
                     del bag, que ya trae el arbol del andador, para no tener
                     dos publicadores de las mismas transformadas estaticas.

/tf_static va siempre con QoS transient_local (config/tf_static_qos_override.yaml):
los bags lo traen grabado sin perfil QoS, ros2 bag play lo reproduciria
entonces con durabilidad volatil y cualquier nodo que arranque despues del
primer mensaje (p.ej. el EKF, esperando al reloj) no lo recibiria jamas.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from os.path import join


def _bag_play(context):
    cmd = ['ros2', 'bag', 'play', LaunchConfiguration('bag_path').perform(context),
           '-r', LaunchConfiguration('rate').perform(context),
           '--clock', LaunchConfiguration('clock_hz').perform(context),
           '--qos-profile-overrides-path',
           join(get_package_share_directory('walker_bringup'), 'config', 'tf_static_qos_override.yaml')]

    topics = LaunchConfiguration('topics').perform(context).split()
    if topics:
        cmd += ['--topics'] + topics

    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():

    ld = LaunchDescription()

    bag_path_arg = DeclareLaunchArgument(
                        "bag_path",
                        default_value="/home/mfcarmona/bagsFolder_unified/MF_test07",
                        description="carpeta de un rosbag2 de Walkit"
                    )
    ld.add_action(bag_path_arg)

    rate_arg = DeclareLaunchArgument(
        "rate",
        default_value="0.3",
        description="velocidad de reproduccion del bag"
    )
    ld.add_action(rate_arg)

    topics_arg = DeclareLaunchArgument(
        "topics",
        default_value="",
        description="topics a reproducir separados por espacios; vacio = todos los del bag"
    )
    ld.add_action(topics_arg)

    clock_hz_arg = DeclareLaunchArgument(
        "clock_hz",
        default_value="200",
        description="frecuencia (Hz de pared) de /clock. ros2 bag play publica a 40 Hz por defecto y los "
                    "temporizadores en tiempo simulado (el del EKF, a 30 Hz) solo disparan una vez por tick de "
                    "reloj: con 40 Hz la salida del EKF caia a ~14-19 Hz reales y variaba con la velocidad de "
                    "reproduccion; a 200 Hz es ~30 Hz y no depende de 'rate'"
    )
    ld.add_action(clock_hz_arg)

    publish_walker_tf_arg = DeclareLaunchArgument(
        "publish_walker_tf",
        default_value="true",
        description="lanzar walker_state_publisher (false si /tf_static se reproduce desde el bag)"
    )
    ld.add_action(publish_walker_tf_arg)

    # TF del andador
    walker_tf = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        join(
                            get_package_share_directory('walker_description'),
                            'launch',
                            'walker_state_publisher.launch.py'
                        )
                    ),
                    condition=IfCondition(LaunchConfiguration('publish_walker_tf'))
                )
    ld.add_action(walker_tf)

    # Reproduce el bag:
    ld.add_action(OpaqueFunction(function=_bag_play))

    return ld
