#!/usr/bin/env python3
"""Nodo ROS (no una herramienta offline: se lanza con `ros2 launch`/
`python3` mientras corre el resto del pipeline) que publica un
nav_msgs/Odometry en /odom a partir del rigid body del andador en el mocap
(mocap4r2_msgs/msg/RigidBodies, rigidbodies[0]).

Por que hace falta: walker_loads/launch/replay_offline.launch.py reproduce
el pipeline completo (deteccion de pasos + partial_loads + gait_monitor_
speed) contra los bags de datasets/labeled_bags para poder comparar sus
Step/Stride time/length, NoS, d, CAD y WV contra el ground truth de mocap
(walker_mocap_eval, punto 2 del plan de validacion). gait_monitor_speed
necesita /odom para "d" (antes hardcodeado, ver el propio nodo -- ya
corregido), y walker_diff_odom podria en teoria generarlo desde
/left_wheel, /right_wheel -- pero comprobado contra CA_test01: /right_wheel
no tiene NINGUN mensaje en el bag, y /left_wheel esta practicamente
congelado (3475 a 3476 en todo el test) durante el test entero. No hay
odometria de rueda utilizable en estos bags para esta validacion.

En vez de intentar arreglar/validar esa cadena de encoders (fuera de
alcance aqui -- se esta trabajando en ello por separado), este puente usa
el mocap como fuente de posicion "conocida buena": gait_monitor_speed solo
acumula la distancia euclidea entre posiciones consecutivas de /odom
(self.travelled), que es invariante a que frame se use, asi que publicar
directamente la pose del rigid body (frame "map", sin tocar) es suficiente
para ese unico proposito -- no pretende ser una fuente de /odom de uso
general (sin covarianza, sin twist, sin convencion odom-vs-map).

Uso (ver replay_offline.launch.py):
    python3 mocap_odom_bridge.py --ros-args \\
        -p rigid_bodies_topic_name:=/rigid_bodies -p odom_topic_name:=/odom
"""
import sys

import rclpy
from mocap4r2_msgs.msg import RigidBodies
from nav_msgs.msg import Odometry
from rclpy.node import Node


class MocapOdomBridge(Node):

    def __init__(self):
        super().__init__('mocap_odom_bridge')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rigid_bodies_topic_name', '/rigid_bodies'),
                ('odom_topic_name', '/odom'),
                ('rigid_body_index', 0),
            ])
        self.rigid_bodies_topic_name = self.get_parameter('rigid_bodies_topic_name').value
        self.odom_topic_name = self.get_parameter('odom_topic_name').value
        self.rigid_body_index = self.get_parameter('rigid_body_index').value

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic_name, 10)
        self.sub = self.create_subscription(
            RigidBodies, self.rigid_bodies_topic_name, self.rigid_bodies_cb, 10)

        self.get_logger().info(
            "mocap odom bridge started: [" + self.rigid_bodies_topic_name +
            "] (rigidbodies[" + str(self.rigid_body_index) + "]) -> [" +
            self.odom_topic_name + "]")

    def rigid_bodies_cb(self, msg):
        if len(msg.rigidbodies) <= self.rigid_body_index:
            return

        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose = msg.rigidbodies[self.rigid_body_index].pose
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = MocapOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('User-requested stop')
    except BaseException:
        print('Exception in execution:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
