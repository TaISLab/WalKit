#!/usr/bin/env python3
"""Publish /user_desc (walker_msgs/UserDesc) and /handle_height (std_msgs/Int32)
from the console, same as the web GUI.

Usage:
  ros2 run walker_web_gui console_config.py user_id:="manusete" gender:="Masculino" \
      age:="46" height:="175" weight:="104" tinetti_score:="28" description:="Test" \
      handle_height:="4"

Omitted fields take the web GUI defaults (handle_height 0-4, default 4). Add once:="true" to exit right after
publishing; otherwise the node stays alive so the latched (transient_local)
message remains available to late subscribers until Ctrl+C.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Int32
from walker_msgs.msg import UserDesc

DEFAULTS = {
    'user_id': 'UCAmI',
    'gender': 'Masculino',
    'age': '42',
    'height': '175',
    'weight': '92',
    'tinetti_score': '24',
    'description': 'No previous condition',
    'handle_height': '4',
}
INT_FIELDS = ('age', 'height', 'weight', 'tinetti_score', 'handle_height')
GENDERS = ('Masculino', 'Femenino')
FLAGS = ('once',)


def parse_args(argv):
    values = dict(DEFAULTS)
    flags = {}
    for arg in argv:
        if ':=' not in arg:
            raise ValueError(f"argumento '{arg}' no tiene el formato nombre:=valor")
        key, value = arg.split(':=', 1)
        if key in FLAGS:
            flags[key] = value.strip().lower() in ('1', 'true', 'yes', 'si')
        elif key in DEFAULTS:
            values[key] = value
        else:
            raise ValueError(f"campo desconocido '{key}'. Válidos: {', '.join(list(DEFAULTS) + list(FLAGS))}")

    if values['gender'] not in GENDERS:
        raise ValueError(f"gender debe ser uno de {GENDERS}, no '{values['gender']}'")
    for key in INT_FIELDS:
        try:
            values[key] = int(values[key])
        except ValueError:
            raise ValueError(f"{key} debe ser un entero, no '{values[key]}'")
    if not 0 <= values['tinetti_score'] <= 28:
        raise ValueError('tinetti_score debe estar entre 0 y 28')
    if not 0 <= values['handle_height'] <= 4:
        raise ValueError('handle_height debe estar entre 0 y 4')
    return values, flags


def main():
    try:
        values, flags = parse_args(sys.argv[1:])
    except ValueError as e:
        print(f'Error: {e}', file=sys.stderr)
        return 2

    rclpy.init(args=[sys.argv[0]])  # our name:=value args are not ROS remap rules
    node = Node('console_config')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(UserDesc, '/user_desc', qos)
    handle_pub = node.create_publisher(Int32, '/handle_height', qos)

    msg = UserDesc()
    for key, value in values.items():
        if key != 'handle_height':
            setattr(msg, key, value)
    pub.publish(msg)
    handle_pub.publish(Int32(data=values['handle_height']))
    node.get_logger().info(f'Publicado: {values}')

    if flags.get('once'):
        # give the middleware a moment to deliver to current subscribers
        deadline = time.time() + 2.0
        while time.time() < deadline and (pub.get_subscription_count() == 0
                                          or handle_pub.get_subscription_count() == 0):
            rclpy.spin_once(node, timeout_sec=0.1)
        rclpy.spin_once(node, timeout_sec=0.2)
    else:
        node.get_logger().info('Mensaje latched mientras este nodo siga vivo. Ctrl+C para salir.')
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
