#!/usr/bin/env python3
"""Publishes, latched, what the definitive bags do not carry and the system expects.

Started by walker_replay/replay.launch.py (namespace /walker_replay), from the
test_config.txt row of the bag being replayed:

  /user_desc       walker_msgs/UserDesc  (transient_local) age, height, weight,
                   user_id, gender ('Masculino'|'Femenino'). test_config has no
                   Tinetti column: tinetti_score is published as 0.
  /handle_height   std_msgs/Int32 (transient_local) index 0-4 of the handles hole.
  /tf_static       base_footprint -> left_handle_id / right_handle_id, the pose of
                   the handles for that index in walker_description/config/params.yaml.
                   The handle height is constant during a test, so unlike the live
                   system (walker_description/handle_tf_publisher.py, dynamic /tf
                   following /handle_height) it is static: always valid, whatever
                   the stamp a node asks for.

The rest of the walker's tf (base_footprint -> laser, imu, wheels...) and the
latched /robot_description come from the URDF through walker_state_publisher
(robot_state_publisher), started by the launch file.
"""
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32
from tf2_ros import StaticTransformBroadcaster
from walker_msgs.msg import UserDesc

from walker_replay.test_config import read_bag_config

LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ReplayContext(Node):

    def __init__(self):
        super().__init__('context')
        self.declare_parameter('bag_folder', '')
        self.declare_parameter('bag_name', '')
        self.declare_parameter('test_config_file', '')
        self.declare_parameter('publish_context', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('handle_calibration_file', os.path.join(
            get_package_share_directory('walker_description'), 'config', 'params.yaml'))
        self.declare_parameter('frame_id', 'base_footprint')
        # same names as walker_description/handle_tf_publisher.py
        self.declare_parameter('child_frame_id_suffix', '_handle_id')

        bag_name = self.get_parameter('bag_name').value
        source = self.get_parameter('test_config_file').value or self.get_parameter('bag_folder').value
        self.cfg = read_bag_config(source, bag_name)
        self.get_logger().info(
            f"{bag_name}: handle_height={self.cfg.handle_height} user='{self.cfg.user_id}' "
            f"{self.cfg.gender_desc} age={self.cfg.age} height={self.cfg.height} weight={self.cfg.weight}")

        self.pubs = []     # keep the latched publishers alive
        if self.get_parameter('publish_context').value:
            self.publish_user_desc()
            self.publish_handle_height()
        if self.get_parameter('publish_tf').value:
            self.publish_handle_tf()

    def publish_user_desc(self):
        msg = UserDesc()
        msg.user_id = self.cfg.user_id
        msg.gender = self.cfg.gender_desc
        msg.age = self.cfg.age
        msg.height = self.cfg.height
        msg.weight = self.cfg.weight
        msg.tinetti_score = 0
        msg.description = self.cfg.notes
        pub = self.create_publisher(UserDesc, '/user_desc', LATCHED)
        pub.publish(msg)
        self.pubs.append(pub)

    def publish_handle_height(self):
        pub = self.create_publisher(Int32, '/handle_height', LATCHED)
        pub.publish(Int32(data=self.cfg.handle_height))
        self.pubs.append(pub)

    def publish_handle_tf(self):
        with open(self.get_parameter('handle_calibration_file').value, encoding='utf-8') as f:
            calib = yaml.safe_load(f)
        idx = self.cfg.handle_height
        if not 0 <= idx < len(calib['handle_z']):
            raise RuntimeError(f"handle_height {idx} out of the {len(calib['handle_z'])} "
                               f"holes of the handle calibration")
        frame_id = self.get_parameter('frame_id').value
        suffix = self.get_parameter('child_frame_id_suffix').value

        tfs = []
        for prefix, y_side in (('right', -1.0), ('left', 1.0)):
            tf = TransformStamped()      # stamp 0: static
            tf.header.frame_id = frame_id
            tf.child_frame_id = prefix + suffix
            tf.transform.translation.x = float(calib['handle_x'][idx])
            tf.transform.translation.y = y_side * float(calib['handle_y'][idx])
            tf.transform.translation.z = float(calib['handle_z'][idx])
            tf.transform.rotation.w = 1.0
            tfs.append(tf)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(tfs)


def main(args=None):
    rclpy.init(args=args)
    node = ReplayContext()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
