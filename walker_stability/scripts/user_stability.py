#!/usr/bin/python3
"""User stability index from the support centroid.

Subscribes to /support_centroid (walker_centroid_support) and /user_desc (latched) and publishes /user_stability
(walker_msgs/StabilityStamped): `sta` in 0..1, the product of
  - y: 1 with the centroid midway between the two handles, 0 on a handle's line or beyond it,
  - x: 1 with the centroid's distance to the handles at the middle of the base of support in x that the user's arms
       allow with the handles at their current height (walker_stability/bos.py), 0 at its limits or beyond them.
The output carries the centroid's own stamp, so it follows the data, not the clock.
"""
import sys

import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped with tf2's transform())
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from walker_msgs.msg import StabilityStamped, UserDesc

from walker_stability import bos


class WalkerStability(Node):

    def __init__(self):
        super().__init__('user_stability')
        self.declare_parameters(namespace='', parameters=[
            ('antropometric_data_file_uri', get_package_share_directory('walker_stability') + '/config/Correlationes.xlsx'),
            ('stability_topic_name', '/user_stability'),
            ('centroid_topic_name', '/support_centroid'),
            ('user_desc_topic_name', '/user_desc'),
            ('base_footprint_frame', 'base_footprint'),
            ('left_handle_frame', 'left_handle_id'),
            ('right_handle_frame', 'right_handle_id'),
        ])
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.stability_topic_name = p('stability_topic_name')
        self.base_footprint_frame = p('base_footprint_frame')
        self.left_handle_frame = p('left_handle_frame')
        self.right_handle_frame = p('right_handle_frame')

        self.tables = bos.AnthropometricTables(p('antropometric_data_file_uri'))
        self._bos_cache = {}
        self.user = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.stability_pub = self.create_publisher(StabilityStamped, self.stability_topic_name, 100)
        # deep queue: the centroids can arrive in bursts (walker_centroid_support publishes in data time)
        self.create_subscription(PointStamped, p('centroid_topic_name'), self.centroid_callback, 500)

        # for the user description topic we mimic a ROS1 'latched' topic
        latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(UserDesc, p('user_desc_topic_name'), self.user_desc_callback, latched)
        self.get_logger().info('user stability node started')

    def user_desc_callback(self, msg):
        self.get_logger().info('Valid user data received.')
        self.user = msg
        self._bos_cache.clear()

    def get_bos_x(self, handlebar_height_m):
        """(min, max) distance in m, cached: it only depends on the user and the handle height."""
        u = self.user
        key = (u.gender, u.age, u.height, round(handlebar_height_m, 3))
        if key not in self._bos_cache:
            row = self.tables.row(u.gender, u.age)
            rng = bos.bos_x_range(row, u.height / 100.0, handlebar_height_m)      # UserDesc.height is in cm
            if rng is None:
                self.get_logger().warn('No arm posture reaches the handlebar at %.3f m for this user; '
                                       'every distance counts as stable' % handlebar_height_m)
                rng = (-1.0, 1.0)
            self._bos_cache[key] = rng
        return self._bos_cache[key]

    def centroid_callback(self, msg):
        if self.user is None:
            self.get_logger().warn('No user description received yet. Ignoring centroid msg.', throttle_duration_sec=5.0)
            return
        try:
            lh = self.tf_buffer.lookup_transform(self.base_footprint_frame, self.left_handle_frame, Time(), Duration(seconds=1.0))
            rh = self.tf_buffer.lookup_transform(self.base_footprint_frame, self.right_handle_frame, Time(), Duration(seconds=1.0))
            centroid = self.tf_buffer.transform(msg, self.base_footprint_frame)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error("Can't get the transform from the handle frames [%s, %s] to [%s]: %s" % (
                self.left_handle_frame, self.right_handle_frame, self.base_footprint_frame, e), throttle_duration_sec=5.0)
            return

        handle_z = lh.transform.translation.z
        handle_x = lh.transform.translation.x
        # whichever handle is at the larger y: the convention of the frames does not matter
        y_ends = sorted((lh.transform.translation.y, rh.transform.translation.y))
        x_min, x_max = self.get_bos_x(handle_z)

        y_sta = bos.centered_score(centroid.point.y, y_ends[0], y_ends[1])
        x_sta = bos.centered_score(handle_x - centroid.point.x, x_min, x_max)      # distance from the centroid to the handles

        out = StabilityStamped()
        out.uid = self.user.user_id
        out.tin = self.user.tinetti_score / 28.0
        out.sta = float(x_sta * y_sta)
        out.pose = PoseStamped()
        out.pose.header = centroid.header
        out.pose.pose.position = centroid.point
        out.pose.pose.orientation.w = 1.0
        self.stability_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WalkerStability()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('User-requested stop')
    except BaseException:
        print('Exception in execution:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
