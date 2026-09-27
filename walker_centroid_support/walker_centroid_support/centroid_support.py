import sys

from geometry_msgs.msg import PointStamped
import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from walker_msgs.msg import ForceStamped, StepStamped


class CentroidSupport(Node):

    def __init__(self):
        super().__init__('centroid_support')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('left_loads_topic_name', '/left_loads'),
                ('right_loads_topic_name', '/right_loads'),
                ('left_hand_loads_topic_name', '/left_hand_loads'),
                ('right_hand_loads_topic_name', '/right_hand_loads'),
                ('period', 0.05),
                ('centroid_topic_name', '/support_centroid'),
            ]
        )

        self.left_loads_topic_name = self.get_parameter('left_loads_topic_name').value
        self.right_loads_topic_name = self.get_parameter('right_loads_topic_name').value
        self.left_hand_loads_topic_name = self.get_parameter('left_hand_loads_topic_name').value
        self.right_hand_loads_topic_name = self.get_parameter('right_hand_loads_topic_name').value
        self.centroid_topic_name = self.get_parameter('centroid_topic_name').value
        self.period = self.get_parameter('period').value

        # stored data
        self.left_handle_msg = None
        self.right_handle_msg = None
        self.left_step_msg = None
        self.right_step_msg = None

        self.right_handle_weight = 0
        self.left_handle_weight = 0

        # ROS stuff
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.new_data_available = False
        self.first_data_ready = False

        self.centroid_pub = self.create_publisher(PointStamped, self.centroid_topic_name, 10)

        # Already-calibrated (kg) hand loads, published by walker_loads/partial_loads
        self.left_handle_sub = self.create_subscription(
            ForceStamped, self.left_hand_loads_topic_name, self.left_hand_load_callback, 10)
        self.right_handle_sub = self.create_subscription(
            ForceStamped, self.right_hand_loads_topic_name, self.right_hand_load_callback, 10)

        self.left_load_sub = self.create_subscription(
            StepStamped, self.left_loads_topic_name, self.left_load_callback, 10)
        self.right_load_sub = self.create_subscription(
            StepStamped, self.right_loads_topic_name, self.right_load_callback, 10)

        self.tmr = self.create_timer(self.period, self.timer_callback)

        self.get_logger().info('centroid support started')

    def left_hand_load_callback(self, msg):
        # msg.force already holds the calibrated weight (kg), as computed
        # and published by walker_loads/partial_loads from the raw handle readings.
        self.left_handle_msg = msg
        self.left_handle_weight = msg.force
        self.new_data_available = True

    def right_hand_load_callback(self, msg):
        self.right_handle_msg = msg
        self.right_handle_weight = msg.force
        self.new_data_available = True

    def left_load_callback(self, msg):
        self.load_callback(msg, 0)

    def right_load_callback(self, msg):
        self.load_callback(msg, 1)

    def load_callback(self, msg, leg_id):
        if leg_id == 1:
            self.right_step_msg = msg
            self.right_leg_load = self.right_step_msg.load
            self.right_speed = self.right_step_msg.speed
            self.new_data_available = True
        elif leg_id == 0:
            self.left_step_msg = msg
            self.left_leg_load = self.left_step_msg.load
            self.left_speed = self.left_step_msg.speed
            self.new_data_available = True
        else:
            self.get_logger().error(
                "Don't know about which step are you talking [" + str(leg_id) + ']')
            return

    def timer_callback(self):
        if not self.new_data_available:
            return
        self.new_data_available = False

        if not self.first_data_ready:
            self.first_data_ready = (
                self.left_handle_msg is not None and
                self.right_handle_msg is not None and
                self.left_step_msg is not None and
                self.right_step_msg is not None
            )

        if not self.first_data_ready:
            return

        # Handles are rigidly mounted on the walker but their height can change
        # at runtime, so the TF is looked up fresh on every cycle instead of
        # being cached, to always reflect the current handle_height.
        right_handle_pose = self.get_handle_pose(self.right_step_msg, self.right_handle_msg)
        left_handle_pose = self.get_handle_pose(self.left_step_msg, self.left_handle_msg)

        if right_handle_pose is None or left_handle_pose is None:
            return

        # Normalize by the sum of the four measured contact loads rather than
        # by the user's declared weight (/user_desc): the two only match
        # exactly when leg_load hasn't been clamped upstream (partial_loads
        # clamps it to [0, weight] to absorb handle-calibration/sensor
        # noise) and when the handle calibration matches the declared weight
        # perfectly. Normalizing by what was actually measured keeps this a
        # true barycenter (weights sum to 1) in both cases, instead of
        # silently drifting off the real contact points when they disagree.
        total_load = (self.right_leg_load + self.left_leg_load +
                      self.right_handle_weight + self.left_handle_weight)
        if total_load <= 0:
            self.get_logger().debug('No load measured yet, skipping centroid update')
            return

        right_step_position = np.array(
            [self.right_step_msg.position.point.x, self.right_step_msg.position.point.y, 0])
        left_step_position = np.array(
            [self.left_step_msg.position.point.x, self.left_step_msg.position.point.y, 0])
        right_handle_position = np.array(
            [right_handle_pose.point.x, right_handle_pose.point.y, right_handle_pose.point.z])
        left_handle_position = np.array(
            [left_handle_pose.point.x, left_handle_pose.point.y, left_handle_pose.point.z])

        centroid = (self.right_leg_load * right_step_position +
                    self.left_leg_load * left_step_position +
                    self.right_handle_weight * right_handle_position +
                    self.left_handle_weight * left_handle_position) / total_load

        centroid_msg = PointStamped()
        centroid_msg.header = self.right_step_msg.position.header
        centroid_msg.point.x = centroid[0]
        centroid_msg.point.y = centroid[1]
        centroid_msg.point.z = centroid[2]

        self.centroid_pub.publish(centroid_msg)

    # We take 0,0,0 in handles frame and cast it to steps frame.
    def get_handle_pose(self, step_msg, handle_msg):
        handle_orig = None
        if (step_msg is not None) and (handle_msg is not None):
            try:
                # Handle "is" at the origin of its own tf. So we just need to know the
                # transform between both
                trans = self.buffer.lookup_transform(
                    step_msg.position.header.frame_id, handle_msg.header.frame_id,
                    rclpy.time.Time())
                handle_orig = PointStamped()
                handle_orig.header = step_msg.position.header
                handle_orig.point.x = trans.transform.translation.x
                handle_orig.point.y = trans.transform.translation.y
                handle_orig.point.z = trans.transform.translation.z
            except Exception as e:
                self.get_logger().error(
                    "Can't transform handle point from frame [" + handle_msg.header.frame_id +
                    '] into frame [' + step_msg.position.header.frame_id + ']: [' + str(e) + ']')
        return handle_orig


def main(args=None):
    rclpy.init(args=args)

    myNode = CentroidSupport()

    try:
        rclpy.spin(myNode)
    except KeyboardInterrupt:
        print('User-requested stop')
    except BaseException:
        print('Exception in execution:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        myNode.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
