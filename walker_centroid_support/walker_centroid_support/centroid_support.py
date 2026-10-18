import sys

from geometry_msgs.msg import PointStamped
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from walker_msgs.msg import ForceStamped, StepStamped
from walker_loads.stamp_sequencer import StampSequencer, Ticks, stamp_ns

LEFT_HAND, RIGHT_HAND, LEFT_LOAD, RIGHT_LOAD = 0, 1, 2, 3



class CentroidSupport(Node):
    """Weighted centroid of the four contact points (two feet, two handles).

    Works on data time, not on the clock or a timer: the four inputs are put in stamp order
    (StampSequencer; the handle loads by their header stamp, the leg loads by the stamp of the step
    position) and, on a fixed grid of data time (`period`), the centroid of the state as of each tick
    is published, if anything new arrived since the previous tick, stamped with the stamp of the step
    position. The centroids therefore depend only on the messages, not on when or in which order they
    arrive (same bag, same centroids, at any replay rate), and lag the data by up to ~max_wait_s
    (normally less than the period of the slowest input, the handles).
    """

    def __init__(self):
        super().__init__('centroid_support')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('left_loads_topic_name', '/left_loads'),
                ('right_loads_topic_name', '/right_loads'),
                ('left_hand_loads_topic_name', '/left_hand_loads'),
                ('right_hand_loads_topic_name', '/right_hand_loads'),
                # period of the output grid, in data time (s)
                ('period', 0.05),
                # data time to wait for a silent input before giving up on it, see StampSequencer;
                # must be longer than the largest lag between inputs (the loads trail the odometry by
                # about a handle period, more when replaying faster than real time)
                ('max_wait_s', 5.0),
                ('centroid_topic_name', '/support_centroid'),
            ]
        )

        self.left_loads_topic_name = self.get_parameter('left_loads_topic_name').value
        self.right_loads_topic_name = self.get_parameter('right_loads_topic_name').value
        self.left_hand_loads_topic_name = self.get_parameter('left_hand_loads_topic_name').value
        self.right_hand_loads_topic_name = self.get_parameter('right_hand_loads_topic_name').value
        self.centroid_topic_name = self.get_parameter('centroid_topic_name').value
        self.period = self.get_parameter('period').value
        self.max_wait_s = self.get_parameter('max_wait_s').value

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

        self.first_data_ready = False
        self.new_data_available = False
        self.ticks = Ticks(int(self.period * 1e9))
        # the leg loads repeat their step's stamp while the load itself is recomputed
        self.sequencer = StampSequencer(4, self.process, max_wait_ns=int(self.max_wait_s * 1e9),
                                        repeating=(LEFT_LOAD, RIGHT_LOAD))

        self.centroid_pub = self.create_publisher(PointStamped, self.centroid_topic_name, 1000)

        # Deep queues: the inputs wait in the sequencer for the slowest one, and a replayed bag can
        # deliver them in bursts; losing one would make the result depend on timing again.
        # Already-calibrated (kg) hand loads, published by walker_loads/partial_loads
        self.left_handle_sub = self.create_subscription(
            ForceStamped, self.left_hand_loads_topic_name, self.left_hand_load_msg, 500)
        self.right_handle_sub = self.create_subscription(
            ForceStamped, self.right_hand_loads_topic_name, self.right_hand_load_msg, 500)

        self.left_load_sub = self.create_subscription(
            StepStamped, self.left_loads_topic_name, self.left_load_msg, 500)
        self.right_load_sub = self.create_subscription(
            StepStamped, self.right_loads_topic_name, self.right_load_msg, 500)

        self.get_logger().info('centroid support started')

    def left_hand_load_msg(self, msg):
        self.push(LEFT_HAND, stamp_ns(msg.header.stamp), msg)

    def right_hand_load_msg(self, msg):
        self.push(RIGHT_HAND, stamp_ns(msg.header.stamp), msg)

    def left_load_msg(self, msg):
        self.push(LEFT_LOAD, stamp_ns(msg.position.header.stamp), msg)

    def right_load_msg(self, msg):
        self.push(RIGHT_LOAD, stamp_ns(msg.position.header.stamp), msg)

    def push(self, stream, stamp, msg):
        if not self.sequencer.push(stream, stamp, msg) and self.sequencer.late % 100 == 1:
            self.get_logger().warn(
                'Dropped %d inputs older than what was already processed (an input stayed silent '
                'for more than max_wait_s)' % self.sequencer.late)
        self.sequencer.release()

    # Called by the sequencer, in stamp order
    def process(self, stream, stamp, msg):
        # the ticks that fall before this input see the state without it
        for _ in self.ticks.until(stamp):
            self.publish_centroid()
        self.new_data_available = True
        if stream == LEFT_HAND:
            # msg.force already holds the calibrated weight (kg), as computed
            # and published by walker_loads/partial_loads from the raw handle readings.
            self.left_handle_msg = msg
            self.left_handle_weight = msg.force
        elif stream == RIGHT_HAND:
            self.right_handle_msg = msg
            self.right_handle_weight = msg.force
        elif stream == LEFT_LOAD:
            self.left_step_msg = msg
            self.left_leg_load = msg.load
            self.left_speed = msg.speed
        else:
            self.right_step_msg = msg
            self.right_leg_load = msg.load
            self.right_speed = msg.speed

    def publish_centroid(self):
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
    except (KeyboardInterrupt, ExternalShutdownException):
        print('User-requested stop')
    except BaseException:
        print('Exception in execution:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        myNode.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
