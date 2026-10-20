#!/usr/bin/python3
"""User stability from the four contacts: how far the user's centre of mass is from the edge of the support polygon.

Subscribes to the loads of the two legs (/left_loads, /right_loads: foot position and load, from walker_loads) and
of the two handles (/left_hand_loads, /right_hand_loads), and to /user_desc (latched, optional) and publishes
/user_stability (walker_msgs/StabilityStamped). The geometry and what the numbers mean are in
walker_stability/stability_model.py.

Works on data time, like walker_centroid_support: the four inputs are put in stamp order (StampSequencer; the
leg loads by the stamp of the step position) and, on a fixed grid of data time (`period`), the state as of each tick is
evaluated and published, if anything new arrived since the previous tick, stamped with the tick. The output depends
only on the messages, not on when or in which order they arrive, and lags the data by up to ~max_wait_s.
"""
import math
import signal
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from walker_loads.stamp_sequencer import StampSequencer, Ticks, stamp_ns
from walker_msgs.msg import ForceStamped, StabilityStamped, StepStamped, UserDesc

from walker_stability.stability_model import LEFT, RIGHT, StabilityModel

LEFT_HAND, RIGHT_HAND, LEFT_LOAD, RIGHT_LOAD = 0, 1, 2, 3


class UserStability(Node):

    def __init__(self):
        super().__init__('user_stability')
        self.declare_parameters(namespace='', parameters=[
            ('left_loads_topic_name', '/left_loads'),
            ('right_loads_topic_name', '/right_loads'),
            ('left_hand_loads_topic_name', '/left_hand_loads'),
            ('right_hand_loads_topic_name', '/right_hand_loads'),
            ('user_desc_topic_name', '/user_desc'),
            ('stability_topic_name', '/user_stability'),
            # period of the output grid, in data time (s)
            ('period', 0.05),
            # data time to wait for a silent input before giving up on it, see StampSequencer
            ('max_wait_s', 5.0),
            # model, see stability_model.StabilityModel
            ('min_foot_load_kg', 5.0),
            ('com_tau_s', 0.5),
            ('margin_ref_m', 0.10),
            ('foot_heel_m', 0.08),
            ('foot_toe_m', 0.18),
            ('foot_half_width_m', 0.05),
        ])
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.stability_topic_name = p('stability_topic_name')
        self.model = StabilityModel(p('min_foot_load_kg'), p('com_tau_s'), p('margin_ref_m'), p('foot_heel_m'),
                                    p('foot_toe_m'), p('foot_half_width_m'))
        self.user = None
        self.step_header = None            # header of the latest step position: frame of the output
        self.handle_msgs = {}              # latest message of each handle, until its frame is resolved
        self.new_data_available = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ticks = Ticks(int(p('period') * 1e9))
        # the leg loads repeat their step's stamp while the load itself is recomputed
        self.sequencer = StampSequencer(4, self.process, max_wait_ns=int(p('max_wait_s') * 1e9),
                                        repeating=(LEFT_LOAD, RIGHT_LOAD))

        self.stability_pub = self.create_publisher(StabilityStamped, self.stability_topic_name, 1000)
        # Deep queues: the inputs wait in the sequencer for the slowest one, and a replayed bag can deliver them in
        # bursts; losing one would make the result depend on timing again.
        self.create_subscription(ForceStamped, p('left_hand_loads_topic_name'),
                                 lambda m: self.push(LEFT_HAND, stamp_ns(m.header.stamp), m), 500)
        self.create_subscription(ForceStamped, p('right_hand_loads_topic_name'),
                                 lambda m: self.push(RIGHT_HAND, stamp_ns(m.header.stamp), m), 500)
        self.create_subscription(StepStamped, p('left_loads_topic_name'),
                                 lambda m: self.push(LEFT_LOAD, stamp_ns(m.position.header.stamp), m), 500)
        self.create_subscription(StepStamped, p('right_loads_topic_name'),
                                 lambda m: self.push(RIGHT_LOAD, stamp_ns(m.position.header.stamp), m), 500)
        # for the user description topic we mimic a ROS1 'latched' topic
        latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(UserDesc, p('user_desc_topic_name'), self.user_desc_callback, latched)
        self.get_logger().info('user stability node started')

    def user_desc_callback(self, msg):
        self.get_logger().info('Valid user data received.')
        self.user = msg

    def push(self, stream, stamp, msg):
        if not self.sequencer.push(stream, stamp, msg) and self.sequencer.late % 100 == 1:
            self.get_logger().warn(
                'Dropped %d inputs older than what was already processed (an input stayed silent '
                'for more than max_wait_s)' % self.sequencer.late)
        self.sequencer.release()

    # Called by the sequencer, in stamp order
    def process(self, stream, stamp, msg):
        # the ticks that fall before this input see the state without it
        for tick in self.ticks.until(stamp):
            self.publish(tick)
        self.new_data_available = True
        if stream in (LEFT_LOAD, RIGHT_LOAD):
            self.step_header = msg.position.header
            self.model.set_foot(LEFT if stream == LEFT_LOAD else RIGHT, msg.position.point.x, msg.position.point.y, msg.load)
        else:
            self.handle_msgs[LEFT if stream == LEFT_HAND else RIGHT] = msg
        self.place_handles()

    def place_handles(self):
        """Hands the model the handle loads, with the handle positions in the frame of the step positions. The handle
        frames are rigid to the walker, but their height can change at runtime: the frame is looked up fresh."""
        if self.step_header is None:
            return
        for side in list(self.handle_msgs):
            msg = self.handle_msgs[side]
            xy = self.handle_position(msg.header.frame_id)
            if xy is not None:
                self.model.set_handle(side, xy[0], xy[1], msg.force)
                del self.handle_msgs[side]

    def handle_position(self, handle_frame):
        """(x, y) of the handle in the frame of the step positions."""
        try:
            t = self.tf_buffer.lookup_transform(self.step_header.frame_id, handle_frame, Time(), Duration(seconds=0.0))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error("Can't transform handle frame [%s] into [%s]: %s" % (
                handle_frame, self.step_header.frame_id, e), throttle_duration_sec=5.0)
            return None
        return t.transform.translation.x, t.transform.translation.y

    def publish(self, tick_ns):
        if not self.new_data_available:
            return
        self.new_data_available = False
        result = self.model.evaluate(tick_ns * 1e-9)
        if result is None:
            return
        out = StabilityStamped()
        out.uid = self.user.user_id if self.user else ''
        out.tin = self.user.tinetti_score / 28.0 if self.user else math.nan
        out.sta = float(result.assessment.sta)
        out.margin = float(result.assessment.margin)
        out.contacts = int(result.contacts)
        out.pose = PoseStamped()
        out.pose.header.frame_id = self.step_header.frame_id
        out.pose.header.stamp.sec, out.pose.header.stamp.nanosec = divmod(tick_ns, 1_000_000_000)
        out.pose.pose.position.x, out.pose.pose.position.y = result.com
        out.pose.pose.orientation.w = 1.0
        self.stability_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = UserStability()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('User-requested stop')
    except BaseException:
        print('Exception in execution:', file=sys.stderr)
        raise
    finally:
        # a second SIGINT (the terminal's and the launch's) can arrive while the node is being destroyed
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
