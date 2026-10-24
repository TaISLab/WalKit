#!/usr/bin/python3
"""Gait quality from the leg loads: step width, double support, and the symmetry and variability of the step time.

Subscribes to /left_loads and /right_loads (StepStamped: foot position and load) and publishes /gait_quality
(walker_msgs/GaitQuality) at every stance onset, over the last `window_s` seconds of data time (0 = since the first onset). The
definitions are walker_loads/gait_quality.py's, shared with the mocap counterpart (mocap_loads) so that the two differ only in
how they find the stance. A leg is in stance when it carries at least `stance_load_fraction` of the user's weight (0.15; the weight
is that of /user_desc): the load of the swinging leg does not go to 0, it ramps with the relative speed of the feet, so "above 0" would
have both legs in stance almost always. The refractory period `min_step_interval_s` of gait_monitor_speed applies to the onsets (a load
that flickers back within it is the same stance).

Like the other nodes of this package it works on data time: the two inputs are put in stamp order (StampSequencer) and the output
carries the stamp of the onset that triggered it, so the same bag gives the same numbers at any replay rate.
"""
import signal
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from walker_msgs.msg import GaitQuality, StepStamped, UserDesc

from walker_loads import gait_quality
from walker_loads.stamp_sequencer import StampSequencer, stamp_ns

LEFT, RIGHT = 0, 1


class GaitQualityNode(Node):

    def __init__(self):
        super().__init__('gait_quality')
        self.declare_parameters(namespace='', parameters=[
            ('left_loads_topic_name', '/left_loads'),
            ('right_loads_topic_name', '/right_loads'),
            ('gait_quality_topic_name', '/gait_quality'),
            ('user_desc_topic_name', '/user_desc'),
            # a leg is in stance when it carries this share of the user's weight; the weight used until /user_desc arrives (kg)
            ('stance_load_fraction', 0.15),
            ('default_weight_kg', 60.0),
            # data time (s) the numbers cover; 0 = since the first onset
            ('window_s', 20.0),
            # refractory period of each leg's stance onsets (s), see step_onsets.py
            ('min_step_interval_s', 0.8),
            # data time to wait for a silent input before giving up on it, see StampSequencer
            ('max_wait_s', 5.0),
        ])
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.window_s = float(p('window_s'))
        self.stance_load_fraction = float(p('stance_load_fraction'))
        self.weight_kg = float(p('default_weight_kg'))
        self.legs = (gait_quality.LegStance(p('min_step_interval_s')), gait_quality.LegStance(p('min_step_interval_s')))
        self.first_onset = None
        # the loads repeat their step's stamp while the load itself is recomputed
        self.sequencer = StampSequencer(2, self.process, max_wait_ns=int(p('max_wait_s') * 1e9), repeating=(LEFT, RIGHT))
        self.pub = self.create_publisher(GaitQuality, p('gait_quality_topic_name'), 1000)
        # deep queues: the inputs wait in the sequencer for the slowest one, and a replayed bag can deliver them in bursts
        self.create_subscription(StepStamped, p('left_loads_topic_name'), lambda m: self.push(LEFT, m), 500)
        self.create_subscription(StepStamped, p('right_loads_topic_name'), lambda m: self.push(RIGHT, m), 500)
        latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(UserDesc, p('user_desc_topic_name'), self.on_user_desc, latched)
        self.get_logger().info('gait quality started')

    def on_user_desc(self, msg):
        if msg.weight > 0:
            self.weight_kg = float(msg.weight)

    def push(self, leg, msg):
        if not self.sequencer.push(leg, stamp_ns(msg.position.header.stamp), msg) and self.sequencer.late % 100 == 1:
            self.get_logger().warn('Dropped %d inputs older than what was already processed' % self.sequencer.late)
        self.sequencer.release()

    # Called by the sequencer, in stamp order
    def process(self, leg, stamp, msg):
        t = stamp * 1e-9
        # the load above the stance threshold: > 0 is a stance
        if not self.legs[leg].update(msg.load - self.stance_load_fraction * self.weight_kg, t, msg.position.point.y):
            return
        if self.first_onset is None:
            self.first_onset = t
        t0 = self.first_onset if self.window_s <= 0 else max(self.first_onset, t - self.window_s)
        q = gait_quality.quality(self.legs[LEFT].onsets, self.legs[RIGHT].onsets, self.legs[LEFT].stance(), self.legs[RIGHT].stance(), t0, t)
        out = GaitQuality()
        out.header.stamp, out.header.frame_id = msg.position.header.stamp, msg.position.header.frame_id
        out.window_s, out.steps = q.window_s, min(q.steps, 65535)
        out.step_width, out.double_support = q.step_width, q.double_support
        out.step_time, out.step_time_si, out.step_time_cv = q.step_time, q.step_time_si, q.step_time_cv
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GaitQualityNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
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
