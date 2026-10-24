#!/usr/bin/env python3
"""chassis_motion: how the walker's chassis moves, from its IMU. Publishes /chassis_motion (walker_msgs/ChassisMotion) every
`period_s` of data time over the last `window_s` seconds (walker_motion/chassis_motion.py defines every number).

The accelerations along the walker's x (AP) and y (ML) and the yaw rate are those of /bwt901cl/imu rotated into base_footprint with the
static transform of the IMU's frame (the IMU is not necessarily mounted aligned); without that transform it assumes it is aligned and
says so once. Data time: the clock is never read, the output carries the stamp of the tick, and the same bag gives the same numbers
at any replay rate.
"""
import math
import signal
import sys
from collections import deque

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from walker_msgs.msg import ChassisMotion

from walker_motion import chassis_motion
from walker_motion.chassis_motion import rotate, rotation_matrix


class ChassisMotionNode(Node):

    def __init__(self):
        super().__init__('chassis_motion')
        self.declare_parameters(namespace='', parameters=[
            ('imu_topic_name', '/bwt901cl/imu'),
            ('chassis_motion_topic_name', '/chassis_motion'),
            ('base_frame_id', 'base_footprint'),
            # data time the numbers cover (s) and how often they are published (s)
            ('window_s', 20.0),
            ('period_s', 1.0),
            # the uniform rate the window is resampled to (Hz)
            ('fs', 50.0),
        ])
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.base_frame, self.window_s, self.period_s, self.fs = p('base_frame_id'), float(p('window_s')), float(p('period_s')), float(p('fs'))
        self.samples = deque()               # (t_s, ax, ay, yaw_rate_deg_s) in base_footprint
        self.next_tick = None
        self.rotation = None
        self.warned = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(ChassisMotion, p('chassis_motion_topic_name'), 100)
        # best effort: compatible with the IMU driver and with a bag player; deep, a replay can deliver in bursts
        qos = QoSProfile(depth=500, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, p('imu_topic_name'), self.on_imu, qos)
        self.get_logger().info('chassis motion started')

    def aligned_rotation(self, frame_id):
        if self.rotation is None:
            try:
                q = self.tf_buffer.lookup_transform(self.base_frame, frame_id, Time(), Duration(seconds=0.0)).transform.rotation
                self.rotation = rotation_matrix((q.x, q.y, q.z, q.w))
            except Exception:  # noqa: BLE001
                if not self.warned:
                    self.get_logger().warn('No transform %s -> %s yet: taking the IMU as aligned with the walker' % (frame_id, self.base_frame))
                    self.warned = True
                return rotation_matrix((0.0, 0.0, 0.0, 1.0))
        return self.rotation

    def on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        r = self.aligned_rotation(msg.header.frame_id)
        a = rotate(r, (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
        w = rotate(r, (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
        if self.samples and t <= self.samples[-1][0]:
            return                                      # not in order: the window needs it
        self.samples.append((t, a[0], a[1], math.degrees(w[2])))
        while self.samples and t - self.samples[0][0] > self.window_s:
            self.samples.popleft()
        if self.next_tick is None:
            self.next_tick = t + self.period_s
        while t >= self.next_tick:
            self.publish(self.next_tick, msg.header.frame_id)
            self.next_tick += self.period_s

    def publish(self, tick_s, frame_id):
        window = [s for s in self.samples if s[0] <= tick_s]
        if len(window) < 8:
            return
        ts = [s[0] for s in window]
        _, ap = chassis_motion.resample_uniform(ts, [s[1] for s in window], self.fs)
        _, ml = chassis_motion.resample_uniform(ts, [s[2] for s in window], self.fs)
        _, yaw = chassis_motion.resample_uniform(ts, [s[3] for s in window], self.fs)
        m = chassis_motion.chassis_metrics(ap, ml, yaw, self.fs)
        out = ChassisMotion()
        out.header.stamp.sec, out.header.stamp.nanosec = divmod(int(round(tick_s * 1e9)), 1_000_000_000)
        out.header.frame_id = self.base_frame
        for field, value in m._asdict().items():
            setattr(out, field, float(value))
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ChassisMotionNode()
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
