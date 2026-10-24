#!/usr/bin/python3
"""How far the user's arms reach to the handles, from /user_stability (the estimate of the user's centre of mass) and the handle frames.

Publishes /arm_posture (walker_msgs/ArmPosture) with the stamp and frame of every /user_stability message it hears: the horizontal
distance from the centre of mass to the handles and, minus one constant, the estimate of the shoulder-to-hand reach; the elbow is NaN
(it cannot be predicted, see walker_stability/arm_posture.py). Data time: no clock, the output follows the input.
"""
import math
import signal
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from walker_msgs.msg import ArmPosture, StabilityStamped

from walker_stability import arm_posture


class ArmPostureNode(Node):

    def __init__(self):
        super().__init__('arm_posture')
        self.declare_parameters(namespace='', parameters=[
            ('stability_topic_name', '/user_stability'),
            ('arm_posture_topic_name', '/arm_posture'),
            ('left_handle_frame', 'left_handle_id'),
            ('right_handle_frame', 'right_handle_id'),
            ('arm_reach_offset_m', arm_posture.ARM_REACH_OFFSET_M),
        ])
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.handle_frames = (p('left_handle_frame'), p('right_handle_frame'))
        self.offset = float(p('arm_reach_offset_m'))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(ArmPosture, p('arm_posture_topic_name'), 1000)
        self.create_subscription(StabilityStamped, p('stability_topic_name'), self.on_stability, 500)
        self.get_logger().info('arm posture started')

    def handle_x(self, frame_id):
        """Mean x of the two handles in `frame_id`, or None while the tf is not there."""
        try:
            xs = [self.tf_buffer.lookup_transform(frame_id, f, Time(), Duration(seconds=0.0)).transform.translation.x
                  for f in self.handle_frames]
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn("Can't transform the handle frames into [%s]: %s" % (frame_id, e), throttle_duration_sec=5.0)
            return None
        return sum(xs) / len(xs)

    def on_stability(self, msg):
        hx = self.handle_x(msg.pose.header.frame_id)
        if hx is None:
            return
        distance = arm_posture.handle_distance(hx, msg.pose.pose.position.x)
        out = ArmPosture()
        out.header = msg.pose.header
        out.handle_distance = float(distance)
        out.reach = float(arm_posture.reach(distance, self.offset))
        out.elbow = math.nan
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ArmPostureNode()
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
