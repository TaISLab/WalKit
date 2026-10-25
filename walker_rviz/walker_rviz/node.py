#!/usr/bin/env python3
"""walker_markers: draws the walker's real-time measures in rviz as markers, one layer per kind of data (see markers.py).

Every layer is a MarkerArray topic of the node (namespace /walker_rviz): handles, steps, detected_steps, centroid, stability,
gait, active_area (latched, drawn once), and can be switched off with show_<layer>:=false. The support polygon of
walker_stability is published by that node as a geometry_msgs/PolygonStamped (/support_polygon), which rviz draws natively.

Nothing here keeps state or reads the clock: a marker takes the stamp and frame of the message it draws, and lives lifetime_s.
"""
import os
import signal
import sys

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import MarkerArray
from walker_msgs.msg import ForceStamped, StabilityStamped, StepStamped, UserDesc, WalkStats

from walker_rviz import markers
from walker_rviz.active_area import boxes_from_yaml
from walker_rviz.handle_calibration import HandleCalibration

LATCHED = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def default_calibration_file():
    # soft dependency: walker_loads depends (through walker_step_detector) on this package's launchers, so this one cannot declare it
    try:
        return os.path.join(get_package_share_directory('walker_loads'), 'config', 'handle_calib.yaml')
    except PackageNotFoundError:
        return ''


def default_area_files():
    try:
        return [os.path.join(get_package_share_directory('walker_laser_filter'), 'config', 'feet_filter.yaml')]
    except PackageNotFoundError:
        return []


class WalkerMarkers(Node):

    def __init__(self):
        super().__init__('walker_markers')
        d = self.declare_parameter
        p = lambda name, default: d(name, default).value  # noqa: E731
        self.lifetime_s = float(p('lifetime_s', 0.5))
        self.handle_max_kg = float(p('handle_max_kg', 20.0))
        self.step_size_m = float(p('step_size_m', 0.15))
        self.weight_kg = float(p('default_weight_kg', 60.0))
        self.gait_frame = p('gait_frame_id', 'base_link')
        self.layers = {name: bool(p('show_' + name, default)) for name, default in (
            ('handles', True), ('steps', True), ('detected_steps', False), ('centroid', True), ('stability', True),
            ('gait', True), ('active_area', True))}
        self.pubs = {}
        self.held_load = {}                  # side -> the last load (kg) /left_loads or /right_loads gave, for the steps layer
        detected_prefix = p('detected_steps_topic_name', '/detected_step')

        if self.layers['handles']:
            raw = bool(p('handles_raw', False))
            calibration_file = p('handle_calibration_file', default_calibration_file())
            calibration = HandleCalibration.from_yaml(calibration_file) if raw else None
            for side, topic in ((markers.LEFT, p('left_hand_loads_topic_name', '/left_hand_loads')),
                                (markers.RIGHT, p('right_hand_loads_topic_name', '/right_hand_loads'))):
                if raw:
                    # the sensors' own readings (/left_handle, /right_handle), turned into kg with the calibration table:
                    # for a bag viewed on its own, with no walker_loads chain
                    topic = p(side + '_handle_topic_name', '/%s_handle' % side)
                self.layer('handles', ForceStamped, topic,
                           lambda m, side=side: markers.handle_markers(
                               self.in_kg(m, side, calibration), side, self.handle_max_kg, self.lifetime_s))
        if self.layers['steps']:
            # Where the loads' own message puts the foot is ~0.7 s old when it comes out (the loads wait for the handles), so the feet
            # trail the scan. With steps_at_detected the foot is drawn where the detected step is (about as old as the scan) and
            # coloured with the last load known, which changes slowly; no foot is drawn until its first load has arrived.
            at_detected = bool(p('steps_at_detected', True))
            for side, loads_topic in ((markers.LEFT, p('left_loads_topic_name', '/left_loads')),
                                      (markers.RIGHT, p('right_loads_topic_name', '/right_loads'))):
                if at_detected:
                    self.create_subscription(StepStamped, loads_topic, lambda m, side=side: self.hold_load(side, m), 10)
                    self.layer('steps', StepStamped, detected_prefix + '_' + side, lambda m, side=side: self.step_at_detected(side, m))
                else:
                    self.layer('steps', StepStamped, loads_topic,
                               lambda m, side=side: markers.step_markers(m, side, self.weight_kg, self.step_size_m, self.lifetime_s))
            self.create_subscription(UserDesc, p('user_desc_topic_name', '/user_desc'), self.on_user_desc, LATCHED)
        if self.layers['detected_steps']:
            min_confidence = float(p('min_confidence', 0.0))
            for side in (markers.LEFT, markers.RIGHT):
                self.layer('detected_steps', StepStamped, detected_prefix + '_' + side,
                           lambda m, side=side: markers.detected_step_markers(m, side, 0.05, self.lifetime_s)
                           if m.confidence >= min_confidence else [])
        if self.layers['centroid']:
            self.layer('centroid', PointStamped, p('centroid_topic_name', '/support_centroid'),
                       lambda m: markers.centroid_marker(m, 0.08, self.lifetime_s))
        if self.layers['stability']:
            self.layer('stability', StabilityStamped, p('stability_topic_name', '/user_stability'),
                       lambda m: markers.stability_markers(m, 0.1, self.lifetime_s))
        if self.layers['gait']:
            self.layer('gait', WalkStats, p('gait_stats_topic_name', '/global_gait_stats'),
                       lambda m: markers.gait_marker(m, self.gait_frame))
        if self.layers['active_area']:
            self.publish_active_area([f for f in p('active_area_files', default_area_files() or ['']) if f])
        self.get_logger().info('walker_markers started: ' + ', '.join(n for n, on in self.layers.items() if on))

    def publisher(self, layer, qos=10):
        if layer not in self.pubs:
            self.pubs[layer] = self.create_publisher(MarkerArray, layer, qos)
        return self.pubs[layer]

    def layer(self, layer, msg_type, topic, build):
        pub = self.publisher(layer)

        def callback(msg):
            out = build(msg)
            if out:
                pub.publish(MarkerArray(markers=out))

        self.create_subscription(msg_type, topic, callback, 10)

    @staticmethod
    def in_kg(msg, side, calibration):
        if calibration is None:
            return msg
        out = ForceStamped()
        out.header = msg.header
        out.force = float(calibration.kg(side, msg.force))
        return out

    def hold_load(self, side, msg):
        self.held_load[side] = float(msg.load)

    def step_at_detected(self, side, msg):
        if side not in self.held_load:
            return []
        return markers.step_markers(markers.with_load(msg, self.held_load[side]), side, self.weight_kg, self.step_size_m, self.lifetime_s)

    def on_user_desc(self, msg):
        if msg.weight > 0:
            self.weight_kg = float(msg.weight)

    def publish_active_area(self, files):
        boxes = []
        for path in files:
            try:
                boxes += boxes_from_yaml(path)
            except (OSError, ValueError) as e:
                self.get_logger().warn("Can't read the filter boxes in [%s]: %s" % (path, e))
        if boxes:
            self.publisher('active_area', LATCHED).publish(MarkerArray(markers=markers.box_markers(boxes, self.get_clock().now().to_msg())))
        else:
            self.get_logger().warn('No filter box to draw (active_area_files: %s)' % (files,))


def main(args=None):
    rclpy.init(args=args)
    node = WalkerMarkers()
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
