import os
import sys
import time

import pytest
import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from walker_rviz.node import WalkerMarkers  # noqa: E402

LAYERS = ('handles', 'steps', 'detected_steps', 'centroid', 'stability', 'gait', 'active_area')


@pytest.fixture
def ros_args():
    # the node declares its parameters in __init__: building it is the test, no graph is needed
    def start(*args):
        rclpy.init(args=['--ros-args', *args])
    yield start
    rclpy.try_shutdown()


def topics(node):
    return {s.topic_name for s in node.subscriptions}


def test_every_layer_at_once_declares_its_parameters_once(ros_args):
    # detected_steps declared its topic prefix inside the loop over the two sides: a second declaration, and the node died
    ros_args(*[a for layer in LAYERS for a in ('-p', 'show_%s:=true' % layer)])
    node = WalkerMarkers()
    try:
        assert set(node.pubs) == set(LAYERS)
        assert {'/detected_step_left', '/detected_step_right'} <= topics(node)
    finally:
        node.destroy_node()


def test_detected_steps_follows_the_topic_prefix(ros_args):
    ros_args('-p', 'show_detected_steps:=true', '-p', 'detected_steps_topic_name:=/detected_step_km')
    node = WalkerMarkers()
    try:
        assert {'/detected_step_km_left', '/detected_step_km_right'} <= topics(node)
    finally:
        node.destroy_node()


def test_handles_from_the_raw_sensors(ros_args):
    ros_args('-p', 'handles_raw:=true')
    node = WalkerMarkers()
    try:
        assert {'/left_handle', '/right_handle'} <= topics(node)
    finally:
        node.destroy_node()


def test_steps_are_drawn_at_the_detected_step_unless_told_to_use_the_loads(ros_args):
    ros_args()
    node = WalkerMarkers()
    try:
        assert {'/left_loads', '/right_loads', '/detected_step_left', '/detected_step_right'} <= topics(node)
    finally:
        node.destroy_node()
    rclpy.shutdown()
    ros_args('-p', 'steps_at_detected:=false')
    node = WalkerMarkers()
    try:
        assert {'/left_loads', '/right_loads'} <= topics(node) and not {'/detected_step_left', '/detected_step_right'} & topics(node)
    finally:
        node.destroy_node()


def test_a_foot_is_drawn_where_the_step_was_detected_with_the_last_load_and_not_before_a_load_is_known(ros_args):
    from rclpy.executors import SingleThreadedExecutor
    from visualization_msgs.msg import MarkerArray
    from walker_msgs.msg import StepStamped
    ros_args('-r', '__ns:=/walker_rviz')
    node = WalkerMarkers()
    helper = rclpy.create_node('walker_markers_test_helper')
    loads = helper.create_publisher(StepStamped, '/left_loads', 10)
    detected = helper.create_publisher(StepStamped, '/detected_step_left', 10)
    drawn = []
    helper.create_subscription(MarkerArray, '/walker_rviz/steps', drawn.append, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)

    def spin(seconds, until=lambda: False):
        end = time.time() + seconds
        while time.time() < end and not until():
            executor.spin_once(timeout_sec=0.05)

    def step(x, y, load):
        m = StepStamped()
        m.position.header.frame_id = 'laser'
        m.position.point.x, m.position.point.y, m.load, m.confidence = x, y, load, 1.0
        return m

    try:
        spin(5.0, lambda: loads.get_subscription_count() > 0 and detected.get_subscription_count() > 0)
        detected.publish(step(-0.6, 0.1, -1.0))
        spin(0.5)
        assert drawn == []                                              # no load known yet: no foot
        loads.publish(step(-1.5, 0.9, 30.0))                           # the load's own position is not used
        spin(0.3)
        detected.publish(step(-0.6, 0.1, -1.0))
        spin(3.0, lambda: bool(drawn))
        body, label = drawn[-1].markers
        assert label.text == 'left:30kg' and (round(body.pose.position.x, 3), round(body.pose.position.y, 3)) == (-0.6, 0.1)
        loads.publish(step(-1.5, 0.9, 12.0))
        spin(0.3)
        detected.publish(step(-0.5, 0.1, -1.0))
        drawn.clear()
        spin(3.0, lambda: bool(drawn))
        assert drawn[-1].markers[1].text == 'left:12kg'
    finally:
        executor.remove_node(node)
        executor.remove_node(helper)
        helper.destroy_node()
        node.destroy_node()
