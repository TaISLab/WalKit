import os
import sys

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
