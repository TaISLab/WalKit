import importlib.util
import os
import time

import pytest
import rclpy
from geometry_msgs.msg import PolygonStamped, TransformStamped
from rclpy.executors import SingleThreadedExecutor
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from walker_msgs.msg import ForceStamped, StabilityStamped, StepStamped

# the node is a script, not a module of the package
spec = importlib.util.spec_from_file_location('user_stability', os.path.join(os.path.dirname(__file__), '..', 'scripts', 'user_stability.py'))
user_stability = importlib.util.module_from_spec(spec)
spec.loader.exec_module(user_stability)

LEFT_HANDLE, RIGHT_HANDLE = (0.1, 0.25), (0.1, -0.25)


class Rig:
    """The node, and a helper node that plays the walker_loads outputs and the tf of the handles."""

    def __init__(self, *args):
        rclpy.init(args=['--ros-args', *args])
        self.node = user_stability.UserStability()
        self.helper = rclpy.create_node('user_stability_test_helper')
        pub = lambda t, topic: self.helper.create_publisher(t, topic, 10)  # noqa: E731
        self.hand = {'left': pub(ForceStamped, '/left_hand_loads'), 'right': pub(ForceStamped, '/right_hand_loads')}
        self.leg = {'left': pub(StepStamped, '/left_loads'), 'right': pub(StepStamped, '/right_loads')}
        self.out, self.polygons = [], []
        self.helper.create_subscription(StabilityStamped, '/user_stability', self.out.append, 100)
        self.helper.create_subscription(PolygonStamped, '/support_polygon', self.polygons.append, 100)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.helper)
        broadcaster = StaticTransformBroadcaster(self.helper)
        broadcaster.sendTransform([self.transform('left_handle', LEFT_HANDLE), self.transform('right_handle', RIGHT_HANDLE)])
        self.spin(10.0, lambda: all(p.get_subscription_count() for p in [*self.hand.values(), *self.leg.values()])
                  and self.node.tf_buffer.can_transform('laser', 'right_handle', rclpy.time.Time()))

    @staticmethod
    def transform(child, xy):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'laser', child
        t.transform.translation.x, t.transform.translation.y = xy
        t.transform.rotation.w = 1.0
        return t

    def spin(self, seconds, until=lambda: False):
        end = time.time() + seconds
        while time.time() < end and not until():
            self.executor.spin_once(timeout_sec=0.02)

    def settle(self, seconds=0.3):
        self.spin(seconds)

    def hand_load(self, side, kg, stamp):
        m = ForceStamped()
        m.header.frame_id, m.force = side + '_handle', float(kg)
        m.header.stamp.sec, m.header.stamp.nanosec = int(stamp), int(stamp % 1 * 1e9)
        self.hand[side].publish(m)

    def leg_load(self, side, x, y, kg, stamp):
        m = StepStamped()
        m.position.header.frame_id, m.load = 'laser', float(kg)
        m.position.point.x, m.position.point.y = x, y
        m.position.header.stamp.sec, m.position.header.stamp.nanosec = int(stamp), int(stamp % 1 * 1e9)
        self.leg[side].publish(m)

    def all_four(self, stamp, hands=(5, 5), legs=(30, 30), hands_stamp=None):
        self.hand_load('left', hands[0], hands_stamp or stamp)
        self.hand_load('right', hands[1], hands_stamp or stamp)
        self.leg_load('left', -0.5, 0.15, legs[0], stamp)
        self.leg_load('right', -0.5, -0.15, legs[1], stamp)

    def close(self):
        self.executor.remove_node(self.node)
        self.executor.remove_node(self.helper)
        self.helper.destroy_node()
        self.node.destroy_node()
        rclpy.try_shutdown()


@pytest.fixture
def rig():
    made = []

    def make(*args):
        made.append(Rig(*args))
        return made[-1]
    yield make
    for r in made:
        r.close()


def stamp_of(msg):
    return msg.pose.header.stamp.sec + msg.pose.header.stamp.nanosec * 1e-9


def test_nothing_is_published_until_the_four_contacts_are_known(rig):
    r = rig()
    r.hand_load('left', 5, 10.0)
    r.hand_load('right', 5, 10.0)
    r.leg_load('left', -0.5, 0.15, 30, 10.0)
    r.settle()
    assert r.out == []
    r.leg_load('right', -0.5, -0.15, 30, 10.0)
    r.spin(3.0, lambda: bool(r.out) and bool(r.polygons))
    assert len(r.out) == 1 and len(r.polygons) == 1
    assert r.out[0].contacts == 4 and r.out[0].pose.header.frame_id == 'laser'


def test_it_is_evaluated_with_every_input_message_with_the_latest_of_each(rig):
    r = rig()
    r.all_four(10.0)
    r.spin(3.0, lambda: bool(r.out))
    r.settle()
    r.out.clear()
    r.leg_load('left', -0.5, 0.15, 2, 10.1)          # one leg lets go: a contact less, at once, without waiting for any other input
    r.spin(3.0, lambda: bool(r.out))
    assert len(r.out) == 1 and r.out[0].contacts == 3
    r.out.clear()
    r.hand_load('right', 20, 10.2)
    r.spin(3.0, lambda: bool(r.out))
    assert len(r.out) == 1 and r.out[0].contacts == 3
    r.out.clear()
    r.leg_load('left', -0.5, 0.15, 30, 10.3)
    r.spin(3.0, lambda: bool(r.out))
    assert len(r.out) == 1 and r.out[0].contacts == 4
    r.settle()
    assert len(r.out) == 1


def test_the_stamp_is_the_data_time_both_legs_have_reached_and_never_goes_back(rig):
    r = rig()
    stamps = []

    def stamp_after(push):
        r.out.clear()
        push()
        r.spin(3.0, lambda: bool(r.out))
        stamps.append(stamp_of(r.out[-1]))

    # the handles' stamps are the newer ones, as in the recorded bags, and do not enter: the stamp is the older of the legs' newest
    stamp_after(lambda: r.all_four(10.0, hands_stamp=10.4))
    stamp_after(lambda: r.leg_load('left', -0.5, 0.15, 25, 10.2))     # the right leg is still at 10.0
    stamp_after(lambda: r.leg_load('right', -0.5, -0.15, 25, 10.3))
    stamp_after(lambda: r.hand_load('left', 4, 10.5))                 # a newer input does not move it
    stamp_after(lambda: r.leg_load('left', -0.5, 0.15, 25, 10.6))
    stamp_after(lambda: r.hand_load('right', 4, 9.0))                 # nor does a handle that lags
    assert stamps == pytest.approx([10.0, 10.0, 10.2, 10.2, 10.3, 10.3])


def test_the_low_pass_advances_with_the_stamps_and_not_with_the_messages(rig):
    r = rig()
    r.all_four(10.0)
    r.spin(3.0, lambda: bool(r.out))
    r.settle()
    before = r.out[-1].pose.pose.position.x
    r.out.clear()
    # a leg moves forward and the polygon changes at once, but the filtered centre of mass only moves as the stamps advance
    r.leg_load('left', 0.5, 0.15, 30, 10.0)
    r.spin(3.0, lambda: bool(r.out))
    same_stamp = r.out[-1].pose.pose.position.x
    assert same_stamp == pytest.approx(before)
    r.out.clear()
    r.hand_load('left', 5, 12.0)
    r.hand_load('right', 5, 12.0)
    r.leg_load('left', 0.5, 0.15, 30, 12.0)
    r.leg_load('right', -0.5, -0.15, 30, 12.0)
    r.spin(3.0, lambda: len(r.out) >= 4)
    assert stamp_of(r.out[-1]) == pytest.approx(12.0) and r.out[-1].pose.pose.position.x > same_stamp


def test_stamp_ordered_holds_the_same_inputs_back_and_stamps_with_the_tick(rig):
    r = rig('-p', 'stamp_ordered:=true')
    r.all_four(10.0)
    r.settle()
    assert r.out == []                               # immediate mode publishes here: the sequencer still waits for later stamps
    r.all_four(10.3)
    r.all_four(10.6)
    r.spin(3.0, lambda: bool(r.out))
    r.settle()
    assert r.out and stamp_of(r.out[0]) == pytest.approx(10.05)
