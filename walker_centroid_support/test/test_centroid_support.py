import os
import sys
import time

import pytest
import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.executors import SingleThreadedExecutor
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from walker_msgs.msg import ForceStamped, StepStamped

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from walker_centroid_support.centroid_support import CentroidSupport  # noqa: E402

LEFT_HANDLE, RIGHT_HANDLE = (0.1, 0.25, 0.9), (0.1, -0.25, 0.9)


class Rig:
    """The node, and a helper node that plays the walker_loads outputs and the tf of the handles."""

    def __init__(self, *args):
        rclpy.init(args=['--ros-args', *args])
        self.node = CentroidSupport()
        self.helper = rclpy.create_node('centroid_support_test_helper')
        pub = lambda t, topic: self.helper.create_publisher(t, topic, 10)  # noqa: E731
        self.hand = {'left': pub(ForceStamped, '/left_hand_loads'), 'right': pub(ForceStamped, '/right_hand_loads')}
        self.leg = {'left': pub(StepStamped, '/left_loads'), 'right': pub(StepStamped, '/right_loads')}
        self.out = []
        self.helper.create_subscription(PointStamped, '/support_centroid', self.out.append, 100)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.helper)
        broadcaster = StaticTransformBroadcaster(self.helper)
        broadcaster.sendTransform([self.transform('left_handle', LEFT_HANDLE), self.transform('right_handle', RIGHT_HANDLE)])
        self.spin(10.0, lambda: all(p.get_subscription_count() for p in [*self.hand.values(), *self.leg.values()])
                  and self.node.buffer.can_transform('laser', 'right_handle', rclpy.time.Time()))

    @staticmethod
    def transform(child, xyz):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'laser', child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
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

    def all_four(self, stamp, hands=(5, 5), legs=(30, 10), hands_stamp=None):
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
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def test_nothing_is_published_until_the_four_contacts_are_known(rig):
    r = rig()
    r.hand_load('left', 5, 10.0)
    r.hand_load('right', 5, 10.0)
    r.leg_load('left', -0.5, 0.15, 30, 10.0)
    r.settle()
    assert r.out == []
    r.leg_load('right', -0.5, -0.15, 10, 10.0)
    r.spin(3.0, lambda: bool(r.out))
    assert len(r.out) == 1


def test_it_is_published_with_every_input_message_with_the_latest_of_each(rig):
    r = rig()
    r.all_four(10.0)
    r.spin(3.0, lambda: bool(r.out))
    first = r.out[-1]
    # (30 * -0.5 + 10 * -0.5 + 5 * 0.1 + 5 * 0.1) / 50 = -0.38 in x; the legs pull to the left in y: (30 - 10) * 0.15 / 50 + 0 = 0.06
    assert (round(first.point.x, 4), round(first.point.y, 4)) == (-0.38, 0.06)
    assert first.header.frame_id == 'laser'
    r.settle()
    r.out.clear()
    r.hand_load('left', 15, 10.4)                    # one handle changes: out at once, without waiting for any other input
    r.spin(3.0, lambda: bool(r.out))
    assert len(r.out) == 1
    assert r.out[0].point.x == pytest.approx((30 * -0.5 + 10 * -0.5 + 15 * 0.1 + 5 * 0.1) / 60)
    r.out.clear()
    r.leg_load('right', -0.5, -0.15, 40, 10.5)       # and so does a leg
    r.spin(3.0, lambda: bool(r.out))
    assert len(r.out) == 1 and r.out[0].point.y == pytest.approx((30 * 0.15 - 40 * 0.15 + 15 * 0.25 - 5 * 0.25) / 90)
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
        assert r.out[-1].header.frame_id == 'laser'

    # the handles' stamps are the newer ones, as in the recorded bags, and do not enter: the stamp is the older of the legs' newest
    stamp_after(lambda: r.all_four(10.0, hands_stamp=10.4))
    stamp_after(lambda: r.leg_load('left', -0.5, 0.15, 25, 10.2))     # the right leg is still at 10.0
    stamp_after(lambda: r.leg_load('right', -0.5, -0.15, 25, 10.3))
    stamp_after(lambda: r.hand_load('left', 4, 10.5))                 # a newer input does not move it
    stamp_after(lambda: r.leg_load('left', -0.5, 0.15, 25, 10.6))
    stamp_after(lambda: r.hand_load('right', 4, 9.0))                 # nor does a handle that lags
    assert stamps == pytest.approx([10.0, 10.0, 10.2, 10.2, 10.3, 10.3])


def test_the_stamp_does_not_alter_the_input_messages_stamps(rig):
    r = rig()
    received = []
    r.helper.create_subscription(StepStamped, '/right_loads', received.append, 10)
    r.all_four(10.0, hands_stamp=10.4)
    r.spin(3.0, lambda: bool(r.out) and bool(received))
    assert stamp_of(r.out[-1]) == pytest.approx(10.0)
    assert received[-1].position.header.stamp.sec == 10 and received[-1].position.header.stamp.nanosec == 0


def test_stamp_ordered_holds_the_same_inputs_back_and_publishes_the_state_as_of_a_tick(rig):
    r = rig('-p', 'stamp_ordered:=true')
    r.all_four(10.0, legs=(30, 10))
    r.settle()
    assert r.out == []                               # immediate mode publishes here: the sequencer still waits for later stamps
    r.all_four(10.3, legs=(10, 30))
    r.all_four(10.6, legs=(10, 30))
    r.spin(3.0, lambda: bool(r.out))
    r.settle()
    assert r.out and stamp_of(r.out[0]) == pytest.approx(10.0)
    assert r.out[0].point.y == pytest.approx(0.06)   # the state of the first round, not the latest one
