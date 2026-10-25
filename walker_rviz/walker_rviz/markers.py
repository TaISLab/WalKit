"""Builders of the markers walker_rviz draws, one function per kind of data. They only need the message classes (no node,
no clock): the header stamp and frame of every marker are those of the message it draws, so the picture follows the data
(a replayed bag, at any rate, looks the same) and each marker is drawn in the frame its data is expressed in.

Layers and what they show:
  handles    an arrow down from each handle, as long and as red as the (calibrated) load it carries, with the kg as text
  steps      each leg's tracked foot on the floor (sphere = left, cube = right), coloured by the share of the user's weight
             it carries, with its label
  detected   the same for the steps of a detector that has no load (fixed colour per leg, transparency = confidence)
  centroid   the support centroid
  stability  the estimate of the user's centre of mass, red to green by the stability index, with the margin as text
  gait       the gait summary as text
  area       the boxes of the laser filters, as outlines
"""
import re

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from walker_rviz.colors import jet, red_to_green

LEFT, RIGHT = 'left', 'right'
SIDE_COLOURS = {LEFT: (0.2, 0.6, 1.0), RIGHT: (1.0, 0.6, 0.2)}


def _duration(seconds):
    return Duration(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))


def _marker(header, ns, marker_id, marker_type, lifetime_s):
    m = Marker()
    m.header = header
    m.ns = ns
    m.id = marker_id
    m.type = marker_type
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.lifetime = _duration(lifetime_s)
    return m


def _colour(m, rgb, alpha=1.0):
    m.color.r, m.color.g, m.color.b = (float(c) for c in rgb)
    m.color.a = float(alpha)


def _text(header, ns, marker_id, text, position, lifetime_s, height=0.05, rgb=(1.0, 1.0, 1.0)):
    m = _marker(header, ns, marker_id, Marker.TEXT_VIEW_FACING, lifetime_s)
    m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(p) for p in position)
    m.scale.z = float(height)
    _colour(m, rgb)
    # No spaces, of any kind: this RViz draws each one tens of times too wide, so a label like "left 45 kg" comes out as three words
    # a metre apart. (The labels are written without them; this keeps one added later from bringing the problem back.)
    m.text = re.sub(r'\s+', '_', text)
    return m


def handle_markers(msg, side, max_kg=20.0, lifetime_s=0.5):
    """msg: walker_msgs/ForceStamped (kg, in the handle's own frame). An arrow ending at the handle's origin, pointing
    down, 0.05 to 0.5 m long for 0 to `max_kg`, and its kg."""
    ident = 0 if side == LEFT else 1
    share = max(0.0, float(msg.force)) / max_kg
    length = 0.05 + 0.45 * min(share, 1.0)
    arrow = _marker(msg.header, 'handles', ident, Marker.ARROW, lifetime_s)
    for z in (length, 0.0):
        p = Point()
        p.z = z
        arrow.points.append(p)
    arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.03, 0.06, 0.06
    _colour(arrow, jet(share))
    label = _text(msg.header, 'handles_text', ident, '%.1fkg' % float(msg.force), (0.0, 0.0, length + 0.06), lifetime_s)
    return [arrow, label]


def step_markers(msg, side, weight_kg=60.0, size_m=0.15, lifetime_s=0.5):
    """msg: walker_msgs/StepStamped with a load (kg), in the frame of its position. A sphere (left) or cube (right) on the
    floor, coloured by load / weight, transparent as the detection's confidence, and the leg's label with its load."""
    ident = 0 if side == LEFT else 1
    body = _marker(msg.position.header, 'steps', ident, Marker.SPHERE if side == LEFT else Marker.CUBE, lifetime_s)
    body.pose.position.x, body.pose.position.y, body.pose.position.z = msg.position.point.x, msg.position.point.y, 0.005
    body.scale.x = body.scale.y = body.scale.z = float(size_m)
    share = max(0.0, float(msg.load)) / weight_kg if weight_kg > 0 else 0.0
    _colour(body, jet(share), alpha=max(float(msg.confidence), 0.1))
    label = _text(msg.position.header, 'steps_text', ident, '%s:%.0fkg' % (side, max(float(msg.load), 0.0)),
                  (msg.position.point.x, msg.position.point.y, 0.005 + size_m), lifetime_s)
    return [body, label]


def detected_step_markers(msg, side, size_m=0.05, lifetime_s=0.5):
    """msg: walker_msgs/StepStamped of a step detector. Same shapes as step_markers, fixed colour per leg."""
    ident = 0 if side == LEFT else 1
    body = _marker(msg.position.header, 'detected_steps', ident, Marker.SPHERE if side == LEFT else Marker.CUBE, lifetime_s)
    body.pose.position.x, body.pose.position.y, body.pose.position.z = msg.position.point.x, msg.position.point.y, 0.0
    body.scale.x = body.scale.y = body.scale.z = float(size_m)
    _colour(body, SIDE_COLOURS[side], alpha=max(float(msg.confidence), 0.1))
    return [body]


def centroid_marker(msg, size_m=0.08, lifetime_s=0.5):
    """msg: geometry_msgs/PointStamped (the support centroid, on the floor of its frame)."""
    m = _marker(msg.header, 'centroid', 0, Marker.SPHERE, lifetime_s)
    m.pose.position.x, m.pose.position.y, m.pose.position.z = msg.point.x, msg.point.y, 0.01
    m.scale.x = m.scale.y = m.scale.z = float(size_m)
    _colour(m, (1.0, 0.0, 1.0))
    return [m]


def stability_markers(msg, size_m=0.1, lifetime_s=0.5):
    """msg: walker_msgs/StabilityStamped. The centre of mass estimate (pose, in the frame of its header) as a sphere,
    red to green by `sta`, and 'margin cm | sta | contacts' above it."""
    header = msg.pose.header
    body = _marker(header, 'stability', 0, Marker.SPHERE, lifetime_s)
    body.pose.position.x, body.pose.position.y, body.pose.position.z = msg.pose.pose.position.x, msg.pose.pose.position.y, 0.02
    body.scale.x = body.scale.y = body.scale.z = float(size_m)
    _colour(body, red_to_green(msg.sta))
    label = _text(header, 'stability_text', 0, 'margin:%+.1fcm|sta:%.2f|contacts:%d' % (100.0 * msg.margin, msg.sta, msg.contacts),
                  (msg.pose.pose.position.x, msg.pose.pose.position.y, 0.02 + size_m + 0.04), lifetime_s)
    return [body, label]


def gait_marker(msg, frame_id, position=(0.0, 0.0, 1.3), lifetime_s=5.0):
    """msg: walker_msgs/WalkStats. A text with the gait summary, floating at `position` of `frame_id`."""
    text = 'steps:%d|cadence:%.0f/min|speed:%.2fm/s|distance:%.1fm' % (msg.nos, msg.cad, msg.wv, msg.d)
    header = Header()
    header.stamp, header.frame_id = msg.header.stamp, frame_id
    return [_text(header, 'gait', 0, text, position, lifetime_s, height=0.08, rgb=(0.9, 0.9, 0.2))]


def box_markers(boxes, stamp):
    """boxes: [{'frame': str, 'min_x', 'max_x', 'min_y', 'max_y'}] -> one outline each, drawn once (lifetime 0)."""
    out = []
    for i, box in enumerate(boxes):
        m = Marker()
        m.header.frame_id = box['frame']
        m.header.stamp = stamp
        m.ns = 'active_area'
        m.id = i
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.01
        _colour(m, (1.0, 0.5, 0.5))
        for x, y in ((box['min_x'], box['min_y']), (box['min_x'], box['max_y']), (box['max_x'], box['max_y']),
                     (box['max_x'], box['min_y']), (box['min_x'], box['min_y'])):
            p = Point()
            p.x, p.y, p.z = float(x), float(y), 0.0
            m.points.append(p)
        out.append(m)
    return out
