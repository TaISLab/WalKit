import os
import sys

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from walker_msgs.msg import ForceStamped, StabilityStamped, StepStamped, WalkStats

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from walker_rviz import colors, markers  # noqa: E402
from walker_rviz.active_area import boxes_from_yaml  # noqa: E402


def force(kg, frame='left_handle_id'):
    m = ForceStamped()
    m.header.frame_id, m.header.stamp = frame, Time(sec=12, nanosec=500)
    m.force = float(kg)
    return m


def step(x, y, load, confidence=1.0, frame='laser'):
    m = StepStamped()
    m.position.header.frame_id, m.position.header.stamp = frame, Time(sec=7, nanosec=1)
    m.position.point.x, m.position.point.y = x, y
    m.load, m.confidence = float(load), float(confidence)
    return m


def test_jet_runs_from_blue_to_red_and_red_to_green_is_the_stability_scale():
    assert colors.jet(0.0)[2] > 0.4 and colors.jet(0.0)[0] == 0.0
    assert colors.jet(1.0)[0] > 0.4 and colors.jet(1.0)[2] == 0.0
    assert colors.jet(-5.0) == colors.jet(0.0) and colors.jet(7.0) == colors.jet(1.0)
    assert colors.red_to_green(0.0) == (1.0, 0.0, 0.0) and colors.red_to_green(1.0) == (0.0, 1.0, 0.0)


def test_handle_arrow_ends_at_the_handle_points_down_and_grows_with_the_load():
    light, heavy = (markers.handle_markers(force(kg), markers.LEFT, max_kg=20.0)[0] for kg in (2.0, 18.0))
    assert light.type == Marker.ARROW and light.header.frame_id == 'left_handle_id' and light.header.stamp.sec == 12
    assert (light.points[1].x, light.points[1].y, light.points[1].z) == (0.0, 0.0, 0.0)       # the tip is at the handle
    assert light.points[0].z < heavy.points[0].z                                              # starts higher the more it carries
    assert light.color.b > light.color.r and heavy.color.r > heavy.color.b


def test_handle_labels_say_the_kg_and_the_sides_have_their_own_ids():
    left_arrow, left_text = markers.handle_markers(force(7.26), markers.LEFT)
    right_arrow, right_text = markers.handle_markers(force(1.0), markers.RIGHT)
    assert left_text.text == '7.3 kg' and left_text.type == Marker.TEXT_VIEW_FACING
    assert left_arrow.id != right_arrow.id and left_text.id != right_text.id
    assert left_arrow.ns == right_arrow.ns


def test_negative_loads_do_not_break_the_handle_arrow():
    arrow = markers.handle_markers(force(-3.0), markers.RIGHT)[0]
    assert arrow.points[0].z == pytest.approx(0.05)


def test_step_marker_sits_on_the_floor_of_its_frame_and_left_is_a_sphere_right_a_cube():
    (left, left_text), (right, _) = (markers.step_markers(step(-0.6, 0.1, 30.0), side) for side in (markers.LEFT, markers.RIGHT))
    assert left.type == Marker.SPHERE and right.type == Marker.CUBE
    assert (left.pose.position.x, left.pose.position.y) == (-0.6, 0.1) and left.pose.position.z < 0.01
    assert left.header.frame_id == 'laser' and left.header.stamp.sec == 7
    assert left_text.text == 'left 30 kg'


def test_step_colour_follows_the_share_of_the_weight_and_transparency_the_confidence():
    light = markers.step_markers(step(0, 0, 3.0, confidence=0.5), markers.LEFT, weight_kg=60.0)[0]
    heavy = markers.step_markers(step(0, 0, 55.0), markers.LEFT, weight_kg=60.0)[0]
    assert light.color.a == pytest.approx(0.5) and heavy.color.a == pytest.approx(1.0)
    assert heavy.color.r > light.color.r
    assert markers.step_markers(step(0, 0, 10.0), markers.LEFT, weight_kg=0.0)[0].color.a > 0.0     # no weight known: no crash


def test_detected_steps_have_a_fixed_colour_per_leg():
    left = markers.detected_step_markers(step(0, 0, -1.0), markers.LEFT)[0]
    right = markers.detected_step_markers(step(0, 0, -1.0), markers.RIGHT)[0]
    assert (left.color.r, left.color.g, left.color.b) != (right.color.r, right.color.g, right.color.b)


def test_centroid_and_stability_markers():
    point = PointStamped()
    point.header.frame_id = 'laser'
    point.point.x, point.point.y = -0.7, 0.05
    (c,) = markers.centroid_marker(point)
    assert c.header.frame_id == 'laser' and (c.pose.position.x, c.pose.position.y) == (-0.7, 0.05)

    stab = StabilityStamped()
    stab.pose.header.frame_id = 'laser'
    stab.pose.pose.position.x = -0.75
    stab.sta, stab.margin, stab.contacts = 0.8, 0.083, 4
    body, label = markers.stability_markers(stab)
    assert body.color.g > body.color.r and body.header.frame_id == 'laser'
    assert label.text == 'margin +8.3 cm | sta 0.80 | 4 contacts'
    stab.sta, stab.margin = 0.0, -0.02
    assert markers.stability_markers(stab)[0].color.r == 1.0


def test_gait_text_is_in_the_frame_asked_for_and_does_not_touch_the_message():
    stats = WalkStats()
    stats.header.frame_id = 'odom'
    stats.nos, stats.cad, stats.wv, stats.d = 42, 97.5, 0.43, 12.3
    (text,) = markers.gait_marker(stats, 'base_link')
    assert text.header.frame_id == 'base_link' and stats.header.frame_id == 'odom'
    assert 'steps 42' in text.text and '0.43 m/s' in text.text


def test_boxes_become_closed_outlines_in_their_frame():
    box = {'frame': 'laser', 'min_x': -1.2, 'max_x': 0.03, 'min_y': -0.3, 'max_y': 0.3}
    (outline,) = markers.box_markers([box], Time())
    assert outline.type == Marker.LINE_STRIP and outline.header.frame_id == 'laser'
    assert len(outline.points) == 5 and (outline.points[0].x, outline.points[0].y) == (outline.points[-1].x, outline.points[-1].y)
    assert outline.lifetime.sec == 0 and outline.lifetime.nanosec == 0


def test_boxes_are_read_from_the_filters_own_yaml(tmp_path):
    f = tmp_path / 'box.yaml'
    f.write_text("""
/**:
  ros__parameters:
    filter1:
      name: active_area
      params:
        box_frame: laser
        max_x: 0.03
        max_y: 0.3
        min_x: -1.2
        min_y: -0.3
""")
    assert boxes_from_yaml(str(f)) == [{'frame': 'laser', 'min_x': -1.2, 'max_x': 0.03, 'min_y': -0.3, 'max_y': 0.3}]
    other = tmp_path / 'none.yaml'
    other.write_text('node:\n  ros__parameters:\n    a: 1\n')
    assert boxes_from_yaml(str(other)) == []


def test_handle_calibration_interpolates_the_table_and_goes_on_beyond_its_ends(tmp_path):
    from walker_rviz.handle_calibration import HandleCalibration
    cal = HandleCalibration([0.0, 10.0, 20.0], [0.0, 100.0, 300.0], [10.0, 110.0, 310.0])
    assert cal.kg('left', 0.0) == 0.0 and cal.kg('left', 100.0) == 10.0 and cal.kg('left', 300.0) == 20.0
    assert cal.kg('left', 200.0) == pytest.approx(15.0) and cal.kg('right', 60.0) == pytest.approx(5.0)
    assert cal.kg('left', -10.0) == pytest.approx(-1.0)               # extrapolated, like the chain's spline: a little below 0
    assert cal.kg('left', 400.0) == pytest.approx(25.0)
    with pytest.raises(ValueError):
        HandleCalibration([0.0, 1.0], [5.0, 5.0], [0.0, 1.0])
    f = tmp_path / 'c.yaml'
    f.write_text('partial_loads:\n  ros__parameters:\n    weight_points: [0.0, 2.0]\n    left_handle_points: [1.0, 3.0]\n    right_handle_points: [0.0, 4.0]\n')
    assert HandleCalibration.from_yaml(str(f)).kg('right', 2.0) == pytest.approx(1.0)
    real = os.path.join(os.path.dirname(__file__), '..', '..', 'walker_loads', 'config', 'handle_calib.yaml')
    if os.path.exists(real):
        assert 0.0 < HandleCalibration.from_yaml(real).kg('left', 255.0) < 9.0
