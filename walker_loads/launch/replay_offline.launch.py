#!/usr/bin/python3

"""Reproduce the walker_loads pipeline offline against a labeled rosbag
(soma13kp/datasets/labeled_bags) that is missing /user_desc, /handle_height
and part of the walker's tf tree: partial_loads -> walker_diff_odom ->
gait_monitor_speed, end to end, so its published GaitStamped/WalkStats can
be compared against the mocap-derived reference computed by
walker_mocap_eval (see WalKit/walker_mocap_eval/readme.md, punto 2 del plan
de validacion).

Those are reconstructed from bagsFolder_unified/test_config.txt, a ':'
separated csv with columns
[test]:[handle_height]:[age]:[height]:[weight]:[user_id]:[gender]:[notes]:
  - /user_desc  <- columns 3-8 ("age:height:weight:user_id:gender:notes"),
    republished latched (transient_local) as a walker_msgs/msg/UserDesc,
    since partial_loads only reads it to update the tracked weight and
    doesn't need a live stream. test_config.txt has no Tinetti column, so
    tinetti_score is always published as 0.
  - handle_height (column 2) <- passed as a parameter straight into
    walker_description's handle_tf_publisher.py, instead of round-tripping
    it through the /handle_height topic.

The rest of the pipeline mirrors partial_loads.launch.py +
step_detector_km.launch.py, with the walker's static tf tree rebuilt via
walker_description/walker_state_publisher.launch.py and
handle_tf_publisher.py (neither is recorded in these bags).

Leg detection needs a scan cropped to where the legs can appear (km_detect_
steps does no distance filtering of its own): originally this launched
km_detect_steps straight on a /scan_filtered assumed to be already in the
bag, but datasets/labeled_bags only has raw /scan -- there is no onboard
filter recorded. Fixed the same way walker_step_detector/launch/
test_step_detector.launch.py already does for these same bags: an
`inverted_box_filter` (walker_laser_filter/box_filter,
walker_step_detector/config/box_filter_eval.yaml) crops /scan -> /scan_feet
live during replay, and km_detect_steps reads that instead.

/odom for gait_monitor_speed's distance-walked/cadence/velocity output (see
that node's docstring for why this used to be a hardcoded test-track
constant instead of real odometry) does NOT come from walker_diff_odom:
/left_wheel, /right_wheel ARE in these bags, but checked against CA_test01,
/right_wheel has zero messages for the whole test and /left_wheel is
essentially frozen (3475->3476 ticks over ~90s) -- no usable wheel odometry
to validate against here, and fixing/validating that encoder chain is out
of scope for this comparison (being looked at separately). Instead,
mocap_odom_bridge.py (walker_mocap_eval/scripts/) republishes the mocap
ground truth of the walker's own rigid body as /odom: gait_monitor_speed
only ever accumulates consecutive position deltas, which is frame-
invariant, so this is a legitimate stand-in for "some correct odometry
source" without pretending to validate wheel/inertial odometry here too.

Ran this against all 47 labeled_bags (walker_mocap_eval/scripts/
compare_gait_mocap_vs_walker.py, punto 2 del plan): d/CAD/WV now match the
mocap reference closely (median d error ~2cm, ~1% MAPE) -- the odom fix
above is good. But NoS/Tr from the real pipeline were severely undercounted
across EVERY bag (median ~75 fewer steps than mocap, walker_tr_s typically
10-30% of the bag's real duration). Checked specifically whether this was
about "circulos" vs "ida y vuelta" tests (most bags are circular; thought a
fixed-frame active-area clip might be the cause) and it was NOT: LA_test*
(straight back-and-forth) showed the same severe undercount.
`fixed_frame_active_area_x`/`_y`, which this launch file used to pass to
km_detect_steps, turned out to be dead parameters anyway (km_detect_steps.cpp
never reads them) and have been removed here.

**Root cause found and fixed** (walker_step_detector/src/km_detect_steps.cpp):
with `kalman_enabled: true` (needed here -- `kalman_enabled: false` never
sets `StepStamped.tracked`, so partial_loads never sees usable data at all),
`KMDetectSteps::laserCallback()` stamped every predicted StepStamped with
`this->now()` (wall-clock) instead of the scan's own header stamp. Live,
that's harmless (wall-clock ~= sensor time). Replayed at `--rate 4.0`
(this launch file's default), wall-clock runs 4x faster than the bag's own
timeline, so Tr/SpT/SdT -- all computed from these header-stamp
differences -- came out truncated to roughly 1/rate of their true value:
confirmed by rerunning at `rate:=1.0` with the bug still in place (Tr went
from ~12% to ~73% of the true bag duration, nothing else changed) before
touching the C++. Fixed by stamping with `rclcpp::Time(scan->header.stamp)`
instead; rebuilt (`colcon build --packages-select walker_step_detector`)
and reran at rate=4.0: Tr coverage on CA_test01/02/03 went from ~10s to
~50-60s (of ~85-90s), and CA_test03 (previously zero usable output) now
produces data. This is a genuine bug independent of gait_monitor_speed,
this launch file, or the leg tracker's tracking quality -- it only bites
non-real-time bag replay, which is presumably why it was never noticed
before this validation effort.

Reran all 47 bags after the fix (rate=4.0): NOT uniform. 13 bags now match
mocap's NoS closely (MF_test17-20, MT_test11-20 except MT_test13) --
walker NoS 70-150% of mocap's, Tr covering the full bag, SpT/SdT errors
under ~2s. The other 30 (every CA_test*, every LA_test*, MF_test01-15) are
still severely undercounted (walker NoS ~8-23% of mocap's) almost exactly
as before the fix -- so there is a SECOND, separate issue that the
timestamp fix does not touch, and it is bag-specific, not a single global
parameter problem. Checked whether it's a /scan quality difference
(irregular timestamps, gaps, duplicates) between the two groups: it is
NOT -- CA_test01 (bad) and MF_test17 (good) have statistically identical
/scan header-stamp behavior (median gap ~0.12-0.18s, no duplicates, no
non-monotonic stamps, in both groups; even MF_test16, same session and
scan quality as MF_test17, still fails with zero output). So whatever
differs between "good" and "bad" bags happens downstream of /scan, inside
km_detect_steps' clustering/tracking or partial_loads' load assignment --
not chased further here. See the task flagged for walker_step_detector
(was started by the user as a separate session while this investigation
was already in progress here: check both for
overlapping findings before doing more work on it).

Usage:
    ros2 launch walker_loads replay_offline.launch.py \\
        bag_path:=/home/mfcarmona/workspace/walker_ws/src/soma13kp/datasets/labeled_bags/MF_test05
"""
from os.path import basename, join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription, OpaqueFunction, Shutdown)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_BAG = "/home/mfcarmona/workspace/walker_ws/src/soma13kp/datasets/labeled_bags/MF_test05"
DEFAULT_TEST_CONFIG = "/home/mfcarmona/bagsFolder_unified/test_config.txt"
MOCAP_ODOM_BRIDGE = "/home/mfcarmona/workspace/walker_ws/src/WalKit/walker_mocap_eval/scripts/mocap_odom_bridge.py"


def read_test_config_row(test_config_file, bag_name):
    with open(test_config_file) as f:
        for line in f:
            fields = [t.strip() for t in line.strip().split(':')]
            if fields and fields[0] == bag_name:
                return fields
    raise RuntimeError(f"No row for '{bag_name}' in {test_config_file}")


def launch_setup(context, *args, **kwargs):
    bag_path = LaunchConfiguration('bag_path').perform(context)
    test_config_file = LaunchConfiguration('test_config_file').perform(context)
    rate = LaunchConfiguration('rate').perform(context)
    bag_name = basename(bag_path.rstrip('/'))

    fields = read_test_config_row(test_config_file, bag_name)
    handle_height = int(fields[1])
    user_age, user_height, user_weight, user_id, user_gender, user_notes = fields[2:8]
    print(f"[replay_offline] {bag_name}: handle_height={handle_height} "
          f"user='{user_id}' age={user_age} height={user_height} weight={user_weight}")

    walker_description_dir = get_package_share_directory('walker_description')
    walker_loads_dir = get_package_share_directory('walker_loads')
    box_filter_cfg_eval = join(
        get_package_share_directory('walker_step_detector'), 'config', 'box_filter_eval.yaml')
    tf_static_qos_override = join(walker_loads_dir, 'config', 'tf_static_qos_override.yaml')

    actions = []

    # Static tf tree from the URDF (base_footprint -> laser, wheels, ...),
    # not recorded in these bags.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(walker_description_dir, 'launch', 'walker_state_publisher.launch.py'))))

    # Handle tf, reconstructed from test_config.txt's handle_height column.
    actions.append(Node(
        package='walker_description', executable='handle_tf_publisher.py',
        name='handle_tf_publisher',
        parameters=[{'handle_height': handle_height}]))

    # user_desc isn't in the bag: republish it latched, from test_config.txt.
    user_desc_yaml = (
        "{age: " + user_age + ", height: " + user_height + ", weight: " + user_weight +
        ", user_id: '" + user_id + "', gender: '" + user_gender +
        "', tinetti_score: 0, description: '" + user_notes + "'}")
    actions.append(ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--qos-history', 'keep_last', '--qos-depth', '1',
             '--qos-durability', 'transient_local', '--qos-reliability', 'reliable',
             '/user_desc', 'walker_msgs/msg/UserDesc', user_desc_yaml],
        output='log'))

    # datasets/labeled_bags only has raw /scan (no onboard filter recorded):
    # crop it to the leg-detection zone the same way test_step_detector.
    # launch.py does for these same bags (km_detect_steps does no distance
    # filtering of its own -- see box_filter_eval.yaml's docstring).
    actions.append(Node(
        package='walker_laser_filter', executable='box_filter',
        name='step_laser_filter',
        parameters=[box_filter_cfg_eval]))

    # Feet position/speed from the laser (same params as step_detector_km.launch.py).
    actions.append(Node(
        package='walker_step_detector', executable='km_detect_steps',
        name='detect_steps_km',
        parameters=[
            {'scan_topic': '/scan_feet'},
            {'detected_steps_topic_name': '/detected_step'},
            {'detected_steps_frame': 'base_link'},
            {'kalman_enabled': True},
            {'kalman_model_d0': 0.001},
            {'kalman_model_a0': 0.001},
            {'kalman_model_f0': 0.001},
            {'kalman_model_p0': 0.001},
            {'plot_leg_kalman': False},
            {'plot_leg_clusters': False},
            {'use_scan_header_stamp_for_tfs': False},
        ]))

    # The node under study.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(walker_loads_dir, 'launch', 'partial_loads.launch.py'))))

    # /odom for gait_monitor_speed, from mocap ground truth -- see the
    # module docstring above and mocap_odom_bridge.py for why not
    # walker_diff_odom on these bags.
    actions.append(ExecuteProcess(
        cmd=['python3', MOCAP_ODOM_BRIDGE, '--ros-args',
             '-p', 'rigid_bodies_topic_name:=/rigid_bodies',
             '-p', 'odom_topic_name:=/odom'],
        output='screen'))

    # The node this whole replay is for: consumes partial_loads' /left_loads,
    # /right_loads and walker_diff_odom's /odom above.
    actions.append(Node(
        package='walker_loads', executable='gait_monitor_speed.py',
        name='gait_stats',
        parameters=[
            {'period': 0.05},
            {'left_loads_topic_name': '/left_loads'},
            {'right_loads_topic_name': '/right_loads'},
            {'left_gait_stats_topic_name': '/left_gait_stats'},
            {'right_gait_stats_topic_name': '/right_gait_stats'},
            {'global_gait_stats_topic_name': '/global_gait_stats'},
            {'odom_topic_name': '/odom'},
            {'global_frame_id': '/odom'},
        ]))

    # Replay only what's needed, added last so the subscribers above are
    # already up when messages start flowing.
    actions.append(ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-s', 'sqlite3', '--disable-keyboard-controls',
             '--rate', rate, '--qos-profile-overrides-path', tf_static_qos_override,
             bag_path, '--topics', '/left_handle', '/right_handle', '/scan',
             '/rigid_bodies', '/tf', '/tf_static'],
        output='screen',
        on_exit=Shutdown()))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path', default_value=DEFAULT_BAG,
            description='Folder of a labeled rosbag2 (soma13kp/datasets/labeled_bags)'),
        DeclareLaunchArgument(
            'test_config_file', default_value=DEFAULT_TEST_CONFIG,
            description='CSV with handle_height/user_desc per bag (bagsFolder_unified)'),
        DeclareLaunchArgument(
            'rate', default_value='1.0', description='Rosbag playback rate'),
        OpaqueFunction(function=launch_setup),
    ])
