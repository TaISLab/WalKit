#!/usr/bin/python3

"""Reproduce the walker_loads pipeline offline against a definitive rosbag
(~/bagsFolder/definitive): partial_loads -> walker_diff_odom ->
gait_monitor_speed, end to end, so its published GaitStamped/WalkStats can
be compared against the mocap-derived reference computed by
walker_mocap_eval (see WalKit/walker_mocap_eval/readme.md, punto 2 del plan
de validacion).

The replay itself (the bag, /clock, use_sim_time, /user_desc,
/handle_height and the walker's tf tree) is walker_replay's: this launch
only adds the nodes under study. What walker_replay builds from
<bag_folder>/test_config.txt, a ':' separated csv with columns
[test]:[handle_height]:[age]:[height]:[weight]:[user_id]:[gender]:[notes]:
  - /user_desc  <- columns 3-8, published latched (transient_local) as a
    walker_msgs/msg/UserDesc, since partial_loads only reads it to update
    the tracked weight and doesn't need a live stream. test_config.txt has
    no Tinetti column, so tinetti_score is always published as 0.
  - /handle_height (column 2) and the left/right handle frames derived
    from it (walker_description/config/params.yaml), also latched.

The rest of the pipeline mirrors partial_loads.launch.py +
step_detector_km.launch.py.

Leg detection needs a scan cropped to where the legs can appear (km_detect_
steps does no distance filtering of its own): originally this launched
km_detect_steps straight on a /scan_filtered assumed to be already in the
bag, but the bags only have raw /scan -- there is no onboard filter
recorded. Fixed the same way walker_step_detector/launch/
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

Arguments: bag_folder / bag_name (the definitive bag and its test_config.txt),
rate, start_offset, sim_time (true: the pipeline runs on bag time, which
gait_monitor_speed needs for its stamps), debug. The launch shuts down when the
bag ends.

Usage:
    ros2 launch walker_loads replay_offline.launch.py bag_name:=MF_test05 rate:=4.0
"""
from os.path import expanduser, join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from walker_replay.launch_utils import replay_actions

MOCAP_ODOM_BRIDGE = "/home/mfcarmona/workspace/walker_ws/src/WalKit/walker_mocap_eval/scripts/mocap_odom_bridge.py"

# What the pipeline consumes of the bag (the rest of the recording is not played).
TOPICS = ['/left_handle', '/right_handle', '/scan', '/rigid_bodies']


def generate_launch_description():
    walker_loads_dir = get_package_share_directory('walker_loads')
    box_filter_cfg_eval = join(
        get_package_share_directory('walker_step_detector'), 'config', 'box_filter_eval.yaml')
    sim_time = LaunchConfiguration('sim_time')

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'bag_folder', default_value=expanduser('~/bagsFolder/definitive'),
            description='Base folder of the definitive bags (and test_config.txt)'),
        DeclareLaunchArgument(
            'bag_name', default_value='MF_test05', description='Definitive bag to replay'),
        DeclareLaunchArgument(
            'rate', default_value='1.0', description='Rosbag playback rate'),
        DeclareLaunchArgument(
            'start_offset', default_value='0.0',
            description='Seconds into the bag (bag-time) to start playback at -- see /tag note below'),
        DeclareLaunchArgument(
            'sim_time', default_value='true',
            description='use_sim_time for the whole pipeline (bag time). Replaying a bag: true'),
        DeclareLaunchArgument(
            'debug', default_value='false', description='Debug logging of the bag player'),
    ])

    # The replay: /clock, use_sim_time for everything below, /user_desc, /handle_height,
    # /robot_description and the walker's tf (URDF + handles). First, so the global
    # use_sim_time reaches the nodes. Replay only what's needed; the player waits
    # `delay` seconds so the subscribers below are already up when messages start
    # flowing. start_offset (seconds into the bag, bag-time, unaffected by rate)
    # defaults to 0 = whole bag; walker_mocap_eval/scripts/
    # compare_gait_mocap_vs_walker.py passes the first /tag message's offset to
    # skip the pre-test setup/calibration portion, matching mocap_analysis/
    # scripts/merge_rosbags.py's own crop-to-/tag behaviour.
    for action in replay_actions(
            bag_folder=LaunchConfiguration('bag_folder'),
            bag_name=LaunchConfiguration('bag_name'),
            rate=LaunchConfiguration('rate'),
            start_offset=LaunchConfiguration('start_offset'),
            topics=TOPICS,
            sim_time=sim_time,
            debug=LaunchConfiguration('debug'),
            shutdown_on_end=True):
        ld.add_action(action)

    # The bags only have raw /scan (no onboard filter recorded): crop it to the
    # leg-detection zone the same way test_step_detector.launch.py does for these
    # same bags (km_detect_steps does no distance filtering of its own -- see
    # box_filter_eval.yaml's docstring).
    ld.add_action(Node(
        package='walker_laser_filter', executable='box_filter',
        name='step_laser_filter',
        parameters=[box_filter_cfg_eval]))

    # Feet position/speed from the laser: detect_steps (Random Forest), NOT
    # km_detect_steps -- switched on recommendation from the walker_step_
    # detector session ("1.[walker_step_detector][mejoras]"), which has it
    # validated at 95.7-95.9% match_rate over these same 47 bags vs ~81%
    # for km (their eval_against_mocap.py). No `kalman_enabled` param here:
    # unlike km/seg, the RF variant's EKF is always on
    # (kalman_tracker.set_status(true) hardcoded in detect_steps.cpp).
    # detection_threshold/max_detected_clusters are what actually gate the
    # RF's confidence filter (defaults are -1.0/-1 = "no filter") -- this is
    # also expected to fix the CA_test09-style outliers from km's unfiltered
    # k-means (see walker_mocap_eval/readme.md, punto 2).
    ld.add_action(Node(
        package='walker_step_detector', executable='detect_steps',
        name='detect_steps_rf',
        parameters=[
            {'scan_topic': '/scan_feet'},
            {'forest_file': join(
                get_package_share_directory('walker_step_detector'),
                'config', 'trained_step_detector_res_0.33.yaml')},
            {'detected_steps_topic_name': '/detected_step'},
            {'kalman_model_d0': 0.001},
            {'kalman_model_a0': 0.001},
            {'kalman_model_f0': 0.001},
            {'kalman_model_p0': 0.001},
            {'detection_threshold': 0.01},
            {'cluster_dist_euclid': 0.13},
            {'max_detect_distance': 1.25},
            {'max_detected_clusters': 2},
            {'min_points_per_cluster': 3},
            {'publish_clusters': True},
        ]))

    # The node under study.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(walker_loads_dir, 'launch', 'partial_loads.launch.py'))))

    # Weighted centroid of the 4 contact points (2 feet + 2 handles), for
    # comparing against walker_mocap_eval's mocap-based CoM estimate
    # (mocap_eval/cog_estimation.py, De Leva segmental model) -- default
    # topic names already match partial_loads' outputs, no remapping.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('walker_centroid_support'),
                 'launch', 'centroid_support.launch.py'))))

    # /odom for gait_monitor_speed, from mocap ground truth -- see the
    # module docstring above and mocap_odom_bridge.py for why not
    # walker_diff_odom on these bags. Not a launch_ros Node: the global
    # use_sim_time does not reach it, so it is passed explicitly.
    ld.add_action(ExecuteProcess(
        cmd=['python3', MOCAP_ODOM_BRIDGE, '--ros-args',
             '-p', 'rigid_bodies_topic_name:=/rigid_bodies',
             '-p', 'odom_topic_name:=/odom',
             '-p', ['use_sim_time:=', sim_time]],
        output='screen'))

    # The node this whole replay is for: consumes partial_loads' /left_loads,
    # /right_loads and walker_diff_odom's /odom above.
    #
    # use_sim_time (from walker_replay's global parameter): gait_monitor_speed.py
    # stamps /left_gait_stats, /right_gait_stats with self.get_clock().now() (see
    # that file), which on the wall clock is today, not bag time -- found
    # exporting CSVs for walker_mocap_eval's report (their header.stamp was
    # unusable to align against mocap's real timestamps). On sim time,
    # self.get_clock().now() returns bag time instead.
    ld.add_action(Node(
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

    return ld
