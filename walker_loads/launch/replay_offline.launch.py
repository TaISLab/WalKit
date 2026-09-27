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
            {'fixed_frame_active_area_x': [-0.75, 0.4]},
            {'fixed_frame_active_area_y': [-0.4, 0.4]},
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
