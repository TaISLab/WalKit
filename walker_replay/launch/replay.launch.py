#!/usr/bin/env python3

"""
Single way of replaying the definitive Walkit rosbags.

Whoever replays a bag must not design anything about it: add this module to the
launcher (see walker_replay.launch_utils.replay_actions, or include this file) and
the system receives exactly the recorded data, with its time.

What it does:
  * plays <bag_folder>/<bag_name> with `ros2 bag play`, publishing /clock;
  * publishes (latched) what a recording of the walker does not carry, built from
    <bag_folder>/test_config.txt and the URDF: /user_desc, /handle_height,
    /robot_description (the URDF, from walker_state_publisher) and the walker's
    static tf (URDF through the same state publisher, handles from the
    handle_height of the test);
  * propagates use_sim_time:=true (global parameter and `use_sim_time` launch
    configuration) to everything launched after it in the same launch description.
    Put it BEFORE your nodes: a global parameter only reaches what comes after.
    Nodes launched with ExecuteProcess (python script --ros-args) are not
    reached: pass them `-p use_sim_time:=true` yourself.

Only definitive bags are replayed (folder with definitive_log.md); anything else is
refused. They carry no latched topic and no QoS profile, so no qos override is
needed (the old tf_static override only existed for the original, non-definitive,
bags that carried /tf_static).

Arguments:
  bag_folder        base folder: where the bag folders and test_config.txt live.
  bag_name          bag to play (MF_test07, CA_test01, ...).
  rate              playback rate.
  topics            topics to play, space separated; empty = all the recorded ones
                    (but /rosout and /parameter_events, unless named here).
  exclude_topics    topics not to play, space separated.
  start_offset      seconds into the bag (bag time, not affected by rate).
  duration          seconds of bag to play from start_offset; empty = until the end.
  delay             seconds to wait before playing, to let the consumers come up.
  remap             "old:=new old2:=new2" topic remappings of the played topics.
  loop              play over and over.
  clock             publish /clock (default true).
  clock_hz          /clock frequency, in wall-clock Hz (see the argument).
  sim_time          propagate use_sim_time:=true (default true). It needs clock.
  debug             log at debug level, and print the player command.
  shutdown_on_end   shut the whole launch down when the bag ends.
  publish_context   publish /user_desc and /handle_height.
  publish_tf        publish the walker's static tf (URDF and handles) and
                    /robot_description: they come from the same state publisher.

Usage:
    ros2 launch walker_replay replay.launch.py bag_name:=MF_test07 rate:=1.0
"""

import shlex
from os.path import expanduser, join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, SetLaunchConfiguration, Shutdown)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from walker_replay.bag import resolve_bag, topic_selection

NAMESPACE = 'walker_replay'
DEFAULT_BAG_FOLDER = '~/bagsFolder/definitive'


def _is_true(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ('true', '1', 'yes', 'on')


def _words(context, name):
    return LaunchConfiguration(name).perform(context).split()


def _setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context).strip()   # noqa: E731

    clock, sim_time, debug = (_is_true(context, n) for n in ('clock', 'sim_time', 'debug'))
    if sim_time and not clock:
        raise RuntimeError(
            "sim_time:=true needs clock:=true: the nodes would wait for a /clock nobody publishes. "
            "Use sim_time:=false clock:=false only to replay on wall time (the stamps of anything "
            "the nodes create will not be bag time)")

    rate = float(arg('rate'))
    if rate <= 0.0:
        raise RuntimeError(f"rate must be > 0, got {rate}")

    bag_folder = expanduser(arg('bag_folder'))
    bag = resolve_bag(bag_folder, arg('bag_name'))
    topics, exclude, missing = topic_selection(bag, _words(context, 'topics'),
                                               _words(context, 'exclude_topics'))

    actions = []
    if missing:
        actions.append(LogInfo(msg=f"[walker_replay] WARNING: {bag.name} does not record {missing}"))

    # Propagation of simulated time: launch configuration for the include-d launchers
    # (walker_state_publisher and anything else that declares use_sim_time) and
    # global parameter for the nodes launched after this module.
    if sim_time:
        actions += [SetLaunchConfiguration('use_sim_time', 'true'),
                    SetParameter(name='use_sim_time', value=True)]

    log_level = 'debug' if debug else 'warn'
    ros_args = ['--ros-args', '--log-level', log_level]

    if _is_true(context, 'publish_tf'):
        # /robot_description (latched) and base_footprint -> laser, imu, wheels...: from the
        # URDF, both published by robot_state_publisher. The witmotion frame is the
        # URDF one (z up); the /tf_static that was recorded in the original bags had it with a
        # 180 degrees roll that was a mistake (WalKit 8f1e8a3), and is not in the definitive ones.
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(join(get_package_share_directory('walker_description'),
                                               'launch', 'walker_state_publisher.launch.py')),
            # walker_state_publisher declares log_level=DEBUG by default and the arguments of an
            # include leak to the parent scope: say it explicitly or everything launched
            # afterwards inherits --log-level DEBUG.
            launch_arguments={'log_level': log_level,
                              'use_sim_time': 'true' if sim_time else 'false'}.items()))

    actions.append(Node(
        package='walker_replay', executable='replay_context.py', namespace=NAMESPACE,
        name='context', output='screen', arguments=ros_args,
        parameters=[{'bag_folder': bag_folder,
                     'bag_name': bag.name,
                     'publish_context': _is_true(context, 'publish_context'),
                     'publish_tf': _is_true(context, 'publish_tf')}]))

    # --- the player. Added last, so the nodes above are already up.
    cmd = ['ros2', 'bag', 'play', bag.path, '--disable-keyboard-controls',
           '--rate', str(rate)]
    if clock:
        cmd += ['--clock', arg('clock_hz')]
    if topics:
        cmd += ['--topics'] + topics
    if exclude:
        cmd += ['--exclude-topics'] + exclude
    if float(arg('start_offset')) > 0.0:
        cmd += ['--start-offset', arg('start_offset')]
    if arg('duration'):
        cmd += ['--playback-duration', arg('duration')]
    if float(arg('delay')) > 0.0:
        cmd += ['--delay', arg('delay')]
    if _words(context, 'remap'):
        cmd += ['--remap'] + _words(context, 'remap')
    if _is_true(context, 'loop'):
        cmd += ['--loop']
    if debug:
        cmd += ['--log-level', 'debug']
        actions.append(LogInfo(msg='[walker_replay] ' + ' '.join(shlex.quote(c) for c in cmd)))

    actions.append(LogInfo(
        msg=f"[walker_replay] {bag.name}: {bag.duration_s:.1f} s, {len(bag.topics)} topics, "
            f"rate {rate}, clock {'%s Hz' % arg('clock_hz') if clock else 'off'}, "
            f"use_sim_time {sim_time}, handle_height {bag.config.handle_height}, "
            f"user {bag.config.user_id}"))

    actions.append(ExecuteProcess(
        cmd=cmd, output='screen', name='bag_play',
        on_exit=[Shutdown(reason='bag finished')] if _is_true(context, 'shutdown_on_end') else None))
    return actions


def generate_launch_description():
    ld = LaunchDescription()

    def declare(name, default, description):
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))

    declare('bag_folder', expanduser(DEFAULT_BAG_FOLDER),
            'base folder: definitive bag folders and test_config.txt')
    declare('bag_name', 'MF_test01', 'definitive bag to replay (MF_test07, CA_test01...)')
    declare('rate', '1.0', 'playback rate')
    declare('topics', '', 'topics to play, space separated; empty = all the recorded ones')
    declare('exclude_topics', '', 'topics not to play, space separated')
    declare('start_offset', '0.0', 'seconds into the bag (bag time) to start at')
    declare('duration', '', 'seconds of bag to play from start_offset; empty = until the end')
    declare('delay', '3.0', 'seconds to wait before playing, so the consumers are up')
    declare('remap', '', '"old:=new old2:=new2" remappings of the played topics')
    declare('loop', 'false', 'play the bag over and over')
    declare('clock', 'true', 'publish /clock')
    declare('clock_hz', '200',
            'frequency (wall-clock Hz) of /clock. ros2 bag play publishes at 40 Hz by default and '
            'timers on simulated time (the EKF 30 Hz one) fire once per clock tick only: at 40 Hz '
            'the EKF output fell to ~14-19 Hz and changed with the playback rate; at 200 Hz it is '
            '~30 Hz whatever the rate')
    declare('sim_time', 'true', 'propagate use_sim_time:=true to what is launched after this module')
    declare('debug', 'false', 'log at debug level and print the player command')
    declare('shutdown_on_end', 'false', 'shut the whole launch down when the bag ends')
    declare('publish_context', 'true', 'publish /user_desc and /handle_height from test_config.txt')
    declare('publish_tf', 'true', 'publish the walker static tf (URDF and handles) and /robot_description')

    ld.add_action(OpaqueFunction(function=_setup))
    return ld
