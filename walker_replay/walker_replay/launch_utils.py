"""The way other launchers add the replay of a definitive bag.

    from walker_replay.launch_utils import replay_actions

    def generate_launch_description():
        ld = LaunchDescription([...your declared arguments...])
        # BEFORE your nodes: use_sim_time reaches what comes after it
        ld.add_action(replay_actions(bag_name=LaunchConfiguration('bag_name'),
                                     rate=LaunchConfiguration('rate'),
                                     topics='/scan /left_handle /right_handle'))
        ld.add_action(Node(...))     # gets use_sim_time:=true
        return ld

Every argument of replay.launch.py is accepted, as a plain value or as a
substitution. The arguments of the replay launch file stay inside the group: they
do not leak into the caller's launch configurations (rate, bag_name... are very
common names). What does leave the group, on purpose, is use_sim_time.
"""
from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter

ARGUMENTS = ('bag_folder', 'bag_name', 'rate', 'topics', 'exclude_topics', 'start_offset',
             'duration', 'delay', 'remap', 'loop', 'clock', 'clock_hz', 'sim_time', 'debug',
             'shutdown_on_end', 'publish_context', 'publish_tf')


def replay_actions(**arguments):
    """List of launch actions: simulated time propagation + the replay module.

    Raises TypeError for an argument the replay launch file does not have.
    """
    unknown = sorted(set(arguments) - set(ARGUMENTS))
    if unknown:
        raise TypeError(f"replay_actions: unknown argument(s) {unknown}; valid: {list(ARGUMENTS)}")

    launch_arguments = {}
    for name, value in arguments.items():
        if isinstance(value, (list, tuple)):
            value = ' '.join(str(v) for v in value)
        elif isinstance(value, bool):
            value = 'true' if value else 'false'
        launch_arguments[name] = value if not isinstance(value, (int, float)) else str(value)

    sim_time = launch_arguments.get('sim_time', 'true')
    return [
        # outside the group: what comes after the replay module in the caller runs on bag time
        SetLaunchConfiguration('use_sim_time', sim_time),
        SetParameter(name='use_sim_time', value=sim_time),
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(join(
                get_package_share_directory('walker_replay'), 'launch', 'replay.launch.py')),
            launch_arguments=launch_arguments.items())]),
    ]
