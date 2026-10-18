"""Looking inside a rosbag2 folder, offline: what is in it, its messages, the window between its /tag
messages, and writing a new one.

Replaying a bag is walker_replay's job, and so is reading it without replaying it: this is the one place
that knows how to open a bag (storage plugin, filters, deserialisation), so the tools that analyse bags
(odometry, steps, loads, ... comparisons against the mocap) do not each carry their own copy of it.
Nothing here knows what the topics mean; that stays with whoever asks for them.

rosbag2_py and the message packages are imported when a bag is actually read or written, so importing this
module does not need a sourced ROS environment (the metadata functions only need PyYAML).
"""
from pathlib import Path

import yaml

from walker_replay.bag import BagError

TAG_TOPIC = '/tag'


def _info(bag_dir):
    meta = Path(bag_dir) / 'metadata.yaml'
    if not meta.is_file():
        raise BagError(f"'{bag_dir}' is not a rosbag2 folder (no metadata.yaml)")
    with open(meta, encoding='utf-8') as f:
        return yaml.safe_load(f)['rosbag2_bagfile_information']


def storage_id(bag_dir):
    """Storage plugin of the bag (sqlite3 or mcap), as recorded in its metadata."""
    return _info(bag_dir).get('storage_identifier', 'sqlite3')


def duration_s(bag_dir):
    """Recorded duration, from the metadata (the bag is not opened)."""
    return _info(bag_dir)['duration']['nanoseconds'] * 1e-9


def start_ns(bag_dir):
    """Start of the recording (epoch ns, the clock of the bag's timestamps), from the metadata: what turns an
    absolute timestamp (a /tag, say) into the offset from the start that `ros2 bag play --start-offset` wants."""
    return _info(bag_dir)['starting_time']['nanoseconds_since_epoch']


def topics(bag_dir):
    """{topic: type} of the bag, from the metadata (the bag is not opened)."""
    return {t['topic_metadata']['name']: t['topic_metadata']['type']
            for t in _info(bag_dir)['topics_with_message_count']}


def message_counts(bag_dir):
    """{topic: number of messages}, from the metadata."""
    return {t['topic_metadata']['name']: t['message_count'] for t in _info(bag_dir)['topics_with_message_count']}


def find_bags(root):
    """Sorted folders under `root` that are rosbag2 bags (have a metadata.yaml)."""
    return sorted(p.parent for p in Path(root).expanduser().glob('*/metadata.yaml'))


def replay_launch_args(bag_dir, test_config_file=None):
    """`bag_folder:=... bag_name:=...` for launching walker_replay on a definitive bag.

    walker_replay reads test_config.txt from the folder of the bags: asking for another file is refused rather than
    replaying with user data that are not the ones the rest of the script uses."""
    bag_dir = Path(bag_dir).expanduser().resolve()
    if test_config_file is not None:
        expected = bag_dir.parent / 'test_config.txt'
        if Path(test_config_file).expanduser().resolve() != expected:
            raise BagError(f"walker_replay reads {expected}; --test-config-file {test_config_file} cannot be used "
                           f"to replay. Put it in the folder of the bags")
    return f'bag_folder:={bag_dir.parent} bag_name:={bag_dir.name}'


def _open_reader(bag_dir, wanted=None):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id(bag_dir)),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if wanted is not None:
        reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in wanted if t in types]))
    return reader, types


def iter_raw(bag_dir, wanted=None):
    """(topic, t_ns, serialized bytes, type) for the messages of `wanted` topics (all if None), in bag order,
    without deserialising them."""
    reader, types = _open_reader(bag_dir, wanted)
    while reader.has_next():
        topic, data, t = reader.read_next()
        yield topic, t, data, types[topic]


def iter_messages(bag_dir, wanted=None):
    """(topic, t_ns, message) for the messages of `wanted` topics (all if None), in bag order. `t_ns` is the
    recording time of the bag, not the stamp in the message header."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    classes = {}
    for topic, t, data, type_name in iter_raw(bag_dir, wanted):
        if type_name not in classes:
            classes[type_name] = get_message(type_name)
        yield topic, t, deserialize_message(data, classes[type_name])


def load_messages(bag_dir, wanted, require_all=True):
    """{topic: [(t_ns, message)]} for the topics asked for, in bag order. With require_all (default) a topic
    the bag does not have raises BagError; otherwise it is left out of the result."""
    wanted = list(wanted)
    have = topics(bag_dir)
    missing = [t for t in wanted if t not in have]
    if missing and require_all:
        raise BagError(f"{bag_dir}: it has no {missing} (its topics: {sorted(have)})")
    present = [t for t in wanted if t in have]
    out = {t: [] for t in present}
    for topic, t, msg in iter_messages(bag_dir, present):
        out[topic].append((t, msg))
    return out


def read_tag_window(bag_dir):
    """(t_start_ns, t_end_ns): recording times of the first and last /tag message, which delimit the active part
    of a test (outside it the user is being placed or calibrated, not walking). None if the bag has fewer than
    two /tag messages."""
    if TAG_TOPIC not in topics(bag_dir):
        return None
    times = sorted(t for t, _ in load_messages(bag_dir, [TAG_TOPIC])[TAG_TOPIC])
    if len(times) < 2:
        return None
    return times[0], times[-1]


def tag_window_offset_and_duration(bag_dir):
    """(start_offset_s, duration_s) of the window between the two /tag messages: the start as seconds from the start
    of the bag (what `start_offset` of the replay takes) and its length in bag seconds. None if there is no window."""
    window = read_tag_window(bag_dir)
    if window is None:
        return None
    t0, t1 = window
    return (t0 - start_ns(bag_dir)) / 1e9, (t1 - t0) / 1e9


def clip_series(series, t_start_ns, t_end_ns):
    """The items of a time-sorted series whose first element is a time (ns) within [t_start_ns, t_end_ns]."""
    return [item for item in series if t_start_ns <= item[0] <= t_end_ns]


class BagWriter:
    """Writes a new rosbag2 (sqlite3), topic by topic or interleaved:

        with BagWriter(out_dir) as bag:
            bag.add_topic('/odom', 'nav_msgs/msg/Odometry')
            bag.write('/odom', t_ns, odometry_message)        # or write_raw(topic, t_ns, bytes)
    """

    def __init__(self, out_dir, storage='sqlite3'):
        import rosbag2_py
        self._rosbag2_py = rosbag2_py
        self._topic_ids = {}
        self._writer = rosbag2_py.SequentialWriter()
        self._writer.open(rosbag2_py.StorageOptions(uri=str(out_dir), storage_id=storage),
                          rosbag2_py.ConverterOptions('', ''))

    def add_topic(self, name, type_name):
        if name not in self._topic_ids:
            self._topic_ids[name] = len(self._topic_ids)
            self._writer.create_topic(self._rosbag2_py.TopicMetadata(
                id=self._topic_ids[name], name=name, type=type_name, serialization_format='cdr'))

    def write(self, topic, t_ns, message):
        from rclpy.serialization import serialize_message
        self._writer.write(topic, serialize_message(message), int(t_ns))

    def write_raw(self, topic, t_ns, data):
        self._writer.write(topic, data, int(t_ns))

    def close(self):
        self._writer = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
