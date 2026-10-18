import pytest
import yaml

from walker_replay import bagio
from walker_replay.bag import BagError


def fake_bag(tmp_path, storage='mcap'):
    bag = tmp_path / 'b1'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': {
        'storage_identifier': storage,
        'duration': {'nanoseconds': 2_500_000_000},
        'starting_time': {'nanoseconds_since_epoch': 1_700_000_000_000_000_000},
        'topics_with_message_count': [
            {'topic_metadata': {'name': '/scan', 'type': 'sensor_msgs/msg/LaserScan'}, 'message_count': 7},
            {'topic_metadata': {'name': '/tag', 'type': 'std_msgs/msg/String'}, 'message_count': 2}]}}))
    return bag


def test_metadata_without_opening_the_bag(tmp_path):
    bag = fake_bag(tmp_path)
    assert bagio.storage_id(bag) == 'mcap'
    assert bagio.duration_s(bag) == pytest.approx(2.5)
    assert bagio.start_ns(bag) == 1_700_000_000_000_000_000
    assert bagio.topics(bag) == {'/scan': 'sensor_msgs/msg/LaserScan', '/tag': 'std_msgs/msg/String'}
    assert bagio.message_counts(bag) == {'/scan': 7, '/tag': 2}
    assert bagio.find_bags(tmp_path) == [bag]


def test_not_a_bag(tmp_path):
    with pytest.raises(BagError):
        bagio.topics(tmp_path)


def test_clip_series():
    series = [(1, 'a'), (5, 'b'), (9, 'c'), (12, 'd')]
    assert bagio.clip_series(series, 5, 9) == [(5, 'b'), (9, 'c')]


def test_write_then_read(tmp_path):
    pytest.importorskip('rosbag2_py')
    from std_msgs.msg import String
    out = tmp_path / 'out'
    with bagio.BagWriter(out) as bag:
        bag.add_topic('/a', 'std_msgs/msg/String')
        bag.add_topic('/tag', 'std_msgs/msg/String')
        for t, topic, text in ((10, '/a', 'x'), (20, '/tag', 'start'), (30, '/a', 'y'), (40, '/tag', 'end'),
                               (50, '/a', 'z')):
            bag.write(topic, t, String(data=text))
    assert bagio.topics(out) == {'/a': 'std_msgs/msg/String', '/tag': 'std_msgs/msg/String'}
    got = bagio.load_messages(out, ['/a'])
    assert [(t, m.data) for t, m in got['/a']] == [(10, 'x'), (30, 'y'), (50, 'z')]
    assert bagio.read_tag_window(out) == (20, 40)
    offset, length = bagio.tag_window_offset_and_duration(out)
    assert length == pytest.approx(20e-9)
    assert [(topic, t) for topic, t, _ in bagio.iter_messages(out)] == [
        ('/a', 10), ('/tag', 20), ('/a', 30), ('/tag', 40), ('/a', 50)]
    assert len(list(bagio.iter_raw(out, ['/tag']))) == 2


def test_missing_topics(tmp_path):
    pytest.importorskip('rosbag2_py')
    from std_msgs.msg import String
    out = tmp_path / 'out'
    with bagio.BagWriter(out) as bag:
        bag.add_topic('/a', 'std_msgs/msg/String')
        bag.write('/a', 1, String(data='x'))
    with pytest.raises(BagError):
        bagio.load_messages(out, ['/a', '/nope'])
    assert list(bagio.load_messages(out, ['/a', '/nope'], require_all=False)) == ['/a']
    assert bagio.read_tag_window(out) is None


def test_replay_launch_args(tmp_path):
    bag = fake_bag(tmp_path)
    assert bagio.replay_launch_args(bag) == f'bag_folder:={tmp_path} bag_name:=b1'
    assert bagio.replay_launch_args(bag, tmp_path / 'test_config.txt') == f'bag_folder:={tmp_path} bag_name:=b1'
    with pytest.raises(BagError):
        bagio.replay_launch_args(bag, tmp_path / 'other.txt')
