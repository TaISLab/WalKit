import pytest
import yaml

from walker_replay.bag import BagError, resolve_bag, topic_selection
from walker_replay.test_config import (ConfigFileError, parse_line, read_bag_config,
                                       read_test_config)

CONFIG = """[test]:[handle_height]:[age]:[height]:[weight]:[user_id]:[gender]:[notes] 
MF_test01:4:44:175:97:MF:m:movimiento en circulos 
CA_test07:0:20:163:53:CA:f:postura inclinada: manetas muy bajas

LA_test01:4:22:165:60:LA:f:Maneta, edad
"""


def make_folder(tmp_path, definitive=True, name='MF_test01', config=CONFIG):
    (tmp_path / 'test_config.txt').write_text(config)
    bag = tmp_path / name
    bag.mkdir()
    (bag / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': {
        'storage_identifier': 'sqlite3', 'duration': {'nanoseconds': 5_000_000_000},
        'topics_with_message_count': [
            {'topic_metadata': {'name': t, 'type': 'x/msg/Y'}, 'message_count': 3}
            for t in ('/scan', '/rosout', '/left_handle')]}}))
    if definitive:
        (bag / 'definitive_log.md').write_text('# Bag definitivo')
    return str(tmp_path)


def test_parse_whole_file(tmp_path):
    rows = read_test_config(make_folder(tmp_path))
    assert list(rows) == ['MF_test01', 'CA_test07', 'LA_test01']
    ca = rows['CA_test07']
    assert (ca.handle_height, ca.age, ca.height, ca.weight, ca.user_id) == (0, 20, 163, 53, 'CA')
    assert ca.notes == 'postura inclinada: manetas muy bajas'      # ':' in the notes
    assert ca.gender_desc == 'Femenino' and rows['MF_test01'].gender_desc == 'Masculino'


def test_bad_lines():
    with pytest.raises(ConfigFileError):
        parse_line('MF_test01:4:44')
    with pytest.raises(ConfigFileError):
        parse_line('MF_test01:x:44:175:97:MF:m:n')
    with pytest.raises(ConfigFileError):
        parse_line('MF_test01:4:44:175:97:MF:x:n')


def test_duplicated_and_missing(tmp_path):
    folder = make_folder(tmp_path, config=CONFIG + 'MF_test01:4:44:175:97:MF:m:otra\n')
    with pytest.raises(ConfigFileError, match='duplicated'):
        read_test_config(folder)
    other = tmp_path / 'other'
    other.mkdir()
    with pytest.raises(ConfigFileError, match='no row'):
        read_bag_config(make_folder(other), 'XX_test99')


def test_resolve_bag(tmp_path):
    bag = resolve_bag(make_folder(tmp_path), 'MF_test01')
    assert bag.config.handle_height == 4 and bag.duration_s == 5.0
    assert set(bag.topics) == {'/scan', '/rosout', '/left_handle'}


def test_non_definitive_refused(tmp_path):
    with pytest.raises(BagError, match='not a definitive bag'):
        resolve_bag(make_folder(tmp_path, definitive=False), 'MF_test01')
    with pytest.raises(BagError, match='does not exist'):
        resolve_bag(str(tmp_path), 'nope')


def test_topic_selection(tmp_path):
    bag = resolve_bag(make_folder(tmp_path), 'MF_test01')
    topics, exclude, missing = topic_selection(bag, [], [])
    assert topics == [] and exclude == ['/rosout', '/parameter_events'] and missing == []
    topics, exclude, missing = topic_selection(bag, ['/scan', '/rosout', '/tf'], ['/scan_x'])
    assert exclude == ['/parameter_events', '/scan_x']      # /rosout asked for explicitly
    assert missing == ['/tf']
