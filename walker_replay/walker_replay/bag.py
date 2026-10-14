"""Checks on a definitive bag folder, before anything is replayed.

Only definitive bags (soma13kp/merge_definitive.py, see GENERAR_BAGS_DEFINITIVOS.md)
are replayed: they are the ones whose recorded data is the same data for everybody
(synchronised, mocap in the current message format, cleaned of derived topics and of
the tf that did not belong in a recording). Anything else is refused instead of
silently replayed with a different meaning.
"""
from dataclasses import dataclass
from os.path import expanduser, isdir, isfile, join

import yaml

from walker_replay.test_config import BagConfig, read_bag_config

DEFINITIVE_MARK = 'definitive_log.md'

# Recorded topics that must not be replayed unless explicitly asked for:
# /rosout would flood the log of whoever is replaying with the recording's
# own messages, /parameter_events belongs to the nodes of the recording day.
DEFAULT_EXCLUDED_TOPICS = ('/rosout', '/parameter_events')


class BagError(RuntimeError):
    pass


@dataclass(frozen=True)
class DefinitiveBag:
    path: str
    name: str
    config: BagConfig
    topics: dict          # name -> (type, message_count)
    duration_s: float
    storage: str


def resolve_bag(bag_folder, bag_name, test_config_file=None):
    """Validates `bag_folder/bag_name` and returns what is known about it.

    Raises BagError / ConfigFileError with a message saying what to do.
    """
    bag_folder = expanduser(str(bag_folder))
    path = join(bag_folder, bag_name)
    if not isdir(path):
        raise BagError(f"bag folder '{path}' does not exist")
    meta_file = join(path, 'metadata.yaml')
    if not isfile(meta_file):
        raise BagError(f"'{path}' is not a rosbag2 folder (no metadata.yaml)")
    if not isfile(join(path, DEFINITIVE_MARK)):
        raise BagError(
            f"'{path}' is not a definitive bag (no {DEFINITIVE_MARK}). Only definitive "
            f"bags are replayed: generate it with soma13kp/merge_definitive.py "
            f"(see GENERAR_BAGS_DEFINITIVOS.md)")

    with open(meta_file, encoding='utf-8') as f:
        info = yaml.safe_load(f)['rosbag2_bagfile_information']
    topics = {t['topic_metadata']['name']: (t['topic_metadata']['type'], t['message_count'])
              for t in info['topics_with_message_count']}
    config = read_bag_config(test_config_file or bag_folder, bag_name)
    return DefinitiveBag(path=path, name=bag_name, config=config, topics=topics,
                         duration_s=info['duration']['nanoseconds'] * 1e-9,
                         storage=info['storage_identifier'])


def topic_selection(bag, topics, exclude_topics):
    """(topics to --topics, topics to --exclude-topics, topics asked for that are not in the bag).

    Default exclusions are dropped for the topics that were asked for explicitly.
    """
    topics = list(topics)
    exclude = [t for t in DEFAULT_EXCLUDED_TOPICS if t not in topics]
    exclude += [t for t in exclude_topics if t not in exclude]
    missing = [t for t in topics if t not in bag.topics]
    return topics, exclude, missing
