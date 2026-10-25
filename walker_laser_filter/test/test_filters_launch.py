import os
import re
import shutil
import signal
import subprocess
import time

import pytest

CONFIG_DIR = os.path.join(os.path.dirname(__file__), '..', 'config')


def running_chains(args, seconds=6.0):
    """{node name: (parameter files, remappings)} of the filter nodes that `ros2 launch walker_laser_filter <args>` starts."""
    launch = subprocess.Popen(['ros2', 'launch', 'walker_laser_filter', *args], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                              start_new_session=True)
    try:
        time.sleep(seconds)
        ps = subprocess.run(['ps', '-eo', 'args'], capture_output=True, text=True).stdout
    finally:
        os.killpg(launch.pid, signal.SIGINT)
        launch.wait(timeout=20)
    out = {}
    for line in ps.splitlines():
        if 'scan_to_scan_filter_chain' in line and '__node:=' in line and '/walker_laser_filter' in line:
            name = re.search(r'__node:=(\w+)', line).group(1)
            out[name] = (sorted(os.path.basename(f) for f in re.findall(r'--params-file (\S+\.yaml)', line)),
                         dict(re.findall(r'-r (scan\w*):=(\S+)', line)))
    return out


@pytest.mark.skipif(shutil.which('ros2') is None, reason='needs a sourced ROS 2 environment with this package installed')
def test_every_stage_of_filters_launch_gets_its_own_file_and_topics():
    chains = running_chains(['filters.launch.py'])
    assert chains == {
        'self_laser_filter': (['self_filter.yaml'], {'scan': '/scan', 'scan_filtered': '/scan_filtered'}),
        'nav_laser_filter': (['nav_filter.yaml'], {'scan': '/scan_filtered', 'scan_filtered': '/scan_nav'}),
        'feet_laser_filter': (['feet_filter.yaml'], {'scan': '/scan_filtered', 'scan_filtered': '/scan_feet'})}


@pytest.mark.skipif(shutil.which('ros2') is None, reason='needs a sourced ROS 2 environment with this package installed')
def test_the_raw_scan_topic_only_changes_the_first_stage():
    chains = running_chains(['filters.launch.py', 'scan_topic:=/raw'])
    assert chains['self_laser_filter'][1]['scan'] == '/raw' and chains['nav_laser_filter'][1]['scan'] == '/scan_filtered'
