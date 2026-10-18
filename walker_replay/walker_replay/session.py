"""Starting and stopping the processes of a replay session (a launch, a recorder, a bag player) from a script.

The tools that replay many bags in a row (comparisons against the mocap) each need to start `ros2 launch ...`
with a sourced environment and an isolated ROS_DOMAIN_ID, and to stop it, with all the nodes it started, when
the bag is over. They had six copies of this; this is the one.

Two details that cost time before: the process goes in its own process group, so that one signal reaches
the launch and every node under it; and its SIGINT disposition is reset to the default, because a script run in
the background of a shell (`cmd &`, nohup) inherits SIGINT as ignored and `ros2 bag record` and friends would
then never finish writing their bag on the signal.
"""
import os
import signal
import subprocess
import time
from pathlib import Path

ROS_SETUP = '/opt/ros/jazzy/setup.bash'
WORKSPACE_SETUP = str(Path.home() / 'workspace' / 'walker_ws' / 'install' / 'setup.bash')


def start(cmd, domain_id=None, log=None, setups=(ROS_SETUP, WORKSPACE_SETUP)):
    """Runs `cmd` under bash with `setups` sourced, in its own process group; stdout/stderr to the file `log`
    (or discarded). `domain_id` isolates it from other ROS 2 processes on the machine: a foreign process in the
    same domain contaminated measurements before without any visible error."""
    prefix = ''.join(f'source {s}; ' for s in setups)
    if domain_id is not None:
        prefix += f'export ROS_DOMAIN_ID={domain_id}; '
    out = open(log, 'w') if log else subprocess.DEVNULL
    return subprocess.Popen(['bash', '-c', prefix + 'exec ' + cmd], stdout=out, stderr=subprocess.STDOUT if log else subprocess.DEVNULL,
                            start_new_session=True,
                            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))


def stop(proc, timeout=8):
    """Stops the process and everything it started: SIGINT (what ros2 launch and ros2 bag record finish
    cleanly on), then SIGTERM, then SIGKILL, waiting `timeout` seconds after each."""
    if proc is None or proc.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass


def wait_alive(proc, seconds):
    """Sleeps `seconds` and says whether the process is still running (a launch that died on startup)."""
    time.sleep(seconds)
    return proc.poll() is None
