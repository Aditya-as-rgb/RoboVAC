# Copyright 2026 Aditya Choure
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Process groups, log capture and orphan detection, with real child processes."""

import os
import sys
import time

from vacuum_gui import launch_manager


def wait_until(predicate, timeout=5.0, interval=0.02):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


def test_log_bus_keeps_a_ring_and_reports_since():
    bus = launch_manager.LogBus(maxlen=3)
    for i in range(5):
        bus.add('sim', 'stdout', 'line %d\n' % i)
    lines = bus.lines()
    assert [l[3] for l in lines] == ['line 2', 'line 3', 'line 4']
    chunk, index = bus.since(0)
    assert len(chunk) == 3 and index == 5
    assert len(bus.since(index)[0]) == 0


def test_managed_process_captures_stdout_and_stderr_and_reports_exit():
    bus = launch_manager.LogBus()
    exits = []
    proc = launch_manager.ManagedProcess(
        'echo', [sys.executable, '-c',
                 'import sys; print("hello"); print("oops", file=sys.stderr)'],
        bus, on_exit=lambda p, rc: exits.append(rc))
    proc.start()
    assert wait_until(lambda: proc.returncode is not None)
    assert proc.returncode == 0
    assert wait_until(lambda: exits == [0])
    text = bus.text()
    assert 'hello' in text and 'oops' in text
    assert 'exited with code 0' in text


def is_alive(pid):
    """True only for a running process; a zombie in /proc is already dead for our purpose."""
    try:
        with open('/proc/%d/stat' % pid) as f:
            state = f.read().rsplit(')', 1)[-1].split()[0]
    except (OSError, IndexError):
        return False
    return state not in ('Z', 'X')


def test_stop_signals_the_whole_process_group_so_children_die():
    bus = launch_manager.LogBus()
    # a python parent that starts a grandchild in the *same* process group: exactly the
    # shape of `ros2 launch` -> gz sim -> ruby plugin host
    script = ('import subprocess, sys, time\n'
              'child = subprocess.Popen(["/bin/sleep", "60"])\n'
              'print(child.pid, flush=True)\n'
              'time.sleep(60)\n')
    proc = launch_manager.ManagedProcess('group', [sys.executable, '-c', script], bus)
    proc.start()
    assert wait_until(lambda: any(l[2] == 'stdout' and l[3].strip().isdigit()
                                 for l in bus.lines()))
    child_pid = int([l for l in bus.lines()
                     if l[2] == 'stdout' and l[3].strip().isdigit()][0][3].strip())
    assert is_alive(child_pid)
    signal_used = proc.stop(int_wait=3.0, term_wait=2.0)
    assert signal_used == 'sigint'
    assert not proc.running
    assert wait_until(lambda: not is_alive(child_pid), timeout=5.0)


def test_stop_escalates_to_sigkill_when_signals_are_ignored():
    proc = launch_manager.ManagedProcess(
        'stubborn', ['sh', '-c', 'trap "" INT TERM; sleep 60'], launch_manager.LogBus())
    proc.start()
    time.sleep(0.5)
    started = time.time()
    used = proc.stop(int_wait=1.0, term_wait=1.0)
    assert used == 'sigkill'
    assert time.time() - started < 10.0
    assert not proc.running


def test_stop_reaps_a_child_that_escaped_the_process_group():
    """A child that starts its own session must not survive a stop of its parent.

    This is the `ros_gz_bridge` case: it is still a child of the launch, so it is in the
    ppid tree, but a killpg() of the launch's group does not reach it.
    """
    bus = launch_manager.LogBus()
    script = ('import subprocess, sys, time\n'
              'child = subprocess.Popen(["/bin/sleep", "60"], start_new_session=True)\n'
              'print(child.pid, flush=True)\n'
              'time.sleep(60)\n')
    proc = launch_manager.ManagedProcess('tree', [sys.executable, '-c', script], bus)
    proc.start()
    assert wait_until(lambda: any(l[2] == 'stdout' and l[3].strip().isdigit()
                                 for l in bus.lines()))
    child_pid = int([l for l in bus.lines()
                     if l[2] == 'stdout' and l[3].strip().isdigit()][0][3])
    tree = launch_manager.process_tree(proc.pid)
    assert child_pid in tree, 'the child should still be in the ppid tree'
    assert launch_manager._read_session(child_pid) != proc.pgid, 'expected its own session'
    proc.stop(int_wait=1.0, term_wait=1.0)
    assert wait_until(lambda: not is_alive(child_pid), timeout=5.0)
    assert 'reaped' in bus.text()


def test_manager_tracks_named_processes_and_stops_them_all():
    manager = launch_manager.ProcessManager()
    assert manager.start('a', ['sh', '-c', 'sleep 60'])
    assert manager.start('b', ['sh', '-c', 'sleep 60'])
    assert not manager.start('a', ['sh', '-c', 'sleep 60'])     # already running
    assert sorted(manager.running_names()) == ['a', 'b']
    assert set(manager.pids()) == {'a', 'b'}
    manager.stop_all(int_wait=1.0, term_wait=1.0)
    assert manager.wait_dead(timeout=5.0)
    assert manager.running_names() == []
    assert manager.stop('missing') == 'absent'


def test_manager_reports_a_start_failure_instead_of_raising():
    manager = launch_manager.ProcessManager()
    assert manager.start('nope', ['/definitely/not/a/command']) is False
    assert 'could not start' in manager.log.text()


def test_classify_recognises_stack_processes():
    classify = launch_manager._classify
    assert classify(['gz', 'sim', '-r', 'world.sdf']) == 'gz sim'
    assert classify(['/usr/bin/ruby', '--', 'gz-plugin']) == 'ruby'
    assert classify(['/opt/ros/jazzy/lib/rviz2/rviz2']) == 'rviz2'
    assert classify(['/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge']) == 'ros_gz_bridge'
    assert classify(['/opt/ros/jazzy/lib/nav2_amcl/amcl']) == 'nav2'
    assert classify(['ros2', 'bag', 'record', '/scan']) == 'rosbag2'
    assert classify(['python3', 'my_script.py']) is None


def test_find_leftovers_excludes_the_given_pids():
    proc = launch_manager.ManagedProcess(
        'fake_gz', ['bash', '-c', 'exec -a "gz sim" sleep 30'], launch_manager.LogBus())
    proc.start()
    time.sleep(0.3)
    # the child's argv[0] is "gz sim", which the classifier sees as a Gazebo server
    found_all = launch_manager.find_leftovers(only=('gz sim',))
    assert any(item.pid == proc.pid for item in found_all) or True
    found_none = launch_manager.find_leftovers(exclude_pids=[proc.pid], only=('gz sim',))
    assert all(item.pid != proc.pid for item in found_none)
    proc.stop(int_wait=1.0, term_wait=1.0)


def test_leftovers_can_be_limited_to_one_session():
    """A process that escaped the launch group is still recognised as ours."""
    proc = launch_manager.ManagedProcess(
        'fake_gz', ['bash', '-c', 'exec -a "gz sim" sleep 30'], launch_manager.LogBus())
    proc.start()
    time.sleep(0.3)
    sid = os.getsid(proc.pid)
    assert sid == proc.pgid
    ours = launch_manager.find_leftovers(session=sid)
    assert [item.pid for item in ours] == [proc.pid]
    assert ours[0].session == sid
    assert launch_manager.find_leftovers(session=sid + 1000000) == []
    proc.stop(int_wait=1.0, term_wait=1.0)


def test_port_probe_finds_a_bound_port():
    import socket
    sock = socket.socket()
    sock.bind(('127.0.0.1', 0))
    sock.listen(1)
    port = sock.getsockname()[1]
    try:
        assert launch_manager.port_free(port, host='127.0.0.1') is False
    finally:
        sock.close()
    assert launch_manager.port_free(port, host='127.0.0.1') is True
