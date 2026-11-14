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

"""Managed subprocesses in their own process groups, plus orphan hunting.

``ros2 launch`` starts a tree of processes; a plain ``Popen.terminate()`` kills only the
launcher and leaves ``gz sim``, its ruby plugin host and ``rviz2`` behind. Every child
here gets ``start_new_session=True`` (``setsid``), so stopping one means signalling the
whole group: SIGINT first (ROS shutdown is clean), then SIGTERM, then SIGKILL.

No ROS and no Qt in this module, which is why ``test/test_launch_manager.py`` can run a
real process group and check that a child that ignores SIGINT still dies.
"""

import errno
import os
import signal
import socket
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass

from .commands import child_env, shell_join

MAX_LOG_LINES = 20000


class LogBus:
    """Thread-safe log buffer: one entry per (process, stream, line)."""

    def __init__(self, maxlen=MAX_LOG_LINES):
        self._lines = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._listeners = []
        self._dropped = 0

    def add(self, proc, stream, text):
        entry = (time.time(), proc, stream, text.rstrip('\n'))
        with self._lock:
            if len(self._lines) == self._lines.maxlen:
                self._dropped += 1
            self._lines.append(entry)
            listeners = list(self._listeners)
        for cb in listeners:
            try:
                cb(entry)
            except Exception:      # noqa: BLE001 - a bad listener must not kill the reader
                pass

    def listen(self, callback):
        with self._lock:
            self._listeners.append(callback)

    def unlisten(self, callback):
        with self._lock:
            if callback in self._listeners:
                self._listeners.remove(callback)

    def lines(self):
        with self._lock:
            return list(self._lines)

    def since(self, index):
        """Entries from ``index`` on; returns (entries, next_index)."""
        with self._lock:
            start = max(0, index - self._dropped)
            chunk = list(self._lines)[start:]
            return chunk, self._dropped + len(self._lines)

    def clear(self):
        with self._lock:
            self._lines.clear()

    def text(self):
        return '\n'.join('%s [%s] %s' % (l[1], l[2], l[3]) for l in self.lines())


@dataclass
class Leftover:
    pid: int
    name: str
    cmdline: str
    session: int = 0


class ManagedProcess:
    """One command in its own session, with line-buffered log capture."""

    def __init__(self, name, argv, log, env=None, cwd=None, on_exit=None):
        self.name = name
        self.argv = list(argv)
        self.log = log
        self.env = env or child_env()
        self.cwd = cwd
        self.on_exit = on_exit
        self.proc = None
        self.pgid = None
        self.returncode = None
        self._threads = []
        self._stop_requested = False
        self.started_at = 0.0
        self.stop_signal = ''

    # ------------------------------------------------------------- lifecycle --
    def start(self):
        self.log.add(self.name, 'cmd', shell_join(self.argv))
        self.proc = subprocess.Popen(
            self.argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            stdin=subprocess.DEVNULL,
            text=True,
            bufsize=1,
            errors='replace',
            env=self.env,
            cwd=self.cwd,
            start_new_session=True,
        )
        self.pgid = os.getpgid(self.proc.pid)
        self.started_at = time.time()
        for stream in ('stdout', 'stderr'):
            pipe = getattr(self.proc, stream)
            t = threading.Thread(target=self._pump, args=(pipe, stream), daemon=True)
            t.start()
            self._threads.append(t)
        t = threading.Thread(target=self._wait, daemon=True)
        t.start()
        self._threads.append(t)
        return self

    def _pump(self, pipe, stream):
        try:
            for line in iter(pipe.readline, ''):
                self.log.add(self.name, stream, line)
        except (ValueError, OSError):
            pass
        finally:
            try:
                pipe.close()
            except OSError:
                pass

    def _wait(self):
        rc = self.proc.wait() if self.proc else None
        self.returncode = rc
        if self._stop_requested:
            self.log.add(self.name, 'exit', 'stopped (%s)' % (self.stop_signal or 'signal'))
        else:
            self.log.add(self.name, 'exit', 'exited with code %s' % rc)
        if self.on_exit:
            try:
                self.on_exit(self, rc)
            except Exception:      # noqa: BLE001
                pass

    @property
    def running(self):
        return self.proc is not None and self.proc.poll() is None

    @property
    def pid(self):
        return self.proc.pid if self.proc else None

    def uptime(self):
        return 0.0 if not self.started_at else max(0.0, time.time() - self.started_at)

    # ------------------------------------------------------------------ stop --
    def signal(self, sig):
        if not self.running or not self.pgid:
            return False
        try:
            os.killpg(self.pgid, sig)
            return True
        except ProcessLookupError:
            return False
        except OSError as exc:
            if exc.errno == errno.ESRCH:
                return False
            raise

    def stop(self, int_wait=8.0, term_wait=4.0):
        """SIGINT the group, then SIGTERM, then SIGKILL. Returns the signal used."""
        proc = self.proc
        if proc is None or not self.running:
            return 'gone'
        # snapshot the descendants first: a child that called setsid() is not in the
        # process group we are about to signal, and would otherwise survive the stop
        tree = process_tree(proc.pid)
        self._stop_requested = True
        for sig, wait, label in ((signal.SIGINT, int_wait, 'sigint'),
                                 (signal.SIGTERM, term_wait, 'sigterm'),
                                 (signal.SIGKILL, 2.0, 'sigkill')):
            if self.signal(sig):
                self.stop_signal = label
            try:
                proc.wait(timeout=wait)
                break
            except subprocess.TimeoutExpired:
                continue
        alive = [pid for pid in tree if os.path.exists('/proc/%d' % pid)]
        if alive:
            _kill_pids(alive)
            self.log.add(self.name, 'stop', 'reaped %d escaped child(ren): %s'
                         % (len(alive), ', '.join(str(p) for p in alive)))
        return label


class ProcessManager:
    """Named processes; one per part of the stack."""

    def __init__(self, log=None, base_env=None):
        self.log = log or LogBus()
        self.base_env = base_env
        self.processes = {}
        self._lock = threading.Lock()

    def env(self, extra=None):
        env = dict(self.base_env) if self.base_env else child_env()
        if extra:
            env.update({str(k): str(v) for k, v in extra.items()})
        return env

    def start(self, name, argv, env=None, cwd=None, on_exit=None):
        with self._lock:
            old = self.processes.get(name)
            if old is not None and old.running:
                return False
            proc = ManagedProcess(name, argv, self.log, env=env or self.env(), cwd=cwd,
                                  on_exit=on_exit)
            try:
                proc.start()
            except (OSError, ValueError) as exc:
                self.log.add(name, 'error', 'could not start: %s' % exc)
                return False
            self.processes[name] = proc
            return True

    def get(self, name):
        return self.processes.get(name)

    def running(self, name):
        proc = self.processes.get(name)
        return bool(proc and proc.running)

    def running_names(self):
        return [n for n, p in self.processes.items() if p.running]

    def pids(self):
        return {n: p.pid for n, p in self.processes.items() if p.running}

    def stop(self, name, **kwargs):
        proc = self.processes.get(name)
        if proc is None:
            return 'absent'
        result = proc.stop(**kwargs)
        self.log.add(name, 'stop', 'stop -> %s' % result)
        return result

    def stop_all(self, names=None, **kwargs):
        results = {}
        names = names if names is not None else list(self.processes)
        # stop the mission/coverage first so nothing keeps writing goals while Nav2 dies
        for name in reversed([n for n in names if self.running(n)]):
            results[name] = self.stop(name, **kwargs)
        return results

    def wait_dead(self, timeout=12.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.running_names():
                return True
            for proc in list(self.processes.values()):
                if proc.running:
                    try:
                        proc.proc.wait(timeout=0.2)
                    except subprocess.TimeoutExpired:
                        pass
        return not self.running_names()


# ------------------------------------------------------------------ leftovers --

LEFTOVER_KINDS = ('gz sim', 'ruby', 'rviz2', 'ros_gz_bridge', 'nav2', 'rosbag2')

# The kinds that actually make the next start fail: a second gz on one world, a second
# /clock from a stray bridge. rviz2, Nav2 servers and rosbag2 may belong to another
# project on the same machine, so they are reported but never block a start and are
# never killed unless they carry the session id of a launch this panel started.
BLOCKING_KINDS = ('gz sim', 'ruby', 'ros_gz_bridge')


def _read_cmdline(pid):
    try:
        with open('/proc/%d/cmdline' % pid, 'rb') as f:
            raw = f.read()
    except OSError:
        return None
    return [p.decode('utf-8', 'replace') for p in raw.split(b'\0') if p]


def _stat_fields(pid):
    """(ppid, pgrp, session) of a process, or None when /proc is unreadable."""
    try:
        with open('/proc/%d/stat' % pid) as f:
            fields = f.read().rsplit(')', 1)[-1].split()
        return int(fields[1]), int(fields[2]), int(fields[3])
    except (OSError, IndexError, ValueError):
        return None


def _read_session(pid):
    """Session id of a process (zeros when it cannot be read)."""
    fields = _stat_fields(pid)
    return fields[2] if fields else 0


def process_tree(root_pid):
    """Every live descendant of ``root_pid``.

    A child that calls setsid() escapes the process group the panel signals, and
    `ros2 launch` itself is started in its own session; walking the ppid chain catches
    those. Returns pids in breadth-first order, root excluded.
    """
    if not root_pid:
        return []
    children = {}
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        fields = _stat_fields(pid)
        if fields is None:
            continue
        children.setdefault(fields[0], []).append(pid)
    found, stack = [], [int(root_pid)]
    while stack:
        parent = stack.pop(0)
        for child in children.get(parent, []):
            if child not in found:
                found.append(child)
                stack.append(child)
    return found


def _kill_pids(pids, wait=2.0):
    """SIGTERM then SIGKILL a list of pids; returns what was done per pid."""
    results = {}
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
            results[pid] = 'sigterm'
        except ProcessLookupError:
            results[pid] = 'gone'
    deadline = time.time() + wait
    while time.time() < deadline:
        if all(not os.path.exists('/proc/%d' % pid) or results.get(pid) == 'gone'
               for pid in results):
            break
        time.sleep(0.1)
    for pid, how in list(results.items()):
        if how == 'gone':
            continue
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            continue
        try:
            os.kill(pid, signal.SIGKILL)
            results[pid] = 'sigkill'
        except ProcessLookupError:
            pass
    return results


def _classify(argv):
    joined = ' '.join(argv)
    base = os.path.basename(argv[0]) if argv else ''
    if base in ('gz', 'gz-sim', 'gz sim'):
        if 'sim' in argv or base != 'gz':
            return 'gz sim'
    if base == 'ruby' and ('gz' in joined or 'sim' in joined):
        return 'ruby'
    if base.startswith('rviz2') or base == 'rviz2':
        return 'rviz2'
    if 'parameter_bridge' in joined:
        return 'ros_gz_bridge'
    if 'nav2_container' in joined or base in ('controller_server', 'planner_server',
                                              'bt_navigator', 'amcl', 'map_server'):
        return 'nav2'
    if base == 'ros2' and argv[1:2] == ['bag'] and 'record' in argv:
        return 'rosbag2'
    return None


def find_leftovers(exclude_pids=(), only=None, session=None):
    """Processes that look like a running stack, excluding ``exclude_pids``.

    ``session`` restricts the result to one session (process group leader), which is how
    the panel recognises processes that escaped the launch's own group: they are ours, so
    they can be killed without asking.
    """
    exclude = set(int(p) for p in exclude_pids if p)
    exclude.add(os.getpid())
    found = []
    try:
        entries = os.listdir('/proc')
    except OSError:
        return found
    for entry in entries:
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in exclude:
            continue
        argv = _read_cmdline(pid)
        if not argv:
            continue
        kind = _classify(argv)
        if kind is None or (only and kind not in only):
            continue
        sid = _read_session(pid)
        if session is not None and sid != int(session):
            continue
        found.append(Leftover(pid, kind, ' '.join(argv)[:300], sid))
    return sorted(found, key=lambda x: x.pid)


def kill_leftovers(leftovers, wait=3.0):
    """SIGTERM then SIGKILL a list of leftovers. Returns {pid: 'sigterm'|'sigkill'|'gone'}."""
    results = {}
    for item in leftovers:
        try:
            os.kill(item.pid, signal.SIGTERM)
            results[item.pid] = 'sigterm'
        except ProcessLookupError:
            results[item.pid] = 'gone'
    deadline = time.time() + wait
    while time.time() < deadline:
        if all(not os.path.exists('/proc/%d' % pid) for pid in results
               if results[pid] != 'gone'):
            break
        time.sleep(0.1)
    for pid, how in list(results.items()):
        if how == 'gone':
            continue
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            continue
        try:
            os.kill(pid, signal.SIGKILL)
            results[pid] = 'sigkill'
        except ProcessLookupError:
            pass
    return results


def port_free(port, host='0.0.0.0'):
    """True when nothing is listening on TCP ``port``."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, int(port)))
        return True
    except OSError:
        return False
    finally:
        sock.close()
