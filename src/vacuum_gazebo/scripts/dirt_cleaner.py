#!/usr/bin/env python3
"""Dirt cleaner: the robot's brush removes the dirt patches it drives over.

Reads the ground-truth layout written by dirt_world.py (current_layout.json), follows the
robot pose (TF map -> base_footprint) and, as the robot moves, counts how many metres its
brush has travelled over each patch (a disc of ``clean_radius`` around the robot). A patch
comes off when that distance reaches its ``scrub_m``:

    crumbs / dust / hair   0.05 m   (one quick pass clears them)
    spills and stubborn    0.50 m   (a single pass over them is not enough)

A patch that is cleaned is removed from Gazebo (service /world/<world>/remove), so you
see it disappear. Patches the main run only brushed past, or never reached, stay: those
are what a vision check after the run has to find and the re-clean has to go back to.

Files (next to the layout, or in ``output_dir``):
    dirt_events_<time>.csv     one row per patch removed: time, name, kind, x, y, remaining
    dirt_summary_<time>.json   totals, per kind, what is left (rewritten after every removal)
Topics:
    /dirt/remaining  std_msgs/Int32        patches still on the floor
    /dirt/total      std_msgs/Int32        patches at the start (latched)
    /dirt/state      std_msgs/String       JSON {"remaining": [patch names]} (latched; the
                     dataset recorder uses it so cleaned patches are not labelled)
    /dirt/markers    visualization_msgs/MarkerArray  ground-truth dirt that is left (for RViz;
                     it is the answer key, not what a camera would see)

    ros2 run vacuum_gazebo dirt_cleaner.py --ros-args -p use_sim_time:=true

The brush radius is a separate setting from the coverage meter's clean_radius (0.22 m): it
defaults to 0.20 m so a patch the plan only just reaches can be missed, like a real brush.

If patches do not disappear in Gazebo, test the removal by hand (Gazebo running with a
dirt world):
    gz service -s /world/default/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \\
        --timeout 2000 --req 'name: "dirt_000" type: MODEL'
"""

import csv
from dataclasses import dataclass
import datetime
import json
import math
import os
import queue
import shutil
import subprocess
import threading
import time

DEFAULT_SCRUB_M = 0.05


# --------------------------------------------------------------------------
# ROS-free logic
# --------------------------------------------------------------------------

def chord_length(p0, p1, centre, radius):
    """Length of the segment p0 -> p1 that lies inside the disc (centre, radius)."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    a = dx * dx + dy * dy
    if a < 1e-12:
        return 0.0
    fx, fy = p0[0] - centre[0], p0[1] - centre[1]
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    disc = b * b - 4.0 * a * c
    if disc <= 0.0:
        return 0.0
    s = math.sqrt(disc)
    lo = max(0.0, (-b - s) / (2.0 * a))
    hi = min(1.0, (-b + s) / (2.0 * a))
    return max(0.0, hi - lo) * math.sqrt(a)


@dataclass
class Patch:
    name: str
    kind: str
    x: float
    y: float
    radius_m: float = 0.08
    scrub_m: float = DEFAULT_SCRUB_M
    stubborn: bool = False
    zone: str = ''
    scrubbed: float = 0.0          # metres of brush travel over it so far
    removed_at: float = None       # sim seconds when it came off, None while it is on the floor


def load_layout(path):
    """Read a dirt_world.py layout; returns (patches, meta)."""
    with open(os.path.expanduser(path)) as f:
        data = json.load(f)
    patches = []
    for p in data['patches']:
        patches.append(Patch(name=p['name'], kind=p.get('kind', 'dirt'), x=float(p['x']), y=float(p['y']),
                             radius_m=float(p.get('radius_m', 0.08)),
                             scrub_m=float(p.get('scrub_m', DEFAULT_SCRUB_M)),
                             stubborn=bool(p.get('stubborn', False)), zone=p.get('zone', '')))
    meta = {k: v for k, v in data.items() if k != 'patches'}
    return patches, meta


class DirtField:
    """Keeps track of what the brush has cleaned. Feed it the robot position with update()."""

    def __init__(self, patches, clean_radius=0.20, min_step=0.02, max_step=1.0):
        self.patches = list(patches)
        self.clean_radius = clean_radius
        self.min_step = min_step      # ignore moves smaller than this (pose jitter is not scrubbing)
        self.max_step = max_step      # a bigger jump is a localisation correction, not driving
        self._anchor = None

    @property
    def total(self):
        return len(self.patches)

    @property
    def remaining(self):
        return sum(1 for p in self.patches if p.removed_at is None)

    def remaining_names(self):
        return [p.name for p in self.patches if p.removed_at is None]

    def update(self, x, y, t):
        """New robot position at sim time t; returns the patches that came off during this move."""
        pos = (float(x), float(y))
        if self._anchor is None:
            self._anchor = pos
            return []
        step = math.hypot(pos[0] - self._anchor[0], pos[1] - self._anchor[1])
        if step < self.min_step:
            return []
        prev, self._anchor = self._anchor, pos
        if step > self.max_step:
            return []
        done = []
        for p in self.patches:
            if p.removed_at is not None:
                continue
            p.scrubbed += chord_length(prev, pos, (p.x, p.y), self.clean_radius)
            if p.scrubbed + 1e-9 >= p.scrub_m:
                p.removed_at = float(t)
                done.append(p)
        return done

    def summary(self):
        by_kind = {}
        for p in self.patches:
            k = by_kind.setdefault(p.kind, {'total': 0, 'removed': 0})
            k['total'] += 1
            k['removed'] += p.removed_at is not None
        stub = [p for p in self.patches if p.stubborn]
        left = [p for p in self.patches if p.removed_at is None]
        return {
            'total': self.total,
            'removed': self.total - len(left),
            'remaining': len(left),
            'removed_pct': round(100.0 * (self.total - len(left)) / max(1, self.total), 1),
            'clean_radius_m': self.clean_radius,
            'by_kind': by_kind,
            'stubborn': {'total': len(stub), 'removed': sum(p.removed_at is not None for p in stub)},
            'remaining_patches': [{'name': p.name, 'kind': p.kind, 'x': p.x, 'y': p.y,
                                   'stubborn': p.stubborn, 'scrubbed_m': round(p.scrubbed, 3),
                                   'scrub_m': p.scrub_m} for p in left],
            'timeline': [{'t_s': round(p.removed_at, 1), 'name': p.name, 'kind': p.kind}
                         for p in sorted((q for q in self.patches if q.removed_at is not None),
                                         key=lambda q: q.removed_at)],
        }


def gz_remove_cmd(world, name):
    return ['gz', 'service', '-s', '/world/%s/remove' % world,
            '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', 'name: "%s" type: MODEL' % name]


class Remover:
    """Takes finished patches out of Gazebo on a worker thread (the gz CLI call takes a moment)."""

    def __init__(self, world='default', runner=subprocess.run, sleep=time.sleep, log=None,
                 retries=5, backoff=2.0, on_done=None):
        self.world = world
        self._run = runner
        self._sleep = sleep
        self._log = log or (lambda msg: None)
        self._on_done = on_done or (lambda: None)       # called after every removal attempt
        self.retries = retries
        self.backoff = backoff
        self.removed = []
        self.failed = []
        self._q = queue.Queue()
        self._thread = None

    def process(self, name):
        """Remove one model, retrying while Gazebo's services are still coming up."""
        cmd = gz_remove_cmd(self.world, name)
        why = ''
        for attempt in range(self.retries):
            try:
                res = self._run(cmd, capture_output=True, text=True, timeout=10)
                if res.returncode == 0 and 'true' in (res.stdout or '').lower():
                    self.removed.append(name)
                    return True
                why = ((res.stderr or '') + (res.stdout or '')).strip()[:200] or 'reply was not "true"'
            except FileNotFoundError:
                self._log('gz command not found: dirt is counted as cleaned but stays visible')
                self.failed.append(name)
                return False
            except Exception as exc:  # noqa: BLE001 - timeouts and the like: try again
                why = str(exc)[:200]
            if attempt + 1 < self.retries:
                self._sleep(self.backoff)
        self.failed.append(name)
        self._log('could not remove %s from Gazebo (%s). Test by hand: %s'
                  % (name, why, ' '.join(c if ' ' not in c else "'%s'" % c for c in cmd)))
        return False

    def submit(self, name):
        if self._thread is None:
            self._thread = threading.Thread(target=self._work, daemon=True)
            self._thread.start()
        self._q.put(name)

    def _work(self):
        while True:
            name = self._q.get()
            if name is None:
                return
            self.process(name)
            self._on_done()

    def stop(self, timeout=5.0):
        if self._thread is not None:
            self._q.put(None)
            self._thread.join(timeout)


class CleanerCore:
    """Layout + field + removal + files, everything except ROS."""

    def __init__(self, layout, clean_radius=0.20, world='default', output_dir='', remove=True,
                 min_step=0.02, runner=subprocess.run, sleep=time.sleep, log=None, stamp=None):
        self.log = log or (lambda msg: None)
        self.patches, self.meta = load_layout(layout)
        if not self.patches:
            raise ValueError('%s has no dirt patches' % layout)
        self.field = DirtField(self.patches, clean_radius, min_step)
        out = os.path.expanduser(output_dir) if output_dir else os.path.dirname(os.path.abspath(
            os.path.expanduser(layout)))
        os.makedirs(out, exist_ok=True)
        stamp = stamp or datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.events_path = os.path.join(out, 'dirt_events_%s.csv' % stamp)
        self.summary_path = os.path.join(out, 'dirt_summary_%s.json' % stamp)
        with open(self.events_path, 'w', newline='') as f:
            csv.writer(f).writerow(['t_s', 'name', 'kind', 'x_m', 'y_m', 'stubborn', 'remaining'])
        self.remover = None
        if remove:
            if runner is subprocess.run and shutil.which('gz') is None:
                self.log('gz command not found: dirt is counted as cleaned but stays visible')
            else:
                self.remover = Remover(world, runner, sleep, self.log, on_done=self.write_summary)
        self.t0 = None
        self._file_lock = threading.Lock()
        self.write_summary()

    def step(self, x, y, t):
        """Robot at (x, y) at sim time t (seconds). Returns the patches that were cleaned."""
        if self.t0 is None:
            self.t0 = t
        rel = t - self.t0
        done = self.field.update(x, y, rel)
        if done:
            with open(self.events_path, 'a', newline='') as f:
                w = csv.writer(f)
                left = self.field.remaining + len(done)      # count before this move
                for p in done:
                    left -= 1
                    w.writerow(['%.1f' % p.removed_at, p.name, p.kind, '%.3f' % p.x, '%.3f' % p.y,
                                int(p.stubborn), left])
            for p in done:
                self.log('cleaned %s (%s) at %.1f s, %d left' % (p.name, p.kind, p.removed_at,
                                                                  self.field.remaining))
                if self.remover is not None:
                    self.remover.submit(p.name)
            self.write_summary()
        return done

    def write_summary(self):
        with self._file_lock:                  # the remover thread and the main loop both call this
            data = {'layout_seed': self.meta.get('seed'), 'world': self.meta.get('world')}
            data.update(self.field.summary())
            if self.remover is not None:
                data['gazebo_removed'] = len(self.remover.removed)
                data['gazebo_removal_failed'] = list(self.remover.failed)
            with open(self.summary_path, 'w') as f:
                json.dump(data, f, indent=2)

    def close(self):
        if self.remover is not None:
            self.remover.stop()
        self.write_summary()


# --------------------------------------------------------------------------
# ROS 2 node
# --------------------------------------------------------------------------

KIND_RGB = {'crumbs': (0.62, 0.47, 0.28), 'dust': (0.75, 0.75, 0.75),
            'spill': (0.30, 0.13, 0.05), 'hair': (0.1, 0.1, 0.1)}


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Int32, String
    import tf2_ros
    from visualization_msgs.msg import Marker, MarkerArray

    class DirtCleaner(Node):
        def __init__(self):
            super().__init__('dirt_cleaner')
            self.declare_parameter('layout', '~/vacuum_runs/dirt/current_layout.json')
            self.declare_parameter('clean_radius', 0.20)
            self.declare_parameter('world_name', 'default')
            self.declare_parameter('map_frame', 'map')
            self.declare_parameter('robot_frame', 'base_footprint')
            self.declare_parameter('rate', 10.0)
            self.declare_parameter('min_step', 0.02)
            self.declare_parameter('remove_in_gazebo', True)
            self.declare_parameter('output_dir', '')
            self.declare_parameter('markers', True)

            self.map_frame = self.get_parameter('map_frame').value
            self.robot_frame = self.get_parameter('robot_frame').value
            layout = os.path.expanduser(self.get_parameter('layout').value)
            if not os.path.exists(layout):
                raise RuntimeError('dirt layout %s not found: launch with dirt:=N first '
                                   '(or pass -p layout:=...)' % layout)
            self.core = CleanerCore(
                layout, float(self.get_parameter('clean_radius').value),
                self.get_parameter('world_name').value, self.get_parameter('output_dir').value,
                bool(self.get_parameter('remove_in_gazebo').value),
                float(self.get_parameter('min_step').value), log=self.get_logger().info)
            self.show_markers = bool(self.get_parameter('markers').value)

            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.remaining_pub = self.create_publisher(Int32, 'dirt/remaining', 10)
            self.total_pub = self.create_publisher(Int32, 'dirt/total', latched)
            self.state_pub = self.create_publisher(String, 'dirt/state', latched)
            self.marker_pub = self.create_publisher(MarkerArray, 'dirt/markers', latched)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.total_pub.publish(Int32(data=self.core.field.total))
            self._publish_all_markers()
            self._publish_state()
            self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)
            self.create_timer(1.0, self._publish_remaining)
            self.get_logger().info('dirt cleaner: %d patches from %s, brush radius %.2f m'
                                   % (self.core.field.total, layout, self.core.field.clean_radius))
            self.get_logger().info('summary -> %s' % self.core.summary_path)

        def _now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def _tick(self):
            try:
                t = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
            except Exception:  # noqa: BLE001 - tf raises several exception types
                return
            done = self.core.step(t.transform.translation.x, t.transform.translation.y, self._now())
            if done:
                self._publish_remaining()
                self._publish_state()
                if self.show_markers:
                    self._delete_markers(done)

        def _publish_state(self):
            self.state_pub.publish(String(data=json.dumps({'remaining': self.core.field.remaining_names()})))

        def _publish_remaining(self):
            self.remaining_pub.publish(Int32(data=self.core.field.remaining))

        def _marker(self, idx, p):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.ns, m.id = 'dirt', idx
            m.type, m.action = Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = p.x, p.y, 0.01
            m.pose.orientation.w = 1.0
            d = 2.0 * max(p.radius_m, 0.04)
            m.scale.x = m.scale.y = d
            m.scale.z = 0.02
            m.color.r, m.color.g, m.color.b = KIND_RGB.get(p.kind, (0.5, 0.5, 0.5))
            m.color.a = 0.6
            return m

        def _publish_all_markers(self):
            if not self.show_markers:
                return
            arr = MarkerArray()
            for i, p in enumerate(self.core.patches):
                arr.markers.append(self._marker(i, p))
            self.marker_pub.publish(arr)

        def _delete_markers(self, done):
            arr = MarkerArray()
            for p in done:
                m = Marker()
                m.header.frame_id = self.map_frame
                m.ns, m.id = 'dirt', self.core.patches.index(p)
                m.action = Marker.DELETE
                arr.markers.append(m)
            self.marker_pub.publish(arr)

    rclpy.init(args=args)
    node = DirtCleaner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.core.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
