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

"""rclpy implementation of :class:`~vacuum_gui.backend.RobotBackend`.

One node, one :class:`MultiThreadedExecutor`, one thread. Callbacks never touch Qt
widgets: they store the latest message and emit a signal, which Qt delivers to the GUI
thread through the event loop. Commands from the GUI go the other way through a small
lock-protected queue drained by a node timer, so no rclpy object is used from the GUI
thread.

The node runs with ``use_sim_time=False`` on purpose. With sim time the node's timers
stop whenever Gazebo is not publishing ``/clock`` (before Start, after a crash), so the
dashboard, health checks and TF polling would freeze exactly when you need them. Sim time
is read from ``/clock`` instead and displayed, and staleness is judged on wall time.
"""

import json
import math
import threading
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Joy, LaserScan
from std_msgs.msg import Float32, Int32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from . import gridmap, status
from .backend import (CAP_ACTIONS, CAP_CAMERA, CAP_DETECTOR, CAP_MAP, CAP_MISSION,
                      CAP_PARAMS, CAP_TELEOP, RobotBackend)
from .model import (Dashboard, Detection, Detections, ImageFrame, JoyState, MarkerSet,
                    MapMarker, Nav2Status, ParticleCloud, PathData, PoseEstimate, ScanData)

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
SENSOR = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST)

MAX_IMAGE_WIDTH = 480        # downscale 640x480 -> 480x360 before it reaches Qt
COSTMAP_ALPHA = 90


def _stamp_seconds(msg):
    stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
    if stamp is None:
        return 0.0
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _marker_color(marker):
    c = marker.color
    alpha = c.a if c.a > 0.0 else 1.0
    return (int(max(0.0, min(1.0, c.r)) * 255), int(max(0.0, min(1.0, c.g)) * 255),
            int(max(0.0, min(1.0, c.b)) * 255), int(max(0.05, min(1.0, alpha)) * 255))


def image_to_frame(msg, kind, max_width=MAX_IMAGE_WIDTH):
    """sensor_msgs/Image -> ImageFrame (RGBA8888, row 0 top), downscaled by integer steps."""
    if msg.width <= 0 or msg.height <= 0:
        return None
    encoding = (msg.encoding or '').lower()
    channels = 4 if encoding in ('rgba8', 'bgra8') else 1 if encoding == 'mono8' else 3
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    expected = msg.height * msg.step
    if buf.size < expected:
        return None
    rows = buf[:expected].reshape(msg.height, msg.step)
    pixels = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    step = max(1, int(math.ceil(msg.width / float(max_width))))
    pixels = pixels[::step, ::step]
    out = np.empty((pixels.shape[0], pixels.shape[1], 4), dtype=np.uint8)
    if channels == 1:
        out[..., 0] = pixels[..., 0]
        out[..., 1] = pixels[..., 0]
        out[..., 2] = pixels[..., 0]
    elif channels == 4:
        out[...] = pixels
    elif encoding == 'bgr8':
        out[..., 0] = pixels[..., 2]
        out[..., 1] = pixels[..., 1]
        out[..., 2] = pixels[..., 0]
    else:
        out[..., :3] = pixels
    out[..., 3] = 255
    return ImageFrame(out.shape[1], out.shape[0], out.tobytes(), kind=kind,
                      stamp=_stamp_seconds(msg), encoding=msg.encoding or '')


class GuiNode(Node):
    """Thin ROS node; the backend owns it and reads its buffers."""

    def __init__(self, backend):
        super().__init__('vacuum_gui')
        self.backend = backend
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscriptions_ = [
            self.create_subscription(OccupancyGrid, 'map', self.backend.on_map, LATCHED),
            self.create_subscription(OccupancyGrid, 'coverage_map',
                                     self.backend.on_coverage, LATCHED),
            self.create_subscription(OccupancyGrid, 'global_costmap/costmap',
                                     self.backend.on_global_costmap, LATCHED),
            self.create_subscription(OccupancyGrid, 'local_costmap/costmap',
                                     self.backend.on_local_costmap, LATCHED),
            self.create_subscription(Image, 'camera/image', self.backend.on_camera, SENSOR),
            self.create_subscription(Image, 'dirt/debug_image', self.backend.on_debug, SENSOR),
            self.create_subscription(LaserScan, 'scan', self.backend.on_scan, SENSOR),
            self.create_subscription(Odometry, 'odom', self.backend.on_odom, 20),
            # AMCL publishes /amcl_pose TRANSIENT_LOCAL (once, then on updates): a
            # volatile subscription is incompatible and silently receives nothing.
            self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                     self.backend.on_amcl_pose, LATCHED),
            self.create_subscription(Path, 'plan', self.backend.on_global_plan, 10),
            self.create_subscription(Path, 'local_plan', self.backend.on_local_plan, 10),
            self.create_subscription(MarkerArray, 'coverage/plan_markers',
                                     self.backend.on_coverage_markers, LATCHED),
            self.create_subscription(MarkerArray, 'dirt/markers',
                                     self.backend.on_dirt_markers, LATCHED),
            self.create_subscription(MarkerArray, 'dirt/detected_markers',
                                     self.backend.on_detected_markers, LATCHED),
            self.create_subscription(String, 'dirt/detections',
                                     self.backend.on_detections, LATCHED),
            self.create_subscription(Float32, 'coverage/percent',
                                     self.backend.on_coverage_pct, 10),
            self.create_subscription(Float32, 'coverage/percent_free',
                                     self.backend.on_coverage_free_pct, 10),
            self.create_subscription(String, 'coverage/status',
                                     self.backend.on_mission_status, LATCHED),
            self.create_subscription(Int32, 'dirt/remaining', self.backend.on_dirt_remaining, 10),
            self.create_subscription(Int32, 'dirt/total', self.backend.on_dirt_total, LATCHED),
            self.create_subscription(PoseArray, 'particle_cloud', self.backend.on_particles, 10),
            self.create_subscription(Joy, 'joy', self.backend.on_joy, 10),
            self.create_subscription(Clock, 'clock', self.backend.on_clock, 10),
            self.create_subscription(TFMessage, 'tf', self.backend.on_tf, 100),
        ]
        self.pose_timer = self.create_timer(0.1, self.backend.tick)
        self.slow_timer = self.create_timer(0.5, self.backend.slow_tick)
        self.command_timer = self.create_timer(0.02, self.backend.drain_commands)


class RosBackend(RobotBackend):
    capabilities = frozenset({CAP_MAP, CAP_CAMERA, CAP_TELEOP, CAP_MISSION,
                              CAP_DETECTOR, CAP_ACTIONS, CAP_PARAMS})

    def __init__(self, parent=None, image_rate=10.0, scan_rate=5.0):
        super().__init__(parent)
        self.image_rate = float(image_rate)
        self.scan_rate = float(scan_rate)
        self.map_rate = 2.0
        self.pose_rate = 10.0
        self._lock = threading.Lock()
        self._commands = deque()
        self._node = None
        self._executor = None
        self._thread = None
        self._running = False
        self._error = ''

        # --- latest message state (written by the ROS thread only) ----------
        self.sim_time = 0.0
        self.rtf = status.RealTimeFactor()
        self.coverage_pct = 0.0
        self.coverage_free_pct = 0.0
        self.mission_status = ''
        self.dirt_remaining = -1
        self.dirt_total = -1
        self.speed = 0.0
        self.pose = None
        self.amcl_cov = None
        self.amcl_wall = None
        self.last_detections = Detections()
        self.detected_spots = 0

        self.rates = {key: status.RateMeter(3.0) for key in
                      ('scan', 'odom', 'tf', 'camera', 'clock', 'detector', 'debug')}
        self.last_seen = {}
        self._last_emit = {}
        self._last_map_emit = 0.0
        self._tf_ok = False
        self._tf_missing = []
        self._node_names = set()
        self._nav2_states = {}
        self._expect = {'running': False, 'slam': False, 'coverage': False,
                        'detector': False, 'teleop': False, 'dirt': False}
        self._detector_params = {}
        self._action_client = None
        self._goal_handle = None
        self._trigger_clients = {}
        self._life_clients = {}
        self._param_client = None
        self._param_busy = False
        self._action_state = 'idle'

    # ------------------------------------------------------------- lifecycle --
    def available(self):
        """True when the backend is usable (it may not have started its thread yet)."""
        return self._running or not self._error

    def reason(self):
        return self._error

    def start(self):
        if self._running:
            return
        self._thread = threading.Thread(target=self._run, name='vacuum_gui_ros', daemon=True)
        self._thread.start()

    def _run(self):
        try:
            rclpy.init(args=None)
        except Exception as exc:      # noqa: BLE001 - rclpy raises several types here
            self._error = 'rclpy.init failed: %s' % exc
            self.connected.emit(False)
            self.notice.emit('error', self._error)
            return
        try:
            self._node = GuiNode(self)
            self._executor = MultiThreadedExecutor(num_threads=2)
            self._executor.add_node(self._node)
            self._setup_clients()
            self._running = True
            self.connected.emit(True)
            self._executor.spin()
        except (ExternalShutdownException, KeyboardInterrupt):
            pass
        except Exception as exc:      # noqa: BLE001
            self._error = 'ROS thread stopped: %s' % exc
            self.notice.emit('error', self._error)
        finally:
            self._running = False
            try:
                if self._executor is not None:
                    self._executor.shutdown(timeout_sec=1.0)
            except Exception:      # noqa: BLE001
                pass
            try:
                if self._node is not None:
                    self._node.destroy_node()
            except Exception:      # noqa: BLE001
                pass
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:      # noqa: BLE001
                pass
            self.connected.emit(False)

    def stop(self):
        if not self._running:
            return
        try:
            if self._executor is not None:
                self._executor.shutdown(timeout_sec=1.0)
        except Exception:      # noqa: BLE001
            pass
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def _setup_clients(self):
        node = self._node
        self._action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        for name in ('plan', 'start', 'stop'):
            self._trigger_clients[name] = node.create_client(Trigger, '/coverage_mission/' + name)
        self._trigger_clients['reset_meter'] = node.create_client(Trigger, '/coverage_meter/reset')
        for name in ('controller_server', 'planner_server', 'bt_navigator', 'amcl', 'map_server'):
            self._life_clients[name] = node.create_client(GetState, '/%s/get_state' % name)
        self._param_client = AsyncParameterClient(node, '/dirt_detector')

    # -------------------------------------------------------------- callbacks --
    def _throttled(self, key, hz):
        now = time.monotonic()
        if now - self._last_emit.get(key, 0.0) < 1.0 / max(hz, 0.1):
            return False
        self._last_emit[key] = now
        return True

    def _seen(self, key):
        self.rates[key].add()
        self.last_seen[key] = time.monotonic()

    def _grid(self, msg):
        try:
            return gridmap.Grid.from_occupancy(msg.info.width, msg.info.height,
                                               msg.info.resolution, msg.info.origin.position.x,
                                               msg.info.origin.position.y, msg.data,
                                               msg.header.frame_id or 'map')
        except (ValueError, AttributeError):
            return None

    def on_map(self, msg):
        grid = self._grid(msg)
        if grid is not None and self._throttled('map', self.map_rate):
            self.map_received.emit(grid)

    def on_coverage(self, msg):
        grid = self._grid(msg)
        if grid is not None and self._throttled('coverage', self.map_rate):
            self.coverage_received.emit(grid)

    def on_global_costmap(self, msg):
        if self._throttled('costmap_global', 0.5):
            grid = self._grid(msg)
            if grid is not None:
                self.costmap_received.emit('global', grid)

    def on_local_costmap(self, msg):
        if self._throttled('costmap_local', 0.5):
            grid = self._grid(msg)
            if grid is not None:
                self.costmap_received.emit('local', grid)

    def on_camera(self, msg):
        self._seen('camera')
        if not self._throttled('camera', self.image_rate):
            return
        frame = image_to_frame(msg, 'camera')
        if frame is not None:
            self.image_received.emit('camera', frame)

    def on_debug(self, msg):
        self._seen('debug')
        if not self._throttled('debug', max(2.0, self.image_rate / 2.0)):
            return
        frame = image_to_frame(msg, 'debug')
        if frame is not None:
            self.image_received.emit('debug', frame)

    def on_odom(self, msg):
        self._seen('odom')
        try:
            t = msg.twist.twist
            self.speed = math.hypot(t.linear.x, t.linear.y)
        except AttributeError:
            pass

    def on_amcl_pose(self, msg):
        try:
            cov = msg.pose.covariance
            self.amcl_cov = (float(cov[0]), float(cov[7]), float(cov[35]))
            self.amcl_wall = time.monotonic()
        except (IndexError, AttributeError):
            pass

    def on_scan(self, msg):
        self._seen('scan')
        if not self._throttled('scan', self.scan_rate):
            return
        pose = self.pose
        if pose is None:
            return
        points = gridmap.scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                     pose.x, pose.y, pose.yaw, msg.range_min, msg.range_max,
                                     step=2)
        self.scan_received.emit(ScanData(tuple(points), pose.x, pose.y,
                                         msg.range_min, msg.range_max,
                                         _stamp_seconds(msg)))

    def _path(self, msg, name):
        try:
            frame = msg.header.frame_id or 'map'
            poses = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            transform = None
            if frame not in ('map', '') and self._node is not None:
                transform = self._node.tf_buffer.lookup_transform('map', frame, Time())
            if transform is not None:
                q, t = transform.transform.rotation, transform.transform.translation
                yaw = gridmap.quat_to_yaw(q.x, q.y, q.z, q.w)
                moved = []
                for x, y in poses:
                    rx, ry = gridmap.rotate(x, y, yaw)
                    moved.append((rx + t.x, ry + t.y))
                poses = moved
            polylines = (tuple(poses),) if poses else ()
            self.path_received.emit(name, PathData(polylines, name, _stamp_seconds(msg)))
        except Exception:      # noqa: BLE001 - a failed TF lookup must not kill the node
            return

    def on_global_plan(self, msg):
        if self._throttled('plan_global', 2.0):
            self._path(msg, 'global')

    def on_local_plan(self, msg):
        if self._throttled('plan_local', 4.0):
            self._path(msg, 'local')

    def _markers(self, msg, name):
        if not self._throttled('markers_' + name, 1.0):
            return
        spots, lines = [], []
        for marker in msg.markers:
            if marker.action == Marker.DELETEALL:
                continue
            if marker.type in (Marker.LINE_STRIP, Marker.LINE_LIST):
                pts = tuple((p.x, p.y) for p in marker.points)
                if pts:
                    lines.append(MapMarker(pts[0][0], pts[0][1], 'polyline', marker.ns,
                                           _marker_color(marker), pts,
                                           max(marker.scale.x, marker.scale.y)))
            elif marker.type == Marker.TEXT_VIEW_FACING:
                spots.append(MapMarker(marker.pose.position.x, marker.pose.position.y,
                                       'text', marker.text or marker.ns,
                                       _marker_color(marker),
                                       size=float(marker.scale.z or 0.1)))
            elif marker.type == Marker.ARROW:
                spots.append(MapMarker(marker.pose.position.x, marker.pose.position.y,
                                       'arrow', marker.ns, _marker_color(marker),
                                       size=float(marker.scale.x or 0.2)))
            else:
                spots.append(MapMarker(marker.pose.position.x, marker.pose.position.y,
                                       'spot', marker.ns, _marker_color(marker),
                                       size=float(max(marker.scale.x, marker.scale.y))))
        self.markers_received.emit(name, MarkerSet(tuple(lines + spots), name))

    def on_coverage_markers(self, msg):
        self._markers(msg, 'coverage')

    def on_dirt_markers(self, msg):
        self._markers(msg, 'dirt')

    def on_detected_markers(self, msg):
        self._markers(msg, 'detected')

    def on_detections(self, msg):
        self._seen('detector')
        try:
            body = json.loads(msg.data or '{}')
        except ValueError:
            return
        spots = []
        for item in body.get('spots') or []:
            try:
                spots.append(Detection(int(item.get('id', 0)), float(item['x']),
                                       float(item['y']), str(item.get('kind', 'dirt')),
                                       float(item.get('conf', 0.0)),
                                       int(item.get('hits', 0)),
                                       float(body.get('stamp', 0.0))))
            except (KeyError, TypeError, ValueError):
                continue
        self.detected_spots = len(spots)
        self.last_detections = Detections(tuple(spots), float(body.get('stamp', 0.0)))
        if self._throttled('detections', 2.0):
            self.detections_received.emit(self.last_detections)

    def on_coverage_pct(self, msg):
        self.coverage_pct = float(msg.data)

    def on_coverage_free_pct(self, msg):
        self.coverage_free_pct = float(msg.data)

    def on_mission_status(self, msg):
        text = msg.data or ''
        if text != self.mission_status:
            self.mission_status = text
            self.mission_status_received.emit(text)

    def on_dirt_remaining(self, msg):
        self.dirt_remaining = int(msg.data)

    def on_dirt_total(self, msg):
        self.dirt_total = int(msg.data)

    def on_particles(self, msg):
        if not self._throttled('particles', 2.0):
            return
        poses = []
        for p in msg.poses[:400]:
            poses.append((p.position.x, p.position.y,
                          gridmap.quat_to_yaw(p.orientation.x, p.orientation.y,
                                              p.orientation.z, p.orientation.w)))
        self.particles_received.emit(ParticleCloud(tuple(poses), _stamp_seconds(msg)))

    def on_joy(self, msg):
        if self._throttled('joy', 5.0):
            self.joy_received.emit(JoyState(True, tuple(msg.axes), tuple(msg.buttons),
                                           _stamp_seconds(msg)))

    def on_tf(self, msg):
        self._seen('tf')

    def on_clock(self, msg):
        sim = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        self.sim_time = sim
        self.rtf.add(sim)
        self._seen('clock')

    # ------------------------------------------------------------- periodic --
    def tick(self):
        """10 Hz: pose from TF, and the place where GUI commands are executed."""
        node = self._node
        if node is None:
            return
        self._update_pose(node)
        self._poll_tf(node)

    def _update_pose(self, node):
        try:
            t = node.tf_buffer.lookup_transform('map', 'base_footprint', Time(),
                                                timeout=Duration(seconds=0.02))
            q, p = t.transform.rotation, t.transform.translation
            self.pose = PoseEstimate(p.x, p.y, gridmap.quat_to_yaw(q.x, q.y, q.z, q.w),
                                     _stamp_seconds(t), 'tf')
            self.pose_received.emit(self.pose)
        except Exception:      # noqa: BLE001 - tf raises several incompatible types
            return

    def _poll_tf(self, node):
        if not self._throttled('tf_poll', 1.0):
            return
        missing = []
        for parent, child in (('map', 'odom'), ('odom', 'base_footprint'),
                              ('base_footprint', 'base_scan')):
            try:
                if not node.tf_buffer.can_transform(parent, child, Time()):
                    missing.append('%s->%s' % (parent, child))
            except Exception:      # noqa: BLE001
                missing.append('%s->%s' % (parent, child))
        self._tf_ok = not missing
        self._tf_missing = missing

    def slow_tick(self):
        node = self._node
        if node is None:
            return
        self._node_names = {'/' + ns.rstrip('/') + '/' + name
                            for name, ns in node.get_node_names_and_namespaces()}
        self._node_names = {n.replace('//', '/') for n in self._node_names}
        try:
            self.health_received.emit(self.build_health())
        except Exception:      # noqa: BLE001
            pass
        self.dashboard_received.emit(self.build_dashboard())
        self._poll_lifecycle()
        self._poll_detector_params()

    def build_dashboard(self):
        pose = self.pose
        age = None if self.amcl_wall is None else time.monotonic() - self.amcl_wall
        cov = self.amcl_cov or (None, None, None)
        # /amcl_pose is latched, so its age says nothing about AMCL being alive. The TF
        # chain does: with AMCL running map->odom keeps arriving. No AMCL at all (SLAM or
        # no localisation) is "unknown", not "lost".
        if not self._tf_ok:
            localised = 'lost'
        elif self.amcl_cov is None:
            localised = 'unknown'
        else:
            localised = status.localisation_state(cov[0], cov[1], cov[2],
                                                0.0 if (age is None or age < 300.0) else 301.0)
        states = [s for s in self._nav2_states.values()]
        if not states:
            nav2 = 'unknown'
        elif all(s == 'active' for s in states):
            nav2 = 'active'
        elif any(s == 'active' for s in states):
            nav2 = 'partially active'
        else:
            nav2 = 'inactive'
        return Dashboard(
            sim_time=self.sim_time,
            rtf=self.rtf.value,
            coverage_pct=self.coverage_pct,
            coverage_free_pct=self.coverage_free_pct,
            mission_status=self.mission_status or 'no mission',
            dirt_remaining=self.dirt_remaining,
            dirt_total=self.dirt_total,
            detected_spots=self.detected_spots,
            detector_rate=self.rates['detector'].rate(),
            camera_rate=self.rates['camera'].rate(),
            speed=self.speed,
            localisation=localised,
            nav2_state=nav2,
            pose=pose,
            stamp=time.time(),
        )

    def build_health(self):
        expect = self._expect
        items = []
        now = time.monotonic()

        def rate(key):
            return self.rates[key].rate(now)

        def age(key):
            return now - self.last_seen[key] if key in self.last_seen else float('inf')

        names = self._node_names

        def has(name):
            return ('/' + name) in names or name in names

        # a stack started outside the panel (or left running) still deserves real lights
        running = bool(expect.get('running')) or rate('clock') > 1.0 or any(
            has(n) for n in ('controller_server', 'ros_gz_bridge'))
        if running:
            items.append(status.light_for_topic('/clock', rate('clock'), 5.0,
                                                stale_age=age('clock'), stale_limit=3.0))
        else:
            items.append(status.HealthItem('clock', '/clock', status.UNKNOWN,
                                           'no stack started yet'))

        checks = [('ros_gz_bridge', True, 'ros_gz_bridge'),
                  ('controller_server', True, 'Nav2 controller'),
                  ('bt_navigator', True, 'Nav2 behaviour tree'),
                  ('planner_server', False, 'Nav2 planner'),
                  ('map_server', False, 'map server')]
        if expect.get('slam'):
            checks.append(('slam_toolbox', True, 'SLAM Toolbox'))
        else:
            checks.append(('amcl', True, 'AMCL'))
        if expect.get('coverage'):
            checks.append(('coverage_mission', True, 'mission'))
            checks.append(('coverage_meter', False, 'coverage meter'))
        if expect.get('detector'):
            checks.append(('dirt_detector', True, 'dirt detector'))
        if expect.get('dirt'):
            checks.append(('dirt_cleaner', False, 'dirt brush'))
        if expect.get('teleop'):
            checks.append(('teleop_twist_joy_node', False, 'gamepad teleop'))

        for name, required, label in checks:
            present = has(name)
            if not present and not running:
                items.append(status.HealthItem(name, label, status.UNKNOWN,
                                               'stack not started'))
                continue
            if not present and not required:
                items.append(status.HealthItem(name, label, status.UNKNOWN,
                                               'not running (optional)',
                                               'start it from the Stack tab if you need it'))
                continue
            lifecycle = self._nav2_states.get(name)
            if lifecycle == 'inactive':
                lifecycle = False
            items.append(status.light_for_node(label, present,
                                               is_lifecycle=lifecycle if present else None))

        items.append(status.light_for_topic('/scan', rate('scan'), 2.0,
                                            stale_age=age('scan'), stale_limit=4.0))
        items.append(status.light_for_topic('/odom', rate('odom'), 5.0,
                                            stale_age=age('odom'), stale_limit=3.0))
        items.append(status.light_for_topic('/tf', rate('tf'), 5.0,
                                            stale_age=age('tf'), stale_limit=3.0))
        if expect.get('camera'):
            items.append(status.light_for_topic('/camera/image', rate('camera'), 3.0,
                                                stale_age=age('camera'), stale_limit=3.0))
        if expect.get('detector'):
            items.append(status.light_for_topic('/dirt/detections', rate('detector'), 0.5,
                                                stale_age=age('detector'), stale_limit=6.0))
        items.append(status.tf_health(self._tf_ok, self._tf_missing))
        return items

    def _poll_lifecycle(self):
        if not self._node:
            return
        for name, client in self._life_clients.items():
            try:
                if not client.service_is_ready():
                    self._nav2_states[name] = 'absent'
                    continue
                future = client.call_async(GetState.Request())
                future.add_done_callback(
                    lambda f, key=name: self._lifecycle_done(key, f))
            except Exception:      # noqa: BLE001
                continue
        self.nav2_received.emit(Nav2Status(dict(self._nav2_states)))

    def _lifecycle_done(self, name, future):
        try:
            result = future.result()
            if result is not None:
                self._nav2_states[name] = result.current_state.label
        except Exception:      # noqa: BLE001
            pass

    def _poll_detector_params(self):
        if not self._node or self._param_client is None or self._param_busy:
            return
        if not self._expect.get('detector'):
            return
        if not self._param_client.service_is_ready():
            return
        self._param_busy = True
        names = ['conf', 'iou', 'max_range', 'min_hits', 'clear_misses']
        try:
            future = self._param_client.get_parameters(names)
            future.add_done_callback(self._param_done)
        except Exception:      # noqa: BLE001
            self._param_busy = False

    def _param_done(self, future):
        self._param_busy = False
        try:
            values = future.result().values
            data = {}
            for name, value in zip(('conf', 'iou', 'max_range', 'min_hits', 'clear_misses'),
                                   values):
                data[name] = value.double_value if value.type_ == value.PARAMETER_DOUBLE \
                    else value.integer_value
            if data and data != self._detector_params:
                self._detector_params = data
                self.detector_params_received.emit(dict(data))
        except Exception:      # noqa: BLE001
            pass

    # --------------------------------------------------------------- commands --
    def set_expectations(self, expectations):
        self._expect.update(expectations or {})

    def set_image_rate(self, hz):
        self.image_rate = max(1.0, float(hz))

    def set_scan_rate(self, hz):
        self.scan_rate = max(1.0, float(hz))

    def _post(self, fn):
        with self._lock:
            self._commands.append(fn)

    def drain_commands(self):
        while True:
            with self._lock:
                if not self._commands:
                    return
                fn = self._commands.popleft()
            try:
                fn()
            except Exception as exc:      # noqa: BLE001
                self.notice.emit('warn', 'command failed: %s' % exc)

    def _stamp(self):
        """Sim time when /clock is flowing, else wall time (Nav2 accepts both)."""
        if self._node is None:
            return None
        from builtin_interfaces.msg import Time as TimeMsg
        seconds = self.sim_time if self.sim_time > 0.0 else time.time()
        msg = TimeMsg()
        msg.sec = int(seconds)
        msg.nanosec = int((seconds - int(seconds)) * 1e9)
        return msg

    def request_initial_pose(self, x, y, yaw):
        def do():
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            stamp = self._stamp()
            if stamp is not None:
                msg.header.stamp = stamp
            msg.pose.pose.position.x = float(x)
            msg.pose.pose.position.y = float(y)
            msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
            msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
            cov = [0.0] * 36
            cov[0] = cov[7] = 0.25
            cov[35] = 0.0685
            msg.pose.covariance = cov
            if not hasattr(self, '_initialpose_pub'):
                self._initialpose_pub = self._node.create_publisher(
                    PoseWithCovarianceStamped, 'initialpose', 10)
            self._initialpose_pub.publish(msg)
            self.notice.emit('info', '2D pose estimate sent: (%.2f, %.2f, %.0f deg)'
                             % (x, y, math.degrees(yaw)))
        self._post(do)

    def send_goal(self, x, y, yaw=0.0):
        def do():
            client = self._action_client
            if client is None:
                return
            if not client.wait_for_server(timeout_sec=1.0):
                self.notice.emit('error', 'Nav2 action server /navigate_to_pose is not up')
                return
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            stamp = self._stamp()
            if stamp is not None:
                goal.pose.header.stamp = stamp
            goal.pose.pose.position.x = float(x)
            goal.pose.pose.position.y = float(y)
            goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
            goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
            self._action_state = 'navigating'
            self.action_state_received.emit(self._action_state)
            future = client.send_goal_async(goal, feedback_callback=self._goal_feedback)
            future.add_done_callback(self._goal_accepted)
        self._post(do)

    def _goal_accepted(self, future):
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self._action_state = 'rejected'
                self.action_state_received.emit(self._action_state)
                return
            self._goal_handle = handle
            handle.get_result_async().add_done_callback(self._goal_result)
        except Exception as exc:      # noqa: BLE001
            self.notice.emit('warn', 'goal failed: %s' % exc)

    def _goal_feedback(self, msg):
        self.action_state_received.emit('navigating')

    def _goal_result(self, future):
        try:
            status_code = future.result().status
            if status_code == GoalStatus.STATUS_SUCCEEDED:
                self._action_state = 'succeeded'
            elif status_code == GoalStatus.STATUS_CANCELED:
                self._action_state = 'cancelled'
            else:
                self._action_state = 'failed'
        except Exception:      # noqa: BLE001
            self._action_state = 'unknown'
        self.action_state_received.emit(self._action_state)

    def cancel_nav_goal(self):
        def do():
            handle = self._goal_handle
            if handle is None:
                return
            handle.cancel_goal_async()
            self._goal_handle = None
            self._action_state = 'cancelled'
            self.action_state_received.emit(self._action_state)
        self._post(do)

    def publish_cmd_vel(self, linear, angular):
        def do():
            if not hasattr(self, '_cmd_pub'):
                self._cmd_pub = self._node.create_publisher(Twist, 'cmd_vel', 10)
            msg = Twist()
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
            self._cmd_pub.publish(msg)
        self._post(do)

    def call_trigger(self, service):
        def do():
            client = self._trigger_clients.get(service)
            if client is None:
                return
            if not client.service_is_ready():
                self.notice.emit('error', 'service %s is not available yet' % service)
                return
            future = client.call_async(Trigger.Request())
            future.add_done_callback(
                lambda f: self.notice.emit('info', '%s: %s' % (
                    service, (f.result().message if f.result() else 'no reply'))))
        self._post(do)

    def set_detector_params(self, values):
        def do():
            if not self._param_client or not self._param_client.service_is_ready():
                self.notice.emit('error', 'dirt detector is not running')
                return
            from rcl_interfaces.msg import Parameter as ParameterMsg
            params = []
            for key, value in values.items():
                msg = ParameterMsg()
                msg.name = str(key)
                if isinstance(value, bool):
                    msg.value.type = 1
                    msg.value.bool_value = bool(value)
                elif isinstance(value, int):
                    msg.value.type = 2
                    msg.value.integer_value = int(value)
                else:
                    msg.value.type = 3
                    msg.value.double_value = float(value)
                params.append(msg)
            try:
                future = self._param_client.set_parameters(params)
                future.add_done_callback(
                    lambda f: self.notice.emit('info', 'detector parameters applied'))
                self._detector_params.update(values)
                self.detector_params_received.emit(dict(self._detector_params))
            except Exception as exc:      # noqa: BLE001
                self.notice.emit('warn', 'parameter set failed: %s' % exc)
        self._post(do)

    def request_detector_params(self):
        self._post(lambda: setattr(self, '_param_busy', False))
