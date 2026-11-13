"""ROS 2 node: plan wall-pass + lane coverage on the current map and drive it with Nav2.

Services (std_srvs/Trigger):  ~/plan   build and publish the plan only (preview in RViz)
                              ~/start  plan (if needed) and run the mission
                              ~/stop   cancel the mission
Topics: /coverage/plan_markers (MarkerArray), /coverage/status (String)

Needs a map on /map (nav2 map_server or slam_toolbox) and Nav2 running. Wall loops and
lane pieces are sent as FollowPath goals; the hops between pieces use NavigateToPose.
"""

from dataclasses import fields, replace
import datetime
import math
import os
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from . import core, mission, reclean, report, sim, validate


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CoverageMission(Node):
    def __init__(self):
        super().__init__('coverage_mission')
        defaults = core.CoverageParams()
        for f in fields(defaults):
            self.declare_parameter(f.name, getattr(defaults, f.name))
        self.declare_parameter('auto_margin', True)
        self.declare_parameter('safety', 0.02)
        self.declare_parameter('pursuit_lookahead', 0.25)
        self.declare_parameter('pursuit_speed', 0.25)
        self.declare_parameter('controller_id', 'CoveragePath')
        self.declare_parameter('goal_checker_id', 'goal_checker')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('autostart', False)
        self.declare_parameter('autostart_delay', 5.0)
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('wait_nav2', True)
        # clean-up pass: after the main run, drive to the spots that are still dirty
        self.declare_parameter('reclean', True)
        self.declare_parameter('reclean_passes', 2)
        self.declare_parameter('reclean_slack', 0.08)       # m, Nav2 stops within its goal tolerance
        self.declare_parameter('reclean_min_area', 0.015)    # m^2, ignore smaller dirty patches
        # where the clean-up pass looks for dirt: the coverage tracker's gaps ('map') or
        # the confirmed spots the floor camera publishes on /dirt/detections ('vision')
        self.declare_parameter('reclean_source', 'map')
        self.declare_parameter('detections_topic', 'dirt/detections')
        self.declare_parameter('reclean_min_conf', 0.3)
        self.declare_parameter('reclean_max_age', 30.0)      # s, older detections mean a dead detector
        # run report (CSV + summary + PNG) written when a mission ends
        self.declare_parameter('report', True)
        self.declare_parameter('report_dir', '~/vacuum_runs')
        self.declare_parameter('report_period', 1.0)        # s between coverage.csv rows
        self.declare_parameter('reset_meter_on_start', True)

        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.cb = ReentrantCallbackGroup()
        self.grid_msg = None
        self.plan = None
        self.runner = None
        self.thread = None
        self.lock = threading.Lock()
        self._state_clients = {}
        # state of the running mission (the tracker is stamped from a timer, so guard it)
        self.run_grid = None
        self.tracker = None
        self.log = None
        self.sampling = False
        self.track_lock = threading.Lock()
        self._last_snapshot = 0.0
        self._pass_pieces = {}
        self._cleanup_info = []
        self.det_lock = threading.Lock()
        self._det_text = None
        self._det_received = -1e9

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, latched, callback_group=self.cb)
        self.create_subscription(String, self.get_parameter('detections_topic').value,
                                 self._on_detections, latched, callback_group=self.cb)
        self.markers_pub = self.create_publisher(MarkerArray, 'coverage/plan_markers', latched)
        self.status_pub = self.create_publisher(String, 'coverage/status', latched)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.follow_client = ActionClient(self, FollowPath, 'follow_path', callback_group=self.cb)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb)

        self.create_service(Trigger, '~/plan', self._srv_plan, callback_group=self.cb)
        self.create_service(Trigger, '~/start', self._srv_start, callback_group=self.cb)
        self.create_service(Trigger, '~/stop', self._srv_stop, callback_group=self.cb)
        self.meter_reset = self.create_client(Trigger, '/coverage_meter/reset', callback_group=self.cb)
        self.create_timer(0.1, self._sample_tick, callback_group=self.cb)

        self._say('ready: waiting for /map')
        if self.get_parameter('autostart').value:
            delay = float(self.get_parameter('autostart_delay').value)
            self._auto_timer = self.create_timer(delay, self._autostart_once, callback_group=self.cb)

    # ------------------------------------------------------------ helpers --

    def _say(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def _on_map(self, msg):
        self.grid_msg = msg

    def _on_detections(self, msg):
        """Keep the latest camera spots (latched, so this also fires once at startup)."""
        with self.det_lock:
            self._det_text = msg.data
            self._det_received = self._now()

    def _vision_spots(self):
        """Confirmed camera spots as (spots, ''), or (None, reason) when there are none usable."""
        with self.det_lock:
            text, received = self._det_text, self._det_received
        if text is None:
            return None, 'no spots on %s yet' % self.get_parameter('detections_topic').value
        try:
            stamp, spots = reclean.parse_detections(
                text, min_conf=float(self.get_parameter('reclean_min_conf').value))
        except ValueError as exc:
            return None, str(exc)
        if not spots:
            return None, 'the camera reports nothing confirmed'
        now = self._now()
        # sim time starts near 0, so a 0 stamp means "unknown": fall back to when it arrived
        age = now - stamp if stamp > 0.0 else now - received
        max_age = float(self.get_parameter('reclean_max_age').value)
        if age > max_age:
            return None, 'detections are %.0f s old (is the detector running?)' % age
        return spots, ''

    def _params(self):
        defaults = core.CoverageParams()
        return replace(defaults, **{f.name: self.get_parameter(f.name).value for f in fields(defaults)})

    def _robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001 - tf raises several exception types
            return None
        return (t.transform.translation.x, t.transform.translation.y, quat_to_yaw(t.transform.rotation))

    def _grid(self):
        m = self.grid_msg
        if m is None:
            return None
        o = m.info.origin
        if abs(quat_to_yaw(o.orientation)) > 1e-3:
            self.get_logger().warn('map origin is rotated; coverage assumes a map with yaw 0')
        return core.GridMap.from_occupancy_grid(m.info.width, m.info.height, m.info.resolution,
                                                o.position.x, o.position.y, list(m.data))

    def _nav2_active(self):
        for name in ('bt_navigator', 'controller_server'):
            cli = self._state_clients.get(name)
            if cli is None:
                cli = self.create_client(GetState, '/%s/get_state' % name, callback_group=self.cb)
                self._state_clients[name] = cli
            if not cli.wait_for_service(timeout_sec=1.0):
                return False
            ev = threading.Event()
            fut = cli.call_async(GetState.Request())
            fut.add_done_callback(lambda _f: ev.set())
            if not ev.wait(2.0) or fut.result() is None or fut.result().current_state.label != 'active':
                return False
        return True

    # ----------------------------------------------------------- planning --

    def _build_plan(self):
        grid = self._grid()
        if grid is None:
            raise RuntimeError('no /map received yet')
        pose = self._robot_pose()
        if pose is None:
            raise RuntimeError('no transform %s -> %s yet' % (self.map_frame, self.robot_frame))
        params = self._params()
        start = (pose[0], pose[1])
        if self.get_parameter('auto_margin').value:
            pursuit = sim.PursuitParams(desired_speed=self.get_parameter('pursuit_speed').value,
                                        lookahead=self.get_parameter('pursuit_lookahead').value)
            plan, rep = validate.plan_validated(grid, params, start, pursuit,
                                                safety=self.get_parameter('safety').value)
            self._say('plan: wall margin %.2f m, simulated clearance %.3f m%s, '
                      'coverage %.1f %% of reachable floor (%.1f %% of all free floor), ~%.1f min'
                      % (rep.wall_margin, rep.min_clearance, '' if rep.ok else ' (BELOW TARGET)',
                         rep.coverage_reachable, rep.coverage_free, rep.est_time_s / 60.0))
        else:
            plan = core.plan_coverage(grid, params, start)
        for w in plan.warnings:
            self.get_logger().warn(w)
        self.plan = plan
        self.run_grid = grid
        self._publish_markers(plan)
        s = plan.stats()
        self._say('plan ready: %s' % ', '.join('%s=%s' % kv for kv in s.items()))
        return plan

    def _publish_markers(self, plan):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, piece in enumerate(plan.pieces):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.ns = 'coverage_' + piece.kind
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.02
            m.pose.orientation.w = 1.0
            if piece.kind == 'wall':
                m.color.r, m.color.g, m.color.b = 0.85, 0.28, 0.06
            else:
                m.color.r, m.color.g, m.color.b = 0.10, 0.44, 0.76
            m.color.a = 0.9
            for x, y, _ in piece.poses[::2]:
                m.points.append(Point(x=float(x), y=float(y), z=0.03))
            arr.markers.append(m)
        self.markers_pub.publish(arr)

    # ------------------------------------------------------------ Nav2 I/O --

    def _wait(self, fut, cancel_check=None, timeout=None):
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        end = None if timeout is None else time.monotonic() + timeout
        while not ev.wait(0.2):
            if cancel_check and cancel_check():
                return False
            if end is not None and time.monotonic() > end:
                return False
        return True

    def _pose_msg(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = yaw_to_quat(yaw)
        return msg

    def _run_goal(self, client, goal):
        """Send a goal, wait for the result, cancel it if the mission is cancelled."""
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('action server not available')
            return False
        cancelled = lambda: self.runner is not None and self.runner.cancelled  # noqa: E731
        send = client.send_goal_async(goal)
        if not self._wait(send, cancelled, timeout=10.0):
            return False
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('goal rejected')
            return False
        res = handle.get_result_async()
        if not self._wait(res, cancelled):
            handle.cancel_goal_async()
            return False
        return res.result().status == GoalStatus.STATUS_SUCCEEDED

    def _navigate(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_msg(x, y, yaw)
        return self._run_goal(self.nav_client, goal)

    def _follow(self, poses):
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in poses:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.z, ps.pose.orientation.w = yaw_to_quat(yaw)
            path.poses.append(ps)
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.get_parameter('controller_id').value
        goal.goal_checker_id = self.get_parameter('goal_checker_id').value
        return self._run_goal(self.follow_client, goal)

    # ------------------------------------------------------------- mission --

    def _mission_thread(self):
        try:
            if self.get_parameter('wait_nav2').value:
                for _ in range(120):
                    if self._nav2_active():
                        break
                    time.sleep(1.0)
                else:
                    self._say('Nav2 did not become active; mission not started')
                    return
            plan = self._build_plan()
            if not plan.pieces:
                self._say('nothing to clean')
                return
            self._begin_run(plan)
            do_reclean = bool(self.get_parameter('reclean').value)
            self.runner = mission.MissionRunner(
                plan, self._navigate, self._follow, self._robot_pose,
                log=lambda m: self.get_logger().info(m), status=self._say,
                max_retries=int(self.get_parameter('max_retries').value),
                reclean=self._reclean_plan if do_reclean else None,
                reclean_passes=int(self.get_parameter('reclean_passes').value),
                pass_done=self._pass_done, clock=self._now)
            result = self.runner.run()
            self._finish_run(plan, result)
        except Exception as exc:  # noqa: BLE001 - report instead of dying silently
            self.get_logger().error('mission failed: %s' % exc)
            self._say('mission failed: %s' % exc)
        finally:
            self.sampling = False
            self.runner = None

    # ----------------------------------------- run log, clean-up pass, report --

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _begin_run(self, plan):
        """Fresh coverage tracker and run log; also zero the RViz meter so both start at 0 %."""
        if self.get_parameter('reset_meter_on_start').value and \
                self.meter_reset.wait_for_service(timeout_sec=1.0):
            ev = threading.Event()
            fut = self.meter_reset.call_async(Trigger.Request())
            fut.add_done_callback(lambda _f: ev.set())
            ev.wait(2.0)
        with self.track_lock:
            self.tracker = core.CoverageTracker(self.run_grid, plan.masks.coverable,
                                                plan.params.clean_radius)
            self.log = report.RunLog(self._now())
            self._last_snapshot = -1e9
            self._pass_pieces = {0: list(plan.pieces)}
            self._cleanup_info = []
            self.started_at = datetime.datetime.now().isoformat(timespec='seconds')
        self.sampling = True

    def _sample_tick(self):
        """10 Hz: stamp the cleaned area, keep the trail, and log one coverage row per period."""
        if not self.sampling:
            return
        pose = self._robot_pose()
        if pose is None:
            return
        with self.track_lock:
            if self.tracker is None or self.log is None:
                return
            self.tracker.stamp_xy(pose[0], pose[1])
            self.log.add_position(pose[0], pose[1])
            now = self._now()
            if now - self._last_snapshot >= float(self.get_parameter('report_period').value):
                self._last_snapshot = now
                self.log.snapshot(now, self.tracker.percent(),
                                  report.percent_free(self.tracker, self.run_grid))

    def _pass_done(self, pass_no):
        with self.track_lock:
            if self.log is None or self.tracker is None:
                return
            now = self._now()
            pct = self.tracker.percent()
            pct_free = report.percent_free(self.tracker, self.run_grid)
            self.log.snapshot(now, pct, pct_free)
            self.log.end_pass(pass_no, now, pct, pct_free)
        self._say('%s done: %.1f %% of the reachable floor cleaned'
                  % ('main pass' if pass_no == 0 else 'clean-up pass %d' % pass_no, pct))

    def _reclean_plan(self, pass_no):
        """Spots that still need cleaning, from the map gaps or from the floor camera."""
        pose = self._robot_pose()
        masks = self.plan.masks
        start = (pose[0], pose[1]) if pose is not None else self.run_grid.cell_to_world(*masks.start_cell)
        with self.track_lock:
            covered = self.tracker.covered.copy()
        slack = float(self.get_parameter('reclean_slack').value)
        min_area = float(self.get_parameter('reclean_min_area').value)
        source = str(self.get_parameter('reclean_source').value)
        extra, why = None, ''
        if source == 'vision':
            spots, why = self._vision_spots()
            if spots:
                extra = reclean.plan_reclean_vision(
                    self.run_grid, masks, self.plan.params, spots, covered, start,
                    slack=slack, min_area=min_area,
                    min_conf=float(self.get_parameter('reclean_min_conf').value))
                if extra is None:
                    why = 'the camera saw nothing worth a trip'
            if extra is None:
                self.get_logger().warn('vision clean-up has nothing to plan (%s); '
                                       'falling back to the map' % why)
        if extra is None:
            extra = reclean.plan_reclean(
                self.run_grid, masks, self.plan.params, covered, start,
                slack=slack, min_area=min_area)
        self._cleanup_info.append({'pass': pass_no, 'source': extra.source,
                                   'spots': len(extra.pieces),
                                   'dirty_area_m2': round(extra.missed_m2, 2),
                                   'left_after_spots_m2': round(extra.left_m2, 2),
                                   'detections': extra.detections,
                                   'unseen_m2': round(extra.unseen_m2, 2)})
        if extra.pieces:
            self._pass_pieces[pass_no] = list(extra.pieces)
            self._say('clean-up pass %d (%s): %d spot(s) for %.2f m2 of dirty floor'
                      % (pass_no, extra.source, len(extra.pieces), extra.missed_m2))
        return extra

    def _finish_run(self, plan, result):
        self.sampling = False
        if not self.get_parameter('report').value:
            return
        try:
            stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            out = os.path.join(os.path.expanduser(self.get_parameter('report_dir').value),
                               'run_' + stamp)
            with self.track_lock:
                self.log.snapshot(self._now(), self.tracker.percent(),
                                  report.percent_free(self.tracker, self.run_grid))
                report.write_report(out, self.log, result, self._pass_pieces, self.tracker,
                                    self.run_grid, plan.params, plan.stats(), self._cleanup_info,
                                    self.started_at)
            self._say('run report saved in %s' % out)
        except Exception as exc:  # noqa: BLE001 - a failed report must not look like a failed run
            self.get_logger().error('could not write the run report: %s' % exc)

    def _srv_plan(self, _req, resp):
        try:
            self._build_plan()
            resp.success, resp.message = True, 'plan published on /coverage/plan_markers'
        except Exception as exc:  # noqa: BLE001
            resp.success, resp.message = False, str(exc)
        return resp

    def _srv_start(self, _req, resp):
        with self.lock:
            if self.thread is not None and self.thread.is_alive():
                resp.success, resp.message = False, 'mission already running'
                return resp
            self.thread = threading.Thread(target=self._mission_thread, daemon=True)
            self.thread.start()
        resp.success, resp.message = True, 'mission starting'
        return resp

    def _srv_stop(self, _req, resp):
        if self.runner is None:
            resp.success, resp.message = False, 'no mission running'
        else:
            self.runner.cancel()
            resp.success, resp.message = True, 'cancelling'
        return resp

    def _autostart_once(self):
        self._auto_timer.cancel()
        self._srv_start(None, Trigger.Response())


def main(args=None):
    rclpy.init(args=args)
    node = CoverageMission()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.runner is not None:
            node.runner.cancel()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
