"""ROS 2 node: coverage meter. Shows what the robot has cleaned and how much.

Subscribes /map (locks onto the first map it gets, so use it with a saved map) and the
robot pose from TF, stamps a disc of ``clean_radius`` around the robot each cycle.

Publishes:
  /coverage_map          nav_msgs/OccupancyGrid  cleaned floor = 10, everything else 0
                         (add it in RViz as a Map display with the "costmap" scheme)
  /coverage/percent      std_msgs/Float32        % of the floor the robot can reach and clean
  /coverage/percent_free std_msgs/Float32        % of ALL free floor cells in the map
  /coverage/text         visualization_msgs/Marker  readable label for RViz
Service ~/reset (std_srvs/Trigger) clears the cleaned area.
"""

from dataclasses import fields, replace

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
import tf2_ros
from visualization_msgs.msg import Marker

from . import core


class CoverageMeter(Node):
    def __init__(self):
        super().__init__('coverage_meter')
        d = core.CoverageParams()
        # the meter must use the same reachability settings as the plan, so a 'reachable'
        # percentage means the same thing; wall_margin 0.11 is what the planner settles on
        # for the living room (see README)
        d = replace(d, wall_margin=0.11)
        for f in fields(d):
            self.declare_parameter(f.name, getattr(d, f.name))
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('covered_value', 10)

        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.grid = None
        self.map_msg = None
        self.tracker = None
        self.cov_free_total = 1
        self.dirty = True

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, latched)
        self.cov_pub = self.create_publisher(OccupancyGrid, 'coverage_map', latched)
        self.pct_pub = self.create_publisher(Float32, 'coverage/percent', 10)
        self.free_pub = self.create_publisher(Float32, 'coverage/percent_free', 10)
        self.text_pub = self.create_publisher(Marker, 'coverage/text', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_service(Trigger, '~/reset', self._srv_reset)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)
        self.create_timer(float(self.get_parameter('publish_period').value), self._publish)

    def _params(self):
        d = core.CoverageParams()
        return replace(d, **{f.name: self.get_parameter(f.name).value for f in fields(d)})

    def _robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        return t.transform.translation.x, t.transform.translation.y

    def _on_map(self, msg):
        if self.map_msg is None:
            self.map_msg = msg
            self.get_logger().info('map received (%dx%d); waiting for the robot pose'
                                   % (msg.info.width, msg.info.height))

    def _setup(self, xy):
        m = self.map_msg
        o = m.info.origin.position
        self.grid = core.GridMap.from_occupancy_grid(m.info.width, m.info.height, m.info.resolution,
                                                     o.x, o.y, list(m.data))
        params = self._params()
        masks = core.build_masks(self.grid, params, xy)
        for w in masks.warnings:
            self.get_logger().warn(w)
        self.tracker = core.CoverageTracker(self.grid, masks.coverable, params.clean_radius)
        self.cov_free_total = max(1, int((self.grid.data == 0).sum()))

    def _tick(self):
        if self.map_msg is None:
            return
        xy = self._robot_xy()
        if xy is None:
            return
        if self.tracker is None:
            try:
                self._setup(xy)
            except ValueError as exc:
                self.get_logger().error('coverage meter: %s' % exc)
                self.map_msg = None
                return
        before = int(self.tracker.covered.sum())
        self.tracker.stamp_xy(xy[0], xy[1])
        if int(self.tracker.covered.sum()) != before:
            self.dirty = True

    def _publish(self):
        if self.tracker is None:
            return
        pct = float(self.tracker.percent())
        free = 100.0 * int((self.tracker.covered & (self.grid.data == 0)).sum()) / self.cov_free_total
        self.pct_pub.publish(Float32(data=pct))
        self.free_pub.publish(Float32(data=float(free)))
        self._publish_text(pct)
        if not self.dirty:
            return
        self.dirty = False
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self.map_msg.info
        img = np.zeros(self.grid.data.shape, dtype=np.int8)
        img[self.tracker.covered & (self.grid.data == 0)] = int(self.get_parameter('covered_value').value)
        msg.data = img.flatten().tolist()
        self.cov_pub.publish(msg)

    def _publish_text(self, pct):
        info = self.map_msg.info
        m = Marker()
        m.header.frame_id = self.map_frame
        m.ns, m.id = 'coverage_text', 0
        m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position = Point(x=info.origin.position.x + 0.3,
                                y=info.origin.position.y + info.height * info.resolution - 0.2,
                                z=0.5)
        m.pose.orientation.w = 1.0
        m.scale.z = 0.25
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.text = 'Cleaned: %.1f %%' % pct
        self.text_pub.publish(m)

    def _srv_reset(self, _req, resp):
        if self.tracker is not None:
            self.tracker.covered[:] = False
            self.dirty = True
        resp.success, resp.message = True, 'coverage cleared'
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = CoverageMeter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
