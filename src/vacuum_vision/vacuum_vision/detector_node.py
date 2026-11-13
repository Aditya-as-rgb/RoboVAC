#!/usr/bin/env python3
"""ROS 2 node: floor camera -> YOLO dirt detector -> dirt spots on the map.

Subscribes   camera/image, camera/camera_info   (sim: bridged from gz, see gz_bridge.yaml)
             TF map -> camera_optical_frame
Publishes    dirt/detections       std_msgs/String   JSON {"stamp", "spots": [{id, x, y, kind, conf, hits}]}
                                   (confirmed spots, map frame; latched, same style as /dirt/state)
             dirt/detected_markers visualization_msgs/MarkerArray  (what the camera thinks is dirty)
             dirt/debug_image      sensor_msgs/Image  camera frame with boxes drawn (only if watched)

/dirt/markers (from dirt_cleaner) is the answer key; /dirt/detected_markers is what the camera sees.
"""

import json

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import CameraPose, Intrinsics
from .pipeline import CLASS_COLORS, DirtVision, draw_detections, msg_to_rgb
from .tracker import SpotTracker
from .yolo import DirtDetector, find_model


class DirtDetectorNode(Node):
    def __init__(self):
        super().__init__('dirt_detector')
        P = self.declare_parameter
        P('model', '')                     # '' = search for models/dirt_detector/dirt_yolo11n.onnx
        P('image_topic', 'camera/image')
        P('info_topic', 'camera/camera_info')
        P('map_frame', 'map')
        P('camera_frame', 'camera_optical_frame')
        P('rate', 5.0)                     # max detections per second (CPU)
        P('conf', 0.25)
        P('iou', 0.5)
        P('threads', 2)
        P('max_range', 1.5)                # m, ignore dirt farther than this from the camera
        P('merge_radius', 0.15)            # m, sightings closer than this are the same patch
        P('min_hits', 3)                   # frames before a spot is confirmed
        P('clear_misses', 10)              # looks at a spot without finding it before it counts as cleaned
        P('clear_range', 1.2)              # m, only look for "gone" this close
        P('debug_image', True)

        g = self.get_parameter
        path = find_model(g('model').value)
        self.get_logger().info('dirt model: %s' % path)
        detector = DirtDetector(path, conf=float(g('conf').value), iou=float(g('iou').value),
                                threads=int(g('threads').value))
        tracker = SpotTracker(merge_radius=float(g('merge_radius').value),
                              min_hits=int(g('min_hits').value),
                              clear_misses=int(g('clear_misses').value))
        self.vision = DirtVision(detector, tracker, max_range=float(g('max_range').value),
                                 clear_range=float(g('clear_range').value))
        self.map_frame = g('map_frame').value
        self.camera_frame = g('camera_frame').value
        self.period = 1.0 / max(float(g('rate').value), 0.1)
        self.debug = bool(g('debug_image').value)
        self.K = None
        self.last_t = -1e9
        self.warned_enc = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.spots_pub = self.create_publisher(String, 'dirt/detections', latched)
        self.marker_pub = self.create_publisher(MarkerArray, 'dirt/detected_markers', latched)
        self.debug_pub = self.create_publisher(Image, 'dirt/debug_image', 5)
        self.create_subscription(CameraInfo, g('info_topic').value, self._on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, g('image_topic').value, self._on_image,
                                 qos_profile_sensor_data)
        self._publish_spots(0.0, [])
        self.get_logger().info('waiting for %s' % g('image_topic').value)

    def _on_info(self, msg):
        if msg.width and msg.k[0]:
            self.K = Intrinsics.from_camera_info(msg)

    def _on_image(self, msg):
        if self.K is None:
            return
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if 0.0 <= t - self.last_t < self.period:
            return
        img = msg_to_rgb(msg.height, msg.width, msg.step, msg.encoding, msg.data)
        if img is None:
            if not self.warned_enc:
                self.get_logger().error('unsupported image encoding %s (need rgb8 or bgr8)' % msg.encoding)
                self.warned_enc = True
            return
        frame = msg.header.frame_id or self.camera_frame
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, frame, msg.header.stamp,
                                                 timeout=Duration(seconds=0.05))
        except TransformException:
            # The camera frame is usually a few milliseconds ahead of odom -> base_footprint
            # (separate publishers on the same sim clock), so the exact-stamp lookup fails
            # often while every other frame succeeds. The mount is rigid: the newest
            # transform is the same camera pose to well under a millimetre.
            try:
                tf = self.tf_buffer.lookup_transform(self.map_frame, frame, Time(),
                                                     timeout=Duration(seconds=0.05))
            except TransformException as e:
                self.get_logger().warn('no TF %s -> %s yet: %s' % (self.map_frame, frame, e),
                                       throttle_duration_sec=5.0)
                return
            self.get_logger().info('image stamp is ahead of TF for %s; using the latest transform'
                                   % frame, throttle_duration_sec=30.0)
        self.last_t = t
        q, p = tf.transform.rotation, tf.transform.translation
        pose = CameraPose.from_quaternion(q.x, q.y, q.z, q.w, (p.x, p.y, p.z))
        res = self.vision.process(img, pose, self.K, t)
        for s in res.cleared:
            self.get_logger().info('spot %d (%s) at (%.2f, %.2f) looks cleaned' % (s.id, s.kind, s.x, s.y))
        self._publish_spots(t, res.spots)
        if self.debug and self.debug_pub.get_subscription_count() > 0:
            out = draw_detections(img, res.detections)
            m = Image()
            m.header = msg.header
            m.height, m.width, m.encoding = out.shape[0], out.shape[1], 'rgb8'
            m.step = out.shape[1] * 3
            m.data = out.tobytes()
            self.debug_pub.publish(m)

    def _publish_spots(self, t, spots):
        body = {'stamp': round(t, 3), 'spots': [
            {'id': s.id, 'x': round(s.x, 3), 'y': round(s.y, 3), 'kind': s.kind,
             'conf': round(s.conf, 3), 'hits': s.hits} for s in spots]}
        self.spots_pub.publish(String(data=json.dumps(body)))
        arr = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        arr.markers.append(wipe)
        for s in spots:
            r, g, b = [c / 255.0 for c in CLASS_COLORS.get(s.kind, (255, 255, 255))]
            m = Marker()
            m.header.frame_id = self.map_frame
            m.ns, m.id, m.type, m.action = 'detected_dirt', s.id, Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = s.x, s.y, 0.01
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.24
            m.scale.z = 0.02
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.75
            arr.markers.append(m)
            label = Marker()
            label.header.frame_id = self.map_frame
            label.ns, label.id, label.type, label.action = 'detected_dirt_label', s.id, Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.x, label.pose.position.y, label.pose.position.z = s.x, s.y, 0.18
            label.pose.orientation.w = 1.0
            label.scale.z = 0.09
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = '%s %.2f' % (s.kind, s.conf)
            arr.markers.append(label)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = DirtDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
