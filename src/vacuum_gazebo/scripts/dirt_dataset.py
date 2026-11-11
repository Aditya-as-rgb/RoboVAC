#!/usr/bin/env python3
"""Records a labelled dirt dataset while the robot cleans (YOLO format).

While the coverage mission runs, every camera frame that is worth keeping is saved together
with a box for each dirt patch in the picture. The boxes come from the ground-truth layout
(dirt_labels.py), so there is no hand labelling.

    <out>/classes.txt                    one class name per line (class id = line number)
    <out>/<episode>/images/000123.jpg
    <out>/<episode>/labels/000123.txt    "class cx cy w h" per patch, normalised (empty = no dirt)
    <out>/<episode>/meta.csv             one row per saved frame: time, robot/camera pose, counts
    <out>/<episode>/episode.json         layout seed, environment, counts, why frames were dropped

Use it through the launch file:
    ros2 launch vacuum_bringup vacuum.launch.py collect_dataset:=true autostart:=true \\
        dirt:=15 dirt_seed:=3 dirt_clean:=false dirt_randomize:=true teleop:=none headless:=true

Keep dirt_clean:=false while collecting: the dirt stays on the floor, so the camera sees it
from every side. Labels are in the `map` frame, so keep the default ground-truth odometry
(odom_source:=ground_truth); with AMCL the pose drifts and the boxes would too.

Frame policy (all in ``FramePolicy``): save only when the robot has moved or turned enough
since the last saved frame (near-duplicate frames add nothing), at most ``max_rate`` Hz, and
drop any frame with an ambiguous patch (see dirt_labels.label_patches). Frames with no dirt are
kept with probability ``neg_prob`` as background examples.
"""

import csv
from dataclasses import dataclass
import importlib.util
import json
import math
import os
import random
import sys
import time

import numpy as np


def _sibling(name):
    """Import dirt_labels.py / dirt_world.py that sit next to this script (installed or not)."""
    here = os.path.dirname(os.path.abspath(__file__))
    spec = importlib.util.spec_from_file_location(name, os.path.join(here, name + '.py'))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


dirt_labels = _sibling('dirt_labels')
dirt_world = _sibling('dirt_world')
CLASSES = list(dirt_world.KINDS)


@dataclass
class FramePolicy:
    min_move_m: float = 0.12
    min_turn_deg: float = 12.0
    max_rate_hz: float = 3.0
    neg_prob: float = 0.12


def yaw_of(pose):
    """Heading of the camera's viewing direction projected on the floor (radians)."""
    f = pose.R[:, 2]
    return math.atan2(f[1], f[0])


def angle_diff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


class DatasetRecorder:
    META_COLS = ['frame', 't_s', 'cam_x', 'cam_y', 'cam_z', 'yaw_deg', 'n_labels', 'n_dirt_left', 'status']

    def __init__(self, out_dir, episode, layout, fmap=None, policy=None, seed=0, jpeg_quality=92):
        self.root = os.path.expanduser(out_dir)
        self.dir = os.path.join(self.root, episode)
        os.makedirs(os.path.join(self.dir, 'images'), exist_ok=True)
        os.makedirs(os.path.join(self.dir, 'labels'), exist_ok=True)
        with open(os.path.join(self.root, 'classes.txt'), 'w') as f:
            f.write('\n'.join(CLASSES) + '\n')
        self.layout = layout
        self.patches = layout['patches']
        self.fmap = fmap
        self.policy = policy or FramePolicy()
        self.rng = random.Random(seed)
        self.quality = jpeg_quality
        self.saved = 0
        self.seen = 0
        self.dropped = {'too_soon': 0, 'too_close_to_last': 0, 'ambiguous': 0, 'background_skipped': 0}
        self.positives = 0
        self.negatives = 0
        self._last = None            # (t, x, y, yaw) of the last saved frame
        self._meta_f = open(os.path.join(self.dir, 'meta.csv'), 'w', newline='')
        self._meta = csv.writer(self._meta_f)
        self._meta.writerow(self.META_COLS)

    def offer(self, t, image, pose, K, remaining=None, status=''):
        """One camera frame (uint8 HxWx3 RGB). Returns the LabelResult if it was saved, else None."""
        self.seen += 1
        pol = self.policy
        if self._last is not None and t - self._last[0] < 1.0 / pol.max_rate_hz:
            self.dropped['too_soon'] += 1
            return None
        x, y, yaw = float(pose.t[0]), float(pose.t[1]), yaw_of(pose)
        if self._last is not None:
            moved = math.hypot(x - self._last[1], y - self._last[2])
            turned = math.degrees(angle_diff(yaw, self._last[3]))
            if moved < pol.min_move_m and turned < pol.min_turn_deg:
                self.dropped['too_close_to_last'] += 1
                return None
        res = dirt_labels.label_patches(self.patches, pose, K, remaining=remaining, fmap=self.fmap)
        if not res.usable:
            self.dropped['ambiguous'] += 1
            return None
        if not res.labels and self.rng.random() >= pol.neg_prob:
            self.dropped['background_skipped'] += 1
            return None
        self._save(t, image, pose, K, res, remaining, status)
        self._last = (t, x, y, yaw)
        return res

    def _save(self, t, image, pose, K, res, remaining, status):
        from PIL import Image
        stem = '%06d' % self.saved
        Image.fromarray(np.ascontiguousarray(image[:, :, :3])).save(
            os.path.join(self.dir, 'images', stem + '.jpg'), quality=self.quality)
        kinds = {p['name']: p.get('kind') for p in self.patches}
        lines = [lab.yolo(K, CLASSES.index(kinds.get(lab.name, lab.kind))) for lab in res.labels]
        with open(os.path.join(self.dir, 'labels', stem + '.txt'), 'w') as f:
            f.write('\n'.join(lines) + ('\n' if lines else ''))
        left = len(self.patches) if remaining is None else len(remaining)
        self._meta.writerow([stem, '%.3f' % t, '%.4f' % pose.t[0], '%.4f' % pose.t[1], '%.4f' % pose.t[2],
                             '%.1f' % math.degrees(yaw_of(pose)), len(res.labels), left, status])
        self._meta_f.flush()
        self.saved += 1
        if res.labels:
            self.positives += 1
        else:
            self.negatives += 1

    def close(self):
        self._meta_f.close()
        info = {'layout_seed': self.layout.get('seed'), 'world': self.layout.get('world'),
                'environment': self.layout.get('environment'), 'n_patches': len(self.patches),
                'classes': CLASSES, 'frames_seen': self.seen, 'frames_saved': self.saved,
                'with_dirt': self.positives, 'background': self.negatives,
                'dropped': self.dropped,
                'policy': self.policy.__dict__, 'saved_at': time.strftime('%Y-%m-%d %H:%M:%S')}
        with open(os.path.join(self.dir, 'episode.json'), 'w') as f:
            json.dump(info, f, indent=2)
        return info


def image_msg_to_rgb(msg):
    """sensor_msgs/Image (rgb8 / bgr8 / mono8) -> uint8 HxWx3 RGB."""
    ch = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(msg.encoding)
    if ch is None:
        raise ValueError('unsupported image encoding %r' % msg.encoding)
    raw = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch]
    img = raw.reshape(msg.height, msg.width, ch)
    if ch == 1:
        img = np.repeat(img, 3, axis=2)
    elif msg.encoding == 'bgr8':
        img = img[:, :, ::-1]
    return img


# --------------------------------------------------------------------------
# ROS 2 node
# --------------------------------------------------------------------------

def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    import tf2_ros

    class DatasetNode(Node):
        def __init__(self):
            super().__init__('dirt_dataset')
            for name, default in [('layout', '~/vacuum_runs/dirt/current_layout.json'),
                                  ('map', ''), ('out_dir', '~/vacuum_runs/dataset'),
                                  ('episode', ''), ('map_frame', 'map'),
                                  ('min_move_m', 0.12), ('min_turn_deg', 12.0), ('max_rate_hz', 3.0),
                                  ('neg_prob', 0.12), ('max_duration_s', 3600.0),
                                  ('finish_wait_s', 8.0), ('seed', 0)]:
                self.declare_parameter(name, default)
            g = lambda n: self.get_parameter(n).value  # noqa: E731
            layout_path = os.path.expanduser(g('layout'))
            with open(layout_path) as f:
                layout = json.load(f)
            map_yaml = os.path.expanduser(g('map') or layout.get('map') or '')
            fmap = dirt_world.load_map(map_yaml) if map_yaml and os.path.exists(map_yaml) else None
            episode = g('episode') or 'ep_%s_seed%s' % (time.strftime('%Y%m%d_%H%M%S'), layout.get('seed'))
            pol = FramePolicy(float(g('min_move_m')), float(g('min_turn_deg')),
                              float(g('max_rate_hz')), float(g('neg_prob')))
            self.rec = DatasetRecorder(g('out_dir'), episode, layout, fmap, pol, int(g('seed')))
            self.map_frame = g('map_frame')
            self.max_duration = float(g('max_duration_s'))
            self.finish_wait = float(g('finish_wait_s'))
            self.K = None
            self.remaining = None
            self.status = ''
            self.t0 = None
            self.finished_at = None
            self.done = False
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(CameraInfo, 'camera/camera_info', self._info, 10)
            self.create_subscription(Image, 'camera/image', self._image, 5)
            self.create_subscription(String, 'dirt/state', self._state, latched)
            self.create_subscription(String, 'coverage/status', self._status, latched)
            self.create_timer(1.0, self._check_done)
            self.get_logger().info('recording %d patches -> %s' % (len(layout['patches']), self.rec.dir))

        def _now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def _info(self, msg):
            if self.K is None:
                self.K = dirt_labels.Intrinsics.from_camera_info(msg)

        def _state(self, msg):
            try:
                self.remaining = set(json.loads(msg.data)['remaining'])
            except (ValueError, KeyError):
                pass

        def _status(self, msg):
            self.status = msg.data
            low = msg.data.lower()
            if self.finished_at is None and (low.startswith('finished') or low.startswith('cancelled')):
                self.finished_at = self._now()
                self.get_logger().info('mission %s: stopping in %.0f s' % (low.split()[0], self.finish_wait))

        def _image(self, msg):
            if self.K is None or self.done:
                return
            try:
                t = rclpy.time.Time.from_msg(msg.header.stamp)
                try:
                    tf = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id or 'camera_optical_frame', t)
                except Exception:  # noqa: BLE001 - fall back to the newest transform
                    tf = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id or 'camera_optical_frame',
                                                         rclpy.time.Time())
                r, tr = tf.transform.rotation, tf.transform.translation
                pose = dirt_labels.CameraPose.from_quaternion(r.x, r.y, r.z, r.w, [tr.x, tr.y, tr.z])
                if self.t0 is None:
                    self.t0 = self._now()
                self.rec.offer(self._now(), image_msg_to_rgb(msg), pose, self.K, self.remaining, self.status)
            except Exception as e:  # noqa: BLE001 - one bad frame must not stop a long collection
                self.get_logger().warn('frame skipped: %s' % e, throttle_duration_sec=10.0)

        def _check_done(self):
            now = self._now()
            if self.finished_at is not None and now - self.finished_at >= self.finish_wait:
                self._finish('mission finished')
            elif self.t0 is not None and now - self.t0 >= self.max_duration:
                self._finish('max_duration_s reached')

        def _finish(self, why):
            if self.done:
                return
            self.done = True
            info = self.rec.close()
            self.get_logger().info('%s: %d frames saved (%d with dirt, %d background) in %s'
                                   % (why, info['frames_saved'], info['with_dirt'], info['background'], self.rec.dir))
            raise SystemExit(0)

    rclpy.init(args=args)
    node = DatasetNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if not node.done:
            node.rec.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
