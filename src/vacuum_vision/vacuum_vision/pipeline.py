"""Frame -> floor positions -> spots. Everything the detector node does except talking to ROS."""

from dataclasses import dataclass, field
import math

import numpy as np

from .geometry import pixel_to_floor, project

CLASS_COLORS = {'crumbs': (255, 170, 0), 'dust': (80, 200, 255), 'spill': (255, 60, 60),
                'hair': (190, 120, 255)}


def msg_to_rgb(height, width, step, encoding, data):
    """sensor_msgs/Image fields -> uint8 RGB array (H, W, 3), or None for an unsupported encoding."""
    enc = encoding.lower()
    if enc not in ('rgb8', 'bgr8'):
        return None
    arr = np.frombuffer(bytes(data), np.uint8).reshape(height, step)[:, :width * 3]
    arr = arr.reshape(height, width, 3)
    return np.ascontiguousarray(arr[:, :, ::-1]) if enc == 'bgr8' else arr


def draw_detections(img_rgb, detections):
    """Copy of the image with a box and "kind conf" label per detection (for RViz)."""
    from PIL import Image, ImageDraw
    im = Image.fromarray(np.ascontiguousarray(img_rgb))
    dr = ImageDraw.Draw(im)
    for d in detections:
        col = CLASS_COLORS.get(d.name, (255, 255, 255))
        dr.rectangle([d.x0, d.y0, d.x1, d.y1], outline=col, width=2)
        dr.text((d.x0 + 2, max(0, d.y0 - 11)), '%s %.2f' % (d.name, d.conf), fill=col)
    return np.asarray(im)


@dataclass
class FrameResult:
    detections: list = field(default_factory=list)   # everything the model found (pixels)
    sightings: list = field(default_factory=list)    # those that became floor positions
    spots: list = field(default_factory=list)        # confirmed spots after this frame
    cleared: list = field(default_factory=list)      # spots dropped this frame (cleaned)


class DirtVision:
    def __init__(self, detector, tracker, max_range=1.5, min_w_px=4, min_h_px=3,
                 clear_range=1.2, clear_margin_px=24):
        self.detector = detector
        self.tracker = tracker
        self.max_range = max_range
        self.min_w_px = min_w_px
        self.min_h_px = min_h_px
        self.clear_range = clear_range
        self.clear_margin_px = clear_margin_px

    def _in_clear_zone(self, pose, K, x, y):
        u, v, z = project(pose, K, (x, y, 0.0))
        if z < 0.05:
            return False
        m = self.clear_margin_px
        if not (m <= u <= K.width - m and m <= v <= K.height - m):
            return False
        return math.hypot(x - pose.t[0], y - pose.t[1]) <= self.clear_range

    def process(self, img_rgb, pose, K, t):
        dets = self.detector.detect(img_rgb)
        sightings = []
        for d in dets:
            if d.w < self.min_w_px or d.h < self.min_h_px:
                continue
            p = pixel_to_floor(pose, K, d.cx, d.cy)
            if p is None:
                continue
            if math.hypot(p[0] - pose.t[0], p[1] - pose.t[1]) > self.max_range:
                continue          # far dirt is only a few pixels: position too uncertain
            sightings.append((p[0], p[1], d.name, d.conf))
        cleared = self.tracker.update(
            sightings, t, in_view=lambda x, y: self._in_clear_zone(pose, K, x, y))
        return FrameResult(dets, sightings, self.tracker.confirmed(), cleared)
