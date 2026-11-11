#!/usr/bin/env python3
"""Auto-labelling of dirt in floor-camera images (no ROS, no Gazebo).

Gazebo knows exactly where every dirt patch is (current_layout.json) and ROS knows where the
camera is (TF), so every image can be labelled for free: project each patch outline into
the image and take the bounding box. Nobody draws boxes by hand.

Camera model: pinhole, ROS optical frame (z forward, x right, y down). ``CameraPose`` holds
the camera-to-world rotation R and the camera position t, so  p_cam = R^T (p_world - t).

Which patches become labels (and which frames are thrown away) is decided in
``label_patches``; the reasons are in the ``Label`` / ``LabelResult`` fields so the recorder
can log them.
"""

import math
from dataclasses import dataclass, field

import numpy as np

# ---- defaults (see label_patches) -----------------------------------------
FAR_M = 2.0              # patches farther than this are not labelled (and are ignored)
NEAR_PLANE_M = 0.03      # points closer than this in front of the lens cannot be projected safely
MIN_W_PX = 4             # smaller boxes are "too small to see"
MIN_H_PX = 3             # (the camera looks down at ~38 deg, so dirt is wide and short)
MIN_VISIBLE = 0.6        # fraction of the box that must be inside the image
AMBIGUOUS_NEAR_M = 1.3   # a patch this close that cannot be labelled makes the frame ambiguous
RAY_MARGIN_M = 0.06      # occlusion ray stops this far short of the patch (patch on a wall edge)


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_fov(cls, width, height, hfov_deg):
        f = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        return cls(f, f, width / 2.0, height / 2.0, width, height)

    @classmethod
    def from_camera_info(cls, msg):
        k = msg.k
        return cls(float(k[0]), float(k[4]), float(k[2]), float(k[5]), int(msg.width), int(msg.height))


@dataclass
class CameraPose:
    R: np.ndarray            # 3x3, camera (optical) axes expressed in the world frame
    t: np.ndarray            # camera position in the world frame

    @classmethod
    def from_quaternion(cls, x, y, z, w, t):
        n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
        x, y, z, w = x / n, y / n, z / n, w / n
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return cls(R, np.array(t, float))

    @classmethod
    def from_mount(cls, x, y, yaw, cam_x=0.1745, cam_z=0.07, pitch_deg=37.6):
        """Pose of a floor camera mounted ``cam_x`` ahead of / ``cam_z`` above the robot origin,
        pitched down by ``pitch_deg``. Same geometry as the URDF (camera_joint + optical joint)."""
        p = math.radians(pitch_deg)
        fwd = np.array([math.cos(p), 0.0, -math.sin(p)])       # optical z, in the robot frame
        right = np.array([0.0, -1.0, 0.0])                      # optical x
        down = np.cross(fwd, right)                             # optical y = z cross x
        c, s = math.cos(yaw), math.sin(yaw)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        R = Rz @ np.column_stack([right, down, fwd])
        t = Rz @ np.array([cam_x, 0.0, cam_z]) + np.array([x, y, 0.0])
        return cls(R, t)

    def to_camera(self, pts):
        return (np.asarray(pts, float) - self.t) @ self.R


def project(pose, K, pts):
    """Project world points (N,3). Returns (uv (N,2), depth (N,))."""
    pc = pose.to_camera(pts)
    z = pc[:, 2]
    zs = np.where(np.abs(z) < 1e-9, 1e-9, z)
    return np.column_stack([K.fx * pc[:, 0] / zs + K.cx, K.fy * pc[:, 1] / zs + K.cy]), z


def pixel_to_floor(pose, K, u, v, floor_z=0.0):
    """World (x, y) where the ray through pixel (u, v) meets the floor; None if it never does."""
    d = pose.R @ np.array([(u - K.cx) / K.fx, (v - K.cy) / K.fy, 1.0])
    if d[2] >= -1e-9:
        return None
    s = (floor_z - pose.t[2]) / d[2]
    p = pose.t + s * d
    return float(p[0]), float(p[1])


def ray_blocked(fmap, a, b, margin=RAY_MARGIN_M):
    """True if the straight 2-D line a->b crosses a wall / unknown cell (stopping ``margin`` short of b)."""
    ax, ay = a
    bx, by = b
    dist = math.hypot(bx - ax, by - ay)
    stop = max(0.0, dist - margin)
    n = max(2, int(stop / (fmap.res * 0.5)) + 1)
    for i in range(n + 1):
        f = (stop * i / n) / dist if dist > 1e-9 else 0.0
        r, c = fmap.world_to_cell(ax + (bx - ax) * f, ay + (by - ay) * f)
        if r < 0 or c < 0 or r >= fmap.shape[0] or c >= fmap.shape[1] or not fmap.free[r, c]:
            return True
    return False


@dataclass
class Label:
    name: str
    kind: str
    x: float                 # box in pixels (clipped to the image)
    y: float
    w: float
    h: float
    visible: float
    range_m: float

    def yolo(self, K, class_id):
        cx, cy = (self.x + self.w / 2.0) / K.width, (self.y + self.h / 2.0) / K.height
        return '%d %.6f %.6f %.6f %.6f' % (class_id, cx, cy, self.w / K.width, self.h / K.height)


@dataclass
class LabelResult:
    labels: list = field(default_factory=list)
    ambiguous: list = field(default_factory=list)     # (patch name, reason)
    ignored: list = field(default_factory=list)       # (patch name, reason) - harmless, not in the image

    @property
    def usable(self):
        """A frame with an ambiguous patch would teach the model that real dirt is background."""
        return not self.ambiguous


def patch_points(p, with_top=True):
    hull = p.get('hull') or [[-p['radius_m'], -p['radius_m']], [p['radius_m'], -p['radius_m']],
                             [p['radius_m'], p['radius_m']], [-p['radius_m'], p['radius_m']]]
    h = float(p.get('height_m', 0.005))
    pts = [(p['x'] + hx, p['y'] + hy, 0.0) for hx, hy in hull]
    if with_top:
        pts += [(p['x'] + hx, p['y'] + hy, h) for hx, hy in hull]
    return np.array(pts, float)


def _in_view(uv, K):
    return len(uv) > 0 and bool(np.any((uv[:, 0] >= 0) & (uv[:, 0] <= K.width) & (uv[:, 1] >= 0) & (uv[:, 1] <= K.height)))


def _sight_points(p):
    """Centre and outline of a patch (offsets), used to see whether a wall hides all, some or none of it."""
    pts = [(0.0, 0.0)] + [tuple(h) for h in (p.get('hull') or [])]
    return pts


def label_patches(patches, pose, K, remaining=None, fmap=None, far_m=FAR_M, min_w=MIN_W_PX,
                  min_h=MIN_H_PX, min_visible=MIN_VISIBLE, ambiguous_near_m=AMBIGUOUS_NEAR_M):
    """Boxes for every dirt patch that is really in the picture.

    patches   layout 'patches' list (name, kind, x, y, hull, height_m, ...)
    remaining set of patch names still on the floor (None = all); cleaned ones are not labelled
    fmap      dirt_world.FloorMap for the occlusion check (None = skip it)
    """
    out = LabelResult()
    cam_xy = (float(pose.t[0]), float(pose.t[1]))
    for p in patches:
        name = p['name']
        if remaining is not None and name not in remaining:
            out.ignored.append((name, 'removed'))
            continue
        rng = math.hypot(p['x'] - cam_xy[0], p['y'] - cam_xy[1])
        if rng > far_m:
            out.ignored.append((name, 'far'))
            continue
        pts = patch_points(p)
        uv, z = project(pose, K, pts)
        if np.all(z < NEAR_PLANE_M):
            out.ignored.append((name, 'behind'))
            continue
        if np.any(z < NEAR_PLANE_M):
            # part of the patch is beside/behind the lens. Only the part in front can show up in
            # the picture, and that is a sliver at the very edge unless the patch is big and close.
            uv, z = uv[z >= NEAR_PLANE_M], z[z >= NEAR_PLANE_M]
            if not _in_view(uv, K):
                out.ignored.append((name, 'beside the camera'))
                continue
            out.ambiguous.append((name, 'straddles near plane'))
            continue
        x0, y0 = uv.min(axis=0)
        x1, y1 = uv.max(axis=0)
        full = max(x1 - x0, 1e-6) * max(y1 - y0, 1e-6)
        cx0, cy0, cx1, cy1 = max(x0, 0.0), max(y0, 0.0), min(x1, K.width), min(y1, K.height)
        if cx1 <= cx0 or cy1 <= cy0:
            out.ignored.append((name, 'outside image'))
            continue
        vis = (cx1 - cx0) * (cy1 - cy0) / full
        if fmap is not None:
            blocked = [ray_blocked(fmap, cam_xy, (p['x'] + hx, p['y'] + hy)) for hx, hy in _sight_points(p)]
            if all(blocked):
                out.ignored.append((name, 'hidden behind a wall'))      # not in the picture: no label needed
                continue
            if any(blocked):
                out.ambiguous.append((name, 'partly occluded'))
                continue
        if vis < min_visible:
            out.ambiguous.append((name, 'cut by image edge (%.0f%% visible)' % (100 * vis)))
            continue
        w, h = cx1 - cx0, cy1 - cy0
        if w < min_w or h < min_h:
            if rng <= ambiguous_near_m:
                out.ambiguous.append((name, 'too small (%.1fx%.1f px)' % (w, h)))
            else:
                out.ignored.append((name, 'too small'))
            continue
        out.labels.append(Label(name, p.get('kind', 'dirt'), cx0, cy0, w, h, vis, rng))
    return out
