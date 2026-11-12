"""Pinhole camera geometry for the floor camera (no ROS). ROS optical frame: z forward, x right, y down.

``CameraPose`` is the camera-to-world rotation R (columns = optical axes in the world/map frame)
and the camera position t, so  p_cam = R^T (p_world - t).  Same convention as
vacuum_gazebo/scripts/dirt_labels.py, which labelled the training images: pixel_to_floor here
is that labelling run backwards.
"""

from dataclasses import dataclass
import math

import numpy as np


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
    R: np.ndarray
    t: np.ndarray

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
        """Camera pose for a robot at (x, y, yaw) with the URDF mount: ``cam_x`` ahead,
        ``cam_z`` above the robot origin, pitched down by ``pitch_deg`` (used by the tests)."""
        p = math.radians(pitch_deg)
        fwd = np.array([math.cos(p), 0.0, -math.sin(p)])
        right = np.array([0.0, -1.0, 0.0])
        down = np.cross(fwd, right)
        c, s = math.cos(yaw), math.sin(yaw)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        R = Rz @ np.column_stack([right, down, fwd])
        t = Rz @ np.array([cam_x, 0.0, cam_z]) + np.array([x, y, 0.0])
        return cls(R, t)


def project(pose, K, point):
    """World point (x, y, z) -> (u, v, depth). depth <= 0 means behind the camera."""
    pc = (np.asarray(point, float) - pose.t) @ pose.R
    z = float(pc[2])
    zs = z if abs(z) > 1e-9 else 1e-9
    return K.fx * float(pc[0]) / zs + K.cx, K.fy * float(pc[1]) / zs + K.cy, z


def pixel_to_floor(pose, K, u, v, floor_z=0.0):
    """World (x, y) where the ray through pixel (u, v) meets the floor; None if it never does."""
    d = pose.R @ np.array([(u - K.cx) / K.fx, (v - K.cy) / K.fy, 1.0])
    if d[2] >= -1e-9:                      # ray points at or above the horizon
        return None
    s = (floor_z - pose.t[2]) / d[2]
    p = pose.t + s * d
    return float(p[0]), float(p[1])
