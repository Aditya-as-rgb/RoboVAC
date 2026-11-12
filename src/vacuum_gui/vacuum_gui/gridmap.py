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

"""Occupancy grids, laser scans and pixel<->world maths.

Pure numpy + stdlib: no ROS message and no Qt object is required, which is what lets
``test/test_gridmap.py`` check the conversion without a display. ``qtimages.py`` wraps
the bytes produced here in a QImage.
"""

from dataclasses import dataclass
import math

import numpy as np

UNKNOWN_RGBA = (205, 205, 205, 255)
FREE_RGBA = (254, 254, 254, 255)
WALL_RGBA = (28, 28, 28, 255)

COVERED_RGBA = (120, 210, 140, 120)
COST_RGBA = (70, 120, 220, 90)


@dataclass(frozen=True)
class Grid:
    """An occupancy grid; ``data`` is int8 with row 0 at the bottom (ROS order)."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: np.ndarray
    frame: str = 'map'

    @classmethod
    def from_occupancy(cls, width, height, resolution, origin_x, origin_y, data, frame='map'):
        arr = np.asarray(data, dtype=np.int8).reshape(int(height), int(width))
        return cls(int(width), int(height), float(resolution), float(origin_x),
                   float(origin_y), arr, frame)

    def __post_init__(self):
        if self.data.shape != (self.height, self.width):
            raise ValueError('grid data shape %s does not match %dx%d'
                             % (self.data.shape, self.width, self.height))

    # ------------------------------------------------------------ geometry --
    @property
    def max_x(self):
        return self.origin_x + self.width * self.resolution

    @property
    def max_y(self):
        return self.origin_y + self.height * self.resolution

    def world_to_pixel(self, x, y):
        """World metres -> (col, row) with row 0 at the *top* of the image."""
        col = (float(x) - self.origin_x) / self.resolution
        row = (self.max_y - float(y)) / self.resolution
        return col, row

    def pixel_to_world(self, col, row):
        x = self.origin_x + float(col) * self.resolution
        y = self.max_y - float(row) * self.resolution
        return x, y

    def contains(self, x, y):
        return self.origin_x <= x <= self.max_x and self.origin_y <= y <= self.max_y

    def scene_rect(self):
        """(left, top, width, height) in image pixels for a QGraphicsScene."""
        return (0.0, 0.0, float(self.width), float(self.height))

    def sample(self, x, y):
        col = int((float(x) - self.origin_x) / self.resolution)
        row = int((float(y) - self.origin_y) / self.resolution)
        if 0 <= col < self.width and 0 <= row < self.height:
            return int(self.data[row, col])
        return -1


def rgba_from_grid(grid, free=FREE_RGBA, wall=WALL_RGBA, unknown=UNKNOWN_RGBA):
    """Occupancy grid -> RGBA8888 bytes, first row = top of the image."""
    img = np.empty((grid.height, grid.width, 4), dtype=np.uint8)
    img[...] = unknown
    img[grid.data == 0] = free
    img[grid.data >= 50] = wall
    img = np.flipud(img)
    return img.tobytes()


def coverage_mask(coverage_grid):
    """Boolean (H, W) mask of cells the coverage meter marked cleaned (value 10)."""
    return np.asarray(coverage_grid.data) >= 5


def rgba_with_coverage(grid, coverage_grid, color=COVERED_RGBA):
    """Map image with the cleaned area tinted on top (alpha-blended here, not by Qt)."""
    base = np.frombuffer(rgba_from_grid(grid), dtype=np.uint8).reshape(
        grid.height, grid.width, 4).astype(np.float32)
    mask = np.flipud(coverage_mask(coverage_grid))
    if mask.shape == base.shape[:2] and mask.any():
        alpha = color[3] / 255.0
        tint = np.array(color[:3], dtype=np.float32)
        base[mask, :3] = base[mask, :3] * (1.0 - alpha) + tint * alpha
    return base.astype(np.uint8).tobytes()


def rgba_costmap(grid, values, color=COST_RGBA):
    """Costmap (0..100 or 0..255) as a translucent tint over transparency."""
    arr = np.asarray(values).reshape(grid.height, grid.width)
    img = np.zeros((grid.height, grid.width, 4), dtype=np.uint8)
    hot = np.flipud(arr) > 0
    img[hot] = color
    return img.tobytes()


def quat_to_yaw(x, y, z, w):
    """Yaw from a quaternion, in radians."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def rotate(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


def scan_points(ranges, angle_min, angle_increment, robot_x, robot_y, yaw,
                range_min=0.0, range_max=float('inf'), step=1):
    """LaserScan ranges -> world points, skipping invalid returns."""
    points = []
    step = max(1, int(step))
    for i in range(0, len(ranges), step):
        r = ranges[i]
        if not math.isfinite(r) or r < range_min or r > range_max or r <= 0.0:
            continue
        a = yaw + angle_min + i * angle_increment
        points.append((robot_x + r * math.cos(a), robot_y + r * math.sin(a)))
    return points


def path_polylines(poses, step=1):
    """Nav2 Path poses -> list of (x, y) vertices (one polyline)."""
    pts = []
    step = max(1, int(step))
    for p in poses[::step]:
        pts.append((float(p[0]), float(p[1])))
    return [pts] if pts else []


def nearest(points, x, y):
    """(distance, index) of the closest (x, y) in ``points``; (inf, -1) when empty."""
    best, idx = float('inf'), -1
    for i, (px, py) in enumerate(points):
        d = math.hypot(px - x, py - y)
        if d < best:
            best, idx = d, i
    return best, idx


def bounds_of_points(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    if not xs:
        return None
    return min(xs), min(ys), max(xs), max(ys)
