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

"""Occupancy grid -> RGBA, world<->pixel, scan and path maths (no display)."""

import math

import numpy as np

from vacuum_gui import gridmap


def make_grid(values=((100, 0), (0, 0))):
    height, width = len(values), len(values[0])
    return gridmap.Grid.from_occupancy(width, height, 0.5, -1.0, -1.0,
                                       np.array(values, dtype=np.int8).ravel())


def test_world_to_pixel_puts_y_up_and_row_zero_on_top():
    grid = make_grid()
    # grid covers x in [-1, 0], y in [-1, 0]; max_y is 0 -> (0,0) is top-right
    assert grid.world_to_pixel(0.0, 0.0) == (2.0, 0.0)
    assert grid.world_to_pixel(-1.0, -1.0) == (0.0, 2.0)
    x, y = grid.pixel_to_world(2.0, 0.0)
    assert (x, y) == (0.0, 0.0)


def test_pixel_round_trip_is_stable_for_centres():
    grid = gridmap.Grid.from_occupancy(10, 8, 0.05, 1.2, -0.4, np.zeros(80, dtype=np.int8))
    for col, row in ((0.5, 0.5), (4.5, 3.5), (9.5, 7.5)):
        x, y = grid.pixel_to_world(col, row)
        c2, r2 = grid.world_to_pixel(x, y)
        assert abs(c2 - col) < 1e-9 and abs(r2 - row) < 1e-9


def test_rgba_row_zero_is_the_top_of_the_image():
    # grid rows: 0 = (wall, free) at the bottom, 1 = (free, free) at the top
    img = np.frombuffer(gridmap.rgba_from_grid(make_grid()), dtype=np.uint8)
    img = img.reshape(2, 2, 4)
    assert tuple(img[0, 0]) == gridmap.FREE_RGBA        # top-left = grid row 1 col 0
    assert tuple(img[1, 0]) == gridmap.WALL_RGBA        # bottom-left = grid row 0 col 0
    assert tuple(img[0, 1]) == gridmap.FREE_RGBA        # top-right = grid row 1 col 1
    assert tuple(img[1, 1]) == gridmap.FREE_RGBA        # bottom-right = grid row 0 col 1


def test_unknown_cells_stay_grey():
    grid = make_grid(((100, 0), (-1, 0)))
    img = np.frombuffer(gridmap.rgba_from_grid(grid), dtype=np.uint8).reshape(2, 2, 4)
    # grid row 1 col 0 is -1 (unknown) -> image row 0 col 0
    assert tuple(img[0, 0]) == gridmap.UNKNOWN_RGBA


def test_coverage_overlay_tints_only_the_cleaned_cells():
    grid = make_grid(((0, 0), (0, 0)))
    coverage = gridmap.Grid.from_occupancy(2, 2, 0.5, -1.0, -1.0,
                                            np.array([[10, 0], [0, 0]], dtype=np.int8))
    base = np.frombuffer(gridmap.rgba_from_grid(grid), dtype=np.uint8).reshape(2, 2, 4)
    tinted = np.frombuffer(gridmap.rgba_with_coverage(grid, coverage),
                           dtype=np.uint8).reshape(2, 2, 4)
    # covered cell (grid row 0, col 0) is the bottom-left pixel
    assert not np.array_equal(tinted[1, 0, :3], base[1, 0, :3])
    assert np.array_equal(tinted[0, 1, :3], base[0, 1, :3])
    assert np.array_equal(tinted[1, 0, 3], base[1, 0, 3])   # alpha untouched


def test_scan_points_skip_invalid_ranges_and_respect_angles():
    ranges = [1.0, float('inf'), 0.0, 2.0, 0.05]
    pts = gridmap.scan_points(ranges, angle_min=0.0, angle_increment=math.pi / 2,
                              robot_x=1.0, robot_y=2.0, yaw=0.0, range_min=0.1)
    assert len(pts) == 2
    assert (round(pts[0][0], 6), round(pts[0][1], 6)) == (2.0, 2.0)
    assert (round(pts[1][0], 6), round(pts[1][1], 6)) == (1.0, 0.0)   # 270 deg


def test_scan_points_apply_the_robot_heading():
    pts = gridmap.scan_points([1.0], 0.0, 0.0, 0.0, 0.0, math.pi / 2)
    assert abs(pts[0][0]) < 1e-9 and abs(pts[0][1] - 1.0) < 1e-9


def test_quat_to_yaw_for_known_rotations():
    assert abs(gridmap.quat_to_yaw(0, 0, 0, 1)) < 1e-9
    yaw = math.pi / 2
    assert abs(gridmap.quat_to_yaw(0, 0, math.sin(yaw / 2), math.cos(yaw / 2)) - yaw) < 1e-9


def test_nearest_and_path_polylines():
    dist, idx = gridmap.nearest([(0, 0), (3, 4)], 2.6, 3.5)
    assert idx == 1 and abs(dist - 0.6403) < 1e-3
    assert gridmap.nearest([], 0, 0) == (float('inf'), -1)
    assert gridmap.path_polylines([(0, 0), (1, 1), (2, 2)], step=2) == [[(0.0, 0.0), (2.0, 2.0)]]
    assert gridmap.path_polylines([]) == []


def test_grid_rejects_mismatched_data():
    try:
        gridmap.Grid.from_occupancy(4, 4, 0.1, 0, 0, np.zeros(9))
    except ValueError:
        return
    raise AssertionError('expected ValueError for a 9-cell grid declared as 4x4')
