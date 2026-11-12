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

"""Payloads exchanged between the ROS thread and the Qt widgets.

Plain dataclasses, no ROS and no Qt, so the panels can be tested against fake data.
"""

from dataclasses import dataclass, field
from typing import Optional


@dataclass(frozen=True)
class ImageFrame:
    """An RGBA8888 image, row 0 at the top (the layout QImage expects)."""

    width: int
    height: int
    rgba: bytes
    kind: str = 'camera'            # 'camera' | 'debug'
    stamp: float = 0.0
    encoding: str = 'rgb8'


@dataclass(frozen=True)
class PoseEstimate:
    x: float
    y: float
    yaw: float
    stamp: float = 0.0
    source: str = 'tf'              # 'tf' | 'amcl' | 'odom'


@dataclass(frozen=True)
class ScanData:
    points: tuple = ()              # ((x, y), ...) in the map frame
    robot_x: float = 0.0
    robot_y: float = 0.0
    range_min: float = 0.0
    range_max: float = 0.0
    stamp: float = 0.0


@dataclass(frozen=True)
class PathData:
    polylines: tuple = ()           # ([(x, y), ...], ...) in the map frame
    name: str = 'global'
    stamp: float = 0.0


@dataclass(frozen=True)
class MapMarker:
    x: float
    y: float
    kind: str = 'spot'              # 'spot' | 'polyline' | 'arrow' | 'text'
    label: str = ''
    color: tuple = (200, 200, 200, 255)
    points: tuple = ()              # polyline vertices in the map frame
    size: float = 0.0


@dataclass(frozen=True)
class MarkerSet:
    markers: tuple = ()
    name: str = 'markers'
    stamp: float = 0.0


@dataclass(frozen=True)
class Detection:
    id: int
    x: float
    y: float
    kind: str
    conf: float
    hits: int
    stamp: float = 0.0


@dataclass(frozen=True)
class Detections:
    spots: tuple = ()
    stamp: float = 0.0


@dataclass(frozen=True)
class ParticleCloud:
    poses: tuple = ()               # ((x, y, yaw), ...)
    stamp: float = 0.0


@dataclass(frozen=True)
class JoyState:
    connected: bool = False
    axes: tuple = ()
    buttons: tuple = ()
    stamp: float = 0.0


@dataclass(frozen=True)
class Dashboard:
    sim_time: float = 0.0
    rtf: float = 0.0
    coverage_pct: float = 0.0
    coverage_free_pct: float = 0.0
    mission_status: str = 'no mission'
    dirt_remaining: int = -1
    dirt_total: int = -1
    detected_spots: int = 0
    detector_rate: float = 0.0
    camera_rate: float = 0.0
    speed: float = 0.0
    localisation: str = 'unknown'
    nav2_state: str = 'unknown'
    pose: Optional[PoseEstimate] = None
    stamp: float = 0.0


@dataclass(frozen=True)
class Nav2Status:
    states: dict = field(default_factory=dict)   # name -> 'active' / 'inactive' / 'absent'
    stamp: float = 0.0
