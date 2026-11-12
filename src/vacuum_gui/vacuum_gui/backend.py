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

"""The small interface the panels talk to.

Everything a widget needs from the robot is a signal on :class:`RobotBackend` plus one
of the command methods; no widget imports rclpy. ``RosBackend`` implements it against
the simulation today, and a real-robot backend (STM32 + RPLidar over serial or micro-ROS)
can implement the same signals later: it would publish ``map_received`` from a saved map,
``pose_received`` from the wheel odometry + LiDAR, and answer the same commands.

``NullBackend`` is the fallback when rclpy is missing or ROS is not sourced - the panels
greys themselves out instead of the window failing to open.
"""

from PySide6.QtCore import QObject, Signal

CAP_MAP = 'map'
CAP_CAMERA = 'camera'
CAP_TELEOP = 'teleop'
CAP_MISSION = 'mission'
CAP_DETECTOR = 'detector'
CAP_ACTIONS = 'actions'
CAP_PARAMS = 'params'

ALL_CAPABILITIES = {CAP_MAP, CAP_CAMERA, CAP_TELEOP, CAP_MISSION, CAP_DETECTOR,
                    CAP_ACTIONS, CAP_PARAMS}


class RobotBackend(QObject):
    """Signals carry :mod:`vacuum_gui.model` dataclasses; see the class docstring."""

    connected = Signal(bool)                 # ROS node up / down
    map_received = Signal(object)            # model/Grid
    coverage_received = Signal(object)       # model/Grid (cleaned-area overlay)
    costmap_received = Signal(str, object)   # 'global'|'local', model/Grid
    image_received = Signal(str, object)     # 'camera'|'debug', model/ImageFrame
    pose_received = Signal(object)           # model/PoseEstimate
    scan_received = Signal(object)           # model/ScanData
    path_received = Signal(str, object)      # 'global'|'local', model/PathData
    markers_received = Signal(str, object)   # 'coverage'|'dirt'|'detected', MarkerSet
    particles_received = Signal(object)      # model/ParticleCloud
    detections_received = Signal(object)     # model/Detections
    joy_received = Signal(object)            # model/JoyState
    dashboard_received = Signal(object)      # model/Dashboard
    health_received = Signal(object)         # list[status.HealthItem]
    nav2_received = Signal(object)           # model/Nav2Status
    mission_status_received = Signal(str)
    detector_params_received = Signal(dict)
    action_state_received = Signal(str)      # 'idle'|'navigating'|'succeeded'|'failed'
    notice = Signal(str, str)                # level ('info'|'warn'|'error'), text

    capabilities = frozenset(ALL_CAPABILITIES)

    # ------------------------------------------------------------- lifecycle --
    def start(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def available(self):
        return False

    def reason(self):
        return 'no ROS backend'

    def set_expectations(self, expectations):
        """Tell the backend which parts of the stack should be running."""

    def set_image_rate(self, hz):
        pass

    def set_scan_rate(self, hz):
        pass

    # -------------------------------------------------------------- commands --
    def request_initial_pose(self, x, y, yaw):
        pass

    def send_goal(self, x, y, yaw=0.0):
        pass

    def cancel_nav_goal(self):
        pass

    def publish_cmd_vel(self, linear, angular):
        pass

    def call_trigger(self, service):
        """``service`` is one of 'plan' | 'start' | 'stop' | 'reset_meter'."""

    def set_detector_params(self, values):
        pass

    def request_detector_params(self):
        pass


class NullBackend(RobotBackend):
    """Used when rclpy is unavailable: every panel shows a reason, nothing crashes."""

    capabilities = frozenset()

    def __init__(self, reason='ROS is not available', parent=None):
        super().__init__(parent)
        self._reason = reason

    def available(self):
        return False

    def reason(self):
        return self._reason

    def start(self):
        self.notice.emit('error', self._reason)

    def stop(self):
        pass
