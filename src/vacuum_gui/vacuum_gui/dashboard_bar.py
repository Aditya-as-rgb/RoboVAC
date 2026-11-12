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

"""The status bar strip: sim time, RTF, coverage, mission, dirt, speed, localisation."""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QWidget

from . import status
from .ui_common import Dot, LabeledValue


class DashboardBar(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('dashboard')
        layout = QHBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(2)

        self.sim_time = LabeledValue('sim time', '--:--:--', 'Gazebo clock (/clock)')
        self.rtf = LabeledValue('real-time', '-- %', 'Sim seconds per wall second')
        self.coverage = LabeledValue('covered (reachable)', '-- %',
                                     '/coverage/percent: share of floor the robot can reach')
        self.coverage_free = LabeledValue('covered (all free)', '-- %',
                                          '/coverage/percent_free: counts floor it cannot reach')
        self.mission = LabeledValue('mission', 'idle', '/coverage/status')
        self.dirt = LabeledValue('dirt left', '--', '/dirt/remaining vs /dirt/total')
        self.detected = LabeledValue('detected', '--',
                                     'confirmed spots on /dirt/detections')
        self.speed = LabeledValue('speed', '-- m/s', '/odom twist')
        self.localisation = LabeledValue('localisation', 'unknown',
                                         'from /amcl_pose covariance')
        self.nav2 = LabeledValue('Nav2', 'unknown', 'lifecycle state of the Nav2 servers')

        self.localisation_dot = Dot()
        self.nav2_dot = Dot()
        localisation_box = QWidget()
        localisation_layout = QHBoxLayout(localisation_box)
        localisation_layout.setContentsMargins(0, 0, 0, 0)
        localisation_layout.setSpacing(0)
        localisation_layout.addWidget(self.localisation)
        localisation_layout.addWidget(self.localisation_dot)
        nav2_box = QWidget()
        nav2_layout = QHBoxLayout(nav2_box)
        nav2_layout.setContentsMargins(0, 0, 0, 0)
        nav2_layout.setSpacing(0)
        nav2_layout.addWidget(self.nav2)
        nav2_layout.addWidget(self.nav2_dot)

        for widget in (self.sim_time, self.rtf, self.coverage, self.coverage_free,
                       self.mission, self.dirt, self.detected, self.speed,
                       localisation_box, nav2_box):
            layout.addWidget(widget)
            line = QFrame()
            line.setFrameShape(QFrame.Shape.VLine)
            line.setObjectName('hline')
            layout.addWidget(line)
        layout.addStretch(1)

        self.setStyleSheet('QFrame#dashboard { border-top: 1px solid #333941; }')

    def update_dashboard(self, dash):
        self.sim_time.set_value(status.fmt_clock(dash.sim_time))
        rtf = dash.rtf
        self.rtf.set_value('-- %' if rtf <= 0.0 else '%.0f %%' % (100.0 * rtf),
                           'sim time is running at %.2f x wall time' % rtf)
        self.coverage.set_value('%.1f %%' % dash.coverage_pct,
                                '/coverage/percent = %.2f (reachable floor)' % dash.coverage_pct)
        self.coverage_free.set_value('%.1f %%' % dash.coverage_free_pct,
                                     '/coverage/percent_free = %.2f (all free floor)'
                                     % dash.coverage_free_pct)
        mission = dash.mission_status or 'idle'
        self.mission.set_value(mission if len(mission) <= 60 else mission[:57] + '...',
                               mission)
        if dash.dirt_total >= 0:
            self.dirt.set_value('%d / %d' % (max(dash.dirt_remaining, 0), dash.dirt_total))
        else:
            self.dirt.set_value('--')
        self.detected.set_value('%d' % dash.detected_spots)
        self.speed.set_value('%.2f m/s' % dash.speed)
        self.localisation.set_value(dash.localisation)
        self.localisation_dot.set_state({'localised': status.OK, 'weak': status.WARN,
                                         'lost': status.FAIL}.get(dash.localisation,
                                                                  status.UNKNOWN))
        self.nav2.set_value(dash.nav2_state)
        self.nav2_dot.set_state(status.OK if dash.nav2_state == 'active'
                                else status.WARN if dash.nav2_state == 'partially active'
                                else status.FAIL if dash.nav2_state not in ('unknown', 'idle')
                                else status.UNKNOWN)
        self.localisation.setToolTip(
            'localised: AMCL covariance is small\nweak: covariance is growing\n'
            'lost: no /amcl_pose for 15 s or covariance is huge\n'
            'unknown: no AMCL (slam:=True) or no message yet')
