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

"""A tiny coverage-vs-time plot painted with QPainter (no QtCharts dependency)."""

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QSizePolicy, QWidget

SERIES_COLORS = (QColor('#4d9de0'), QColor('#9aa4af'))
LABELS = ('reachable floor', 'all free floor')


class CoveragePlot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.series = []
        self.setMinimumHeight(150)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setToolTip('coverage.csv: cleaned %% over mission time')

    def set_series(self, series_list):
        """``series_list`` = [{'label': str, 't': [...], 'y': [...]}, ...]."""
        self.series = [s for s in series_list if s.get('t')]
        self.update()

    def clear(self):
        self.series = []
        self.update()

    def paintEvent(self, event):      # noqa: N802
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.rect()
        painter.fillRect(rect, QColor('#14171b'))
        margin_left, margin_right, margin_top, margin_bottom = 44, 10, 18, 26
        plot = QRectF(margin_left, margin_top,
                      max(10.0, rect.width() - margin_left - margin_right),
                      max(10.0, rect.height() - margin_top - margin_bottom))
        painter.setPen(QPen(QColor('#333941'), 1))
        painter.drawRect(plot)
        font = QFont()
        font.setPointSize(8)
        painter.setFont(font)
        painter.setPen(QColor('#8b929b'))
        for pct in (0, 25, 50, 75, 100):
            y = plot.bottom() - plot.height() * pct / 100.0
            painter.setPen(QPen(QColor('#262b31'), 1, Qt.PenStyle.DotLine))
            painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
            painter.setPen(QColor('#8b929b'))
            painter.drawText(QRectF(2, y - 7, margin_left - 6, 14),
                             Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                             '%d' % pct)
        if not self.series:
            painter.setPen(QColor('#8b929b'))
            painter.drawText(plot, Qt.AlignmentFlag.AlignCenter,
                             'pick a run with a coverage.csv')
            painter.end()
            return
        t_max = max(max(s['t']) for s in self.series) or 1.0
        for index, series in enumerate(self.series[:2]):
            t = series['t']
            y = series.get('y') or []
            if len(t) < 2:
                continue
            polygon = QPolygonF()
            for ti, yi in zip(t, y):
                px = plot.left() + plot.width() * (ti / t_max)
                py = plot.bottom() - plot.height() * (max(0.0, min(100.0, yi)) / 100.0)
                polygon.append(QPointF(px, py))
            painter.setPen(QPen(SERIES_COLORS[index % 2], 2.0))
            painter.drawPolyline(polygon)
            painter.setBrush(SERIES_COLORS[index % 2])
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawEllipse(polygon[-1], 3, 3)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.setPen(QColor('#c0c6cc'))
            painter.drawText(QRectF(plot.left() + 8 + index * 120, plot.top() + 2, 118, 14),
                             Qt.AlignmentFlag.AlignLeft,
                             series.get('label', LABELS[index % 2]))
        painter.setPen(QColor('#8b929b'))
        painter.drawText(QRectF(plot.left(), plot.bottom() + 4, plot.width(), 16),
                         Qt.AlignmentFlag.AlignCenter,
                         'time 0 - %.1f min' % (t_max / 60.0))
        painter.end()
