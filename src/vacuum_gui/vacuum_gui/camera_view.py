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

"""Camera and detector-debug views with a real FPS counter and a "no signal" state."""

import time

from PySide6.QtCore import QRect, Qt
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen
from PySide6.QtWidgets import QLabel, QSizePolicy, QTabWidget, QVBoxLayout, QWidget

from . import qtimages


class CameraView(QWidget):
    """One image stream, letterboxed, with its own FPS and a stale-image notice."""

    def __init__(self, title, stale_after=1.5, parent=None):
        super().__init__(parent)
        self.title = title
        self.stale_after = float(stale_after)
        self._image = None
        self._stamp = 0.0
        self._frames = 0
        self._fps = 0.0
        self._window_start = time.monotonic()
        self._reason = 'no signal yet'
        self.setMinimumSize(160, 110)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_reason(self, reason):
        if reason != self._reason:
            self._reason = reason
            self.update()

    def set_frame(self, frame):
        self._image = qtimages.image_from_frame(frame)
        self._stamp = time.monotonic()
        self._frames += 1
        now = time.monotonic()
        if now - self._window_start >= 1.0:
            self._fps = self._frames / (now - self._window_start)
            self._frames = 0
            self._window_start = now
        self.update()

    def fps(self):
        if self._image is None or time.monotonic() - self._stamp > self.stale_after:
            return 0.0
        return self._fps

    def clear(self, reason='stream stopped'):
        self._image = None
        self._reason = reason
        self._fps = 0.0
        self.update()

    def grab_image(self):
        return None if self._image is None else self._image.copy()

    def paintEvent(self, event):      # noqa: N802
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#0d0f12'))
        painter.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        rect = self.rect()
        if self._image is not None and time.monotonic() - self._stamp <= self.stale_after:
            scaled = self._image.size().scaled(rect.size(), Qt.AspectRatioMode.KeepAspectRatio)
            target = QRect(0, 0, scaled.width(), scaled.height())
            target.moveCenter(rect.center())
            painter.drawImage(target, self._image)
            label = '%s  %dx%d  %.1f fps' % (self.title, self._image.width(),
                                             self._image.height(), self._fps)
            color = QColor(220, 230, 240)
        else:
            reason = self._reason
            if self._image is not None:
                reason = 'no signal for %.1f s' % (time.monotonic() - self._stamp)
            painter.setPen(QColor('#8b929b'))
            font = QFont()
            font.setPointSize(11)
            painter.setFont(font)
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, reason)
            label = '%s  no signal' % self.title
            color = QColor(224, 49, 49)
        painter.setPen(QPen(color))
        font = QFont()
        font.setPointSize(8)
        painter.setFont(font)
        painter.drawText(self.rect().adjusted(6, 4, -6, -4),
                         Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft, label)
        painter.end()

    def grab_png(self, path):
        return self.grab().save(path, 'PNG')


class CameraPanel(QWidget):
    """Tabs for /camera/image and /dirt/debug_image, plus a reason line."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self.tabs = QTabWidget(self)
        self.camera = CameraView('camera/image', parent=self)
        self.debug = CameraView('dirt/debug_image', parent=self)
        self.tabs.addTab(self.camera, 'Image')
        self.tabs.addTab(self.debug, 'Boxes')
        self.reason = QLabel('waiting for the stack')
        self.reason.setObjectName('hint')
        self.reason.setWordWrap(True)
        self.reason.setContentsMargins(8, 2, 8, 2)
        layout.addWidget(self.tabs, 1)
        layout.addWidget(self.reason)

    def on_image(self, kind, frame):
        if kind == 'camera':
            self.camera.set_frame(frame)
            self.camera.set_reason('')
        else:
            self.debug.set_frame(frame)
            self.debug.set_reason('')

    def set_reason(self, text):
        self.reason.setText(text)
        for view in (self.camera, self.debug):
            view.set_reason(text or 'no signal yet')

    def active_view(self):
        return self.debug if self.tabs.currentIndex() == 1 else self.camera

    def grab_png(self, path):
        return self.active_view().grab_png(path)
