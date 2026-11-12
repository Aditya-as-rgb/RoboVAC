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

"""Small conversions from the plain payloads to Qt images (kept apart from gridmap.py)."""

from PySide6.QtGui import QImage, QPixmap


def image_from_frame(frame):
    """model.ImageFrame -> QImage; ``.copy()`` so Qt owns the pixels after the signal."""
    img = QImage(frame.rgba, frame.width, frame.height, frame.width * 4,
                 QImage.Format.Format_RGBA8888)
    return img.copy()


def pixmap_from_rgba(rgba, width, height):
    img = QImage(rgba, width, height, width * 4, QImage.Format.Format_RGBA8888)
    return QPixmap.fromImage(img.copy())
