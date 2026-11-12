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

"""Click a detected spot: its kind, confidence, hits and the nearest ground-truth patch."""

import math

from PySide6.QtWidgets import (QDialog, QDialogButtonBox, QFormLayout, QLabel,
                               QVBoxLayout)


class DirtInspector(QDialog):
    def __init__(self, detection, truth, distance, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Dirt inspector')
        self.resize(420, 260)
        layout = QVBoxLayout(self)
        form = QFormLayout()
        if detection is None:
            form.addRow('Detected', QLabel('nothing detected within 40 cm of the click'))
        else:
            form.addRow('Detected', QLabel('#%d  %s' % (detection.id, detection.kind)))
            form.addRow('Confidence', QLabel('%.3f' % detection.conf))
            form.addRow('Hits', QLabel('%d frame(s) confirmed it' % detection.hits))
            form.addRow('Position', QLabel('(%.3f, %.3f) m' % (detection.x, detection.y)))
        if truth is None:
            form.addRow('Ground truth', QLabel('no /dirt/markers patch nearby'))
        else:
            left = detection is not None
            error = distance * 100.0 if left else float('nan')
            form.addRow('Nearest patch', QLabel('%s at (%.3f, %.3f) m'
                                                % (truth[2] or 'patch', truth[0], truth[1])))
            form.addRow('Error', QLabel('%.1f cm' % error if left and not math.isnan(error)
                                        else '--'))
        layout.addLayout(form)
        note = QLabel('Ground truth comes from /dirt/markers (what Gazebo actually placed); '
                      'it is the answer key, not what the camera sees.')
        note.setWordWrap(True)
        note.setObjectName('hint')
        layout.addWidget(note)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
