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

"""Dark / light styling that follows the desktop, with a manual override.

Qt 6.5+ reports the platform colour scheme; ``mode='system'`` follows it and re-applies
when the desktop flips. The stylesheet only sets what the default Fusion style gets
wrong for a dense control panel; the palette does the real work.
"""

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication

DARK = {
    'window': '#1c1f24', 'base': '#14171b', 'text': '#e6e6e6', 'subtext': '#a0a4a8',
    'accent': '#4d9de0', 'panel': '#23272e', 'border': '#333941', 'ok': '#2f9e44',
    'warn': '#e8a33d', 'fail': '#e03131', 'caption': '#8b929b',
}
LIGHT = {
    'window': '#f2f4f7', 'base': '#ffffff', 'text': '#1b1f24', 'subtext': '#4b535c',
    'accent': '#1971c2', 'panel': '#ffffff', 'border': '#d0d7de', 'ok': '#2b8a3e',
    'warn': '#b7791f', 'fail': '#c92a2a', 'caption': '#5c6670',
}

STYLESHEET = """
QMainWindow, QDialog {{ background: {window}; }}
QWidget {{ color: {text}; }}
QToolTip {{ background: {panel}; color: {text}; border: 1px solid {border}; padding: 4px; }}
QGroupBox {{ border: 1px solid {border}; border-radius: 6px; margin-top: 14px;
            padding: 8px 6px 6px 6px; font-weight: 600; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px;
                   color: {subtext}; }}
QDockWidget {{ color: {subtext}; titlebar-close-icon: none; titlebar-normal-icon: none; }}
QDockWidget::title {{ background: {panel}; padding: 5px 8px; border: 1px solid {border};
                     border-bottom: none; }}
QTabWidget::pane {{ border: 1px solid {border}; border-radius: 4px; }}
QTabBar::tab {{ background: transparent; padding: 5px 10px; color: {subtext}; }}
QTabBar::tab:selected {{ color: {text}; border-bottom: 2px solid {accent}; }}
QPushButton {{ background: {panel}; border: 1px solid {border}; border-radius: 5px;
              padding: 5px 10px; }}
QPushButton:hover {{ border-color: {accent}; }}
QPushButton:disabled {{ color: {caption}; }}
QPushButton#primary {{ background: {accent}; color: #ffffff; border: none; font-weight: 600; }}
QPushButton#danger {{ background: {fail}; color: #ffffff; border: none; font-weight: 700; }}
QPushButton#danger:checked {{ background: #ff2020; border: 2px solid #ffd43b; }}
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QPlainTextEdit, QTextEdit {{
    background: {base}; border: 1px solid {border}; border-radius: 4px; padding: 3px 6px;
    selection-background-color: {accent}; }}
QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled {{
    color: {caption}; background: {window}; }}
QCheckBox::indicator, QRadioButton::indicator {{ width: 14px; height: 14px; }}
QHeaderView::section {{ background: {panel}; border: none; border-bottom: 1px solid {border};
                       padding: 4px; color: {subtext}; }}
QTableWidget, QTreeWidget, QListWidget {{ background: {base}; border: 1px solid {border};
                                        gridline-color: {border}; }}
QListWidget::item:selected, QTableWidget::item:selected {{ background: {accent}; color: #fff; }}
QLabel#caption {{ color: {caption}; font-size: 9px; letter-spacing: 1px; }}
QLabel#value {{ font-size: 15px; font-weight: 600; }}
QLabel#section {{ color: {subtext}; font-size: 10px; font-weight: 700; letter-spacing: 1px;
                 margin-top: 6px; }}
QLabel#hint {{ color: {caption}; font-size: 11px; }}
QLabel#badge {{ background: {warn}; color: #1b1b1b; border-radius: 7px; padding: 1px 8px;
               font-size: 10px; font-weight: 700; }}
QLabel#badgeOk {{ background: transparent; color: {caption}; padding: 1px 8px; font-size: 10px; }}
QFrame#hline {{ color: {border}; max-height: 1px; }}
QSplitter::handle {{ background: {border}; }}
QScrollBar:vertical {{ background: {window}; width: 10px; }}
QScrollBar::handle:vertical {{ background: {border}; border-radius: 5px; min-height: 20px; }}
"""


def _palette(colors):
    pal = QPalette()
    pal.setColor(QPalette.ColorRole.Window, QColor(colors['window']))
    pal.setColor(QPalette.ColorRole.WindowText, QColor(colors['text']))
    pal.setColor(QPalette.ColorRole.Base, QColor(colors['base']))
    pal.setColor(QPalette.ColorRole.AlternateBase, QColor(colors['panel']))
    pal.setColor(QPalette.ColorRole.Text, QColor(colors['text']))
    pal.setColor(QPalette.ColorRole.Button, QColor(colors['panel']))
    pal.setColor(QPalette.ColorRole.ButtonText, QColor(colors['text']))
    pal.setColor(QPalette.ColorRole.Highlight, QColor(colors['accent']))
    pal.setColor(QPalette.ColorRole.HighlightedText, QColor('#ffffff'))
    pal.setColor(QPalette.ColorRole.ToolTipBase, QColor(colors['panel']))
    pal.setColor(QPalette.ColorRole.ToolTipText, QColor(colors['text']))
    pal.setColor(QPalette.ColorRole.PlaceholderText, QColor(colors['caption']))
    pal.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, QColor(colors['caption']))
    pal.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText,
                 QColor(colors['caption']))
    return pal


def system_scheme(app):
    """'dark' or 'light' as the desktop reports it (defaults to dark)."""
    try:
        scheme = app.styleHints().colorScheme()
        if scheme == Qt.ColorScheme.Light:
            return 'light'
        if scheme == Qt.ColorScheme.Dark:
            return 'dark'
    except AttributeError:
        pass
    return 'dark'


def resolved(app, mode):
    return system_scheme(app) if str(mode).lower() not in ('dark', 'light') else str(mode).lower()


def apply(app, mode='system'):
    """Apply the theme; returns the concrete mode that was used."""
    chosen = resolved(app, mode)
    colors = DARK if chosen == 'dark' else LIGHT
    app.setPalette(_palette(colors))
    app.setStyleSheet(STYLESHEET.format(**colors))
    return chosen


def connect_system_theme(app, callback):
    """Call ``callback()`` when the desktop switches between light and dark."""
    try:
        app.styleHints().colorSchemeChanged.connect(lambda *_: callback())
    except AttributeError:
        pass
